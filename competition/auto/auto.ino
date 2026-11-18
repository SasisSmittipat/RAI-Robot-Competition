#include <Wire.h>
#include <EEPROM.h>
#include <VL53L0X.h>

const byte PIN_ENA = 5;
const byte PIN_IN1 = 7;
const byte PIN_IN2 = 8;
const byte PIN_IN3 = 9;
const byte PIN_IN4 = 10;
const byte PIN_ENB = 6;
const byte PIN_XSHUT_FRONT = 2;
const byte PIN_XSHUT_LEFT  = 3;
const byte PIN_XSHUT_RIGHT = 4;
const byte PIN_TAPE = A0;
const byte PIN_LED = 13;

const bool LEFT_MOTOR_REVERSED  = true;
const bool RIGHT_MOTOR_REVERSED = true;

const int   BASE_SPEED     = 90;
const int   MIN_DRIVE_PWM  = 80;
const int   TRIM           = 3;
const float KP_WALL        = 0.4;
const float KP_HEADING     = 3.0;
const int   MAX_STEER      = 20;
const int   DEAD_ZONE_MM   = 6;
const int   SIDE_TARGET_MM = 62;
const int   WALL_MM        = 110;

const bool  STALL_PROTECTION = true;
const int   KICK_POWER     = 150;
const unsigned long KICK_MS = 50;
const unsigned long STALL_CHECK_MS = 800;
const int   STALL_MIN_MM   = 10;
const int   STALL_STEP     = 15;
const int   MAX_BOOST      = 60;
const int   STALL_RANGE_MM = 1000;

const int   FRONT_STOP_MM        = 130;
const int   JUNCTION_ADVANCE_MM  = 0;
const int   POST_TURN_ADVANCE_MM = 90;
const int   FRONT_ODO_MAX_MM     = 600;
const float MS_PER_MM            = 7.0;

const bool  USE_CLOSE_TURN = true;
const int   CLOSE_TURN_MM  = 30;
const int   NEAR_WALL_MM   = 250;
const unsigned long APPROACH_MAX_MS = 1500;

const byte  OPEN_CONFIRM  = 2;
const byte  WALL_CONFIRM  = 3;
const byte  FRONT_CONFIRM = 2;
const unsigned long REARM_MS = 1500;
const unsigned long SENSOR_BUDGET_US = 20000;

const int   TURN_SPEED    = 140;
const int   TURN_SLOW     = 110;
const float SLOW_ZONE_DEG = 25.0;
const float FINE_TOLERANCE_DEG = 2.0;

const int   NUDGE_POWER = 120;
const unsigned long NUDGE_MS = 120;

const int   REVERSE_POWER = 130;
const int   REVERSE_MS    = 40;

const bool  USE_TAPE          = true;
const bool  TAPE_DIGITAL      = true;
const int   TAPE_MARGIN       = 150;
const unsigned long TAPE_HOLD_MS = 10;
const int   PARK_ADVANCE_MM   = 0;
const int   PARK_FRONT_MIN_MM = 50;

const byte MPU_ADDR = 0x68;
const byte EEPROM_MAGIC = 0xA7;

VL53L0X tofFront, tofLeft, tofRight;

bool sensorsReady = false;
byte failCount = 0;
int front = 2000, left = 2000, right = 2000;
byte leftOpenCnt = 0, rightOpenCnt = 0, frontCnt = 0;
byte leftWallCnt = 0, rightWallCnt = 0;
bool leftArmed = true, rightArmed = true;
unsigned long disarmedAt = 0;

float bias[3] = {0, 0, 0};
byte  axis = 2;
float gyroSign = 1;
float yaw = 0;
float gyroRate = 0;
float coastGain = 0.05;
float savedGain = -1;
unsigned long lastMicros = 0;
unsigned long lastGoodMs = 0;
float targetHeading = 0;

unsigned long lastDriveMs = 0;
int stallRefFront = 0;
unsigned long stallRefMs = 0;
int boost = 0;

bool finished = false;
bool motorsLocked = false;
bool parked = false;
bool tapeArmed = false;
unsigned long tapeSince = 0;
int floorValue = 0;
byte printCount = 0;

void brake() {
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, 255);
  analogWrite(PIN_ENB, 255);
}

int readTape() {
  if (TAPE_DIGITAL) return digitalRead(PIN_TAPE);
  return analogRead(PIN_TAPE);
}

bool tapeDiffers() {
  int v = readTape();
  if (TAPE_DIGITAL) return v != floorValue;
  return abs(v - floorValue) > TAPE_MARGIN;
}

void checkTape() {
  if (!USE_TAPE || !tapeArmed || finished) return;
  if (tapeDiffers()) {
    if (tapeSince == 0) {
      tapeSince = millis();
    } else if (millis() - tapeSince >= TAPE_HOLD_MS) {
      brake();
      motorsLocked = true;
      finished = true;
    }
  } else {
    tapeSince = 0;
  }
}

void setOneMotor(byte pinEn, byte pinA, byte pinB, int power, bool reversed) {
  bool forward = (power >= 0) != reversed;
  digitalWrite(pinA, forward ? HIGH : LOW);
  digitalWrite(pinB, forward ? LOW : HIGH);
  analogWrite(pinEn, constrain(abs(power), 0, 255));
}

void setMotors(int leftPower, int rightPower) {
  if (motorsLocked) return;
  setOneMotor(PIN_ENA, PIN_IN1, PIN_IN2, leftPower, LEFT_MOTOR_REVERSED);
  setOneMotor(PIN_ENB, PIN_IN3, PIN_IN4, rightPower, RIGHT_MOTOR_REVERSED);
}

void loadGain() {
  if (EEPROM.read(0) == EEPROM_MAGIC) {
    float g;
    EEPROM.get(1, g);
    if (g >= 0.0 && g <= 0.4) {
      coastGain = g;
      savedGain = g;
    }
  }
}

void saveGain() {
  if (abs(coastGain - savedGain) < 0.004) return;
  EEPROM.write(0, EEPROM_MAGIC);
  EEPROM.put(1, coastGain);
  savedGain = coastGain;
}

bool gyroPresent() {
  Wire.beginTransmission(MPU_ADDR);
  return Wire.endTransmission() == 0;
}

void gyroWrite(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readGyro(int16_t v[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (byte)6) != 6) return false;
  for (byte i = 0; i < 3; i++) {
    byte hi = Wire.read();
    byte lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

bool updateGyro() {
  checkTape();
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6;
  lastMicros = now;
  int16_t v[3];
  if (!readGyro(v)) return false;
  gyroRate = gyroSign * ((v[axis] - bias[axis]) / 32.8);
  yaw += gyroRate * dt;
  lastGoodMs = millis();
  return true;
}

bool gyroAlive() {
  return millis() - lastGoodMs < 150;
}

void waitWithGyro(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms && !finished) updateGyro();
}

void startGyro() {
  setMotors(0, 0);
  while (!gyroPresent()) {
    Serial.println(F("Gyro not answering - check its SDA and SCL wires"));
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(100);
  }
  gyroWrite(0x6B, 0x00);
  gyroWrite(0x1B, 0x10);
  gyroWrite(0x1A, 0x03);
  delay(100);

  digitalWrite(PIN_LED, HIGH);
  Serial.println(F("Calibrating - keep the robot still"));
  delay(1000);
  long sum[3] = {0, 0, 0};
  int count = 0;
  for (int i = 0; i < 500; i++) {
    int16_t v[3];
    if (readGyro(v)) {
      for (byte k = 0; k < 3; k++) sum[k] += v[k];
      count++;
    }
    delay(3);
  }
  if (count > 0) {
    for (byte k = 0; k < 3; k++) bias[k] = (float)sum[k] / count;
  }
  digitalWrite(PIN_LED, LOW);
  yaw = 0;
  gyroRate = 0;
  targetHeading = 0;
  lastMicros = micros();
  lastGoodMs = millis();
}

void integrateAllAxes(float angle[3], unsigned long ms, unsigned long brakeAfterMs) {
  unsigned long t0 = millis();
  unsigned long last = micros();
  bool braked = false;
  while (millis() - t0 < ms) {
    if (!braked && millis() - t0 >= brakeAfterMs) {
      brake();
      braked = true;
    }
    unsigned long now = micros();
    float dt = (now - last) * 1e-6;
    last = now;
    int16_t v[3];
    if (readGyro(v)) {
      for (byte k = 0; k < 3; k++) angle[k] += ((v[k] - bias[k]) / 32.8) * dt;
    }
  }
  setMotors(0, 0);
}

void findAxisAndDirection() {
  float angle[3] = {0, 0, 0};

  setMotors(-NUDGE_POWER, NUDGE_POWER);
  integrateAllAxes(angle, NUDGE_MS + 250, NUDGE_MS);

  axis = 0;
  for (byte k = 1; k < 3; k++) {
    if (abs(angle[k]) > abs(angle[axis])) axis = k;
  }
  gyroSign = (angle[axis] >= 0) ? 1 : -1;

  setMotors(NUDGE_POWER, -NUDGE_POWER);
  integrateAllAxes(angle, NUDGE_MS + 250, NUDGE_MS);

  yaw = gyroSign * angle[axis];
  gyroRate = 0;
  lastMicros = micros();
  lastGoodMs = millis();

  Serial.print(F("Gyro axis "));
  Serial.print(axis == 0 ? F("X") : axis == 1 ? F("Y") : F("Z"));
  Serial.print(F("  leftover angle after wiggle "));
  Serial.println(yaw, 1);
}

bool turnTo(float goal) {
  float dir = (goal > yaw) ? 1 : -1;
  float rateAtCut = 0;
  unsigned long limit = (abs(goal - yaw) > 100) ? 5000 : 3000;
  unsigned long t0 = millis();
  while (millis() - t0 < limit) {
    if (finished) return true;
    if (!updateGyro()) {
      if (!gyroAlive()) {
        setMotors(0, 0);
        Serial.println(F("STOP REASON: gyro lost during turn - recalibrating"));
        return false;
      }
      continue;
    }
    float remaining = (goal - yaw) * dir;
    float lead = max(2.0, coastGain * abs(gyroRate));
    if (remaining <= lead) {
      rateAtCut = abs(gyroRate);
      break;
    }
    int power = (remaining > SLOW_ZONE_DEG) ? TURN_SPEED : TURN_SLOW;
    setMotors(-dir * power, dir * power);
  }
  if (finished) return true;
  brake();
  waitWithGyro(350);
  if (finished) return true;

  float overshoot = (yaw - goal) * dir;
  if (rateAtCut > 30) {
    coastGain = constrain(coastGain + 0.6 * overshoot / rateAtCut, 0.0, 0.4);
    saveGain();
  }

  for (byte i = 0; i < 8; i++) {
    if (finished) return true;
    float err = goal - yaw;
    if (abs(err) <= FINE_TOLERANCE_DEG) break;
    float d = (err > 0) ? 1 : -1;
    setMotors(-d * TURN_SLOW, d * TURN_SLOW);
    waitWithGyro(35);
    brake();
    waitWithGyro(120);
  }
  setMotors(0, 0);
  return true;
}

void turnBy(float degrees) {
  brake();
  waitWithGyro(150);
  if (finished) return;
  float goal = targetHeading + degrees;
  if (turnTo(goal)) {
    targetHeading = goal;
  } else {
    startGyro();
  }
}

void hardStop() {
  setMotors(-REVERSE_POWER, -REVERSE_POWER);
  waitWithGyro(REVERSE_MS);
  brake();
  waitWithGyro(250);
  setMotors(0, 0);
}

bool startOne(VL53L0X &sensor, byte xshutPin, byte address) {
  pinMode(xshutPin, INPUT);
  delay(10);
  sensor.setAddress(0x29);
  sensor.setTimeout(100);
  if (!sensor.init()) return false;
  sensor.setAddress(address);
  sensor.setMeasurementTimingBudget(SENSOR_BUDGET_US);
  sensor.startContinuous();
  return true;
}

byte startAllSensors() {
  pinMode(PIN_XSHUT_FRONT, OUTPUT); digitalWrite(PIN_XSHUT_FRONT, LOW);
  pinMode(PIN_XSHUT_LEFT,  OUTPUT); digitalWrite(PIN_XSHUT_LEFT,  LOW);
  pinMode(PIN_XSHUT_RIGHT, OUTPUT); digitalWrite(PIN_XSHUT_RIGHT, LOW);
  delay(20);
  if (!startOne(tofFront, PIN_XSHUT_FRONT, 0x30)) return 1;
  if (!startOne(tofLeft,  PIN_XSHUT_LEFT,  0x31)) return 2;
  if (!startOne(tofRight, PIN_XSHUT_RIGHT, 0x32)) return 3;
  return 0;
}

bool readOne(VL53L0X &sensor, int &mm) {
  checkTape();
  uint16_t r = sensor.readRangeContinuousMillimeters();
  checkTape();
  if (sensor.timeoutOccurred()) return false;
  mm = (r >= 2000) ? 2000 : (int)r;
  return true;
}

void resetCounts() {
  leftOpenCnt = 0;
  rightOpenCnt = 0;
  frontCnt = 0;
}

void disarmSides() {
  leftArmed = false;
  rightArmed = false;
  leftWallCnt = 0;
  rightWallCnt = 0;
  disarmedAt = millis();
}

bool readAll() {
  updateGyro();
  int f, l, r;
  bool ok = readOne(tofFront, f);
  ok = readOne(tofLeft, l) && ok;
  ok = readOne(tofRight, r) && ok;
  updateGyro();
  if (!ok) {
    failCount++;
    if (failCount >= 5) {
      setMotors(0, 0);
      sensorsReady = false;
      Serial.println(F("STOP REASON: a distance sensor dropped out - restarting them"));
    }
    return false;
  }
  failCount = 0;
  front = f; left = l; right = r;

  if (left > WALL_MM) {
    if (leftOpenCnt < 100) leftOpenCnt++;
    leftWallCnt = 0;
  } else {
    leftOpenCnt = 0;
    if (leftWallCnt < 100) leftWallCnt++;
    if (leftWallCnt >= WALL_CONFIRM) leftArmed = true;
  }

  if (right > WALL_MM) {
    if (rightOpenCnt < 100) rightOpenCnt++;
    rightWallCnt = 0;
  } else {
    rightOpenCnt = 0;
    if (rightWallCnt < 100) rightWallCnt++;
    if (rightWallCnt >= WALL_CONFIRM) rightArmed = true;
  }

  if (millis() - disarmedAt >= REARM_MS) {
    leftArmed = true;
    rightArmed = true;
  }

  if (front <= FRONT_STOP_MM) { if (frontCnt < 100) frontCnt++; } else frontCnt = 0;

  return true;
}

void settleSensors() {
  resetCounts();
  for (byte i = 0; i < 6 && !finished; i++) readAll();
}

bool sideOpenRaw(int side) {
  return (side == 1) ? (leftOpenCnt >= OPEN_CONFIRM) : (rightOpenCnt >= OPEN_CONFIRM);
}

bool sideOpen(int side) {
  bool armed = (side == 1) ? leftArmed : rightArmed;
  return armed && sideOpenRaw(side);
}

void driveStep(int speed, bool useWalls) {
  if (finished) return;

  if (STALL_PROTECTION) {
    if (millis() - lastDriveMs > 250) {
      setMotors(KICK_POWER, KICK_POWER);
      waitWithGyro(KICK_MS);
      if (finished) return;
      stallRefFront = front;
      stallRefMs = millis();
      boost = 0;
    }
    lastDriveMs = millis();

    if (front < STALL_RANGE_MM) {
      if (millis() - stallRefMs >= STALL_CHECK_MS) {
        if (stallRefFront - front < STALL_MIN_MM) {
          boost = min(boost + STALL_STEP, MAX_BOOST);
          Serial.print(F("Stall detected - extra power now "));
          Serial.println(boost);
        } else if (boost > 0) {
          boost = max(boost - 5, 0);
        }
        stallRefFront = front;
        stallRefMs = millis();
      }
    } else {
      stallRefFront = front;
      stallRefMs = millis();
    }
    speed += boost;
  }

  bool wallL = left <= WALL_MM;
  bool wallR = right <= WALL_MM;
  float steer;
  if (useWalls && (wallL || wallR)) {
    float error;
    if (wallL && wallR) {
      error = left - right;
      targetHeading += (yaw - targetHeading) * 0.1;
    } else if (wallL) {
      error = 2.0 * (left - SIDE_TARGET_MM);
    } else {
      error = 2.0 * (SIDE_TARGET_MM - right);
    }
    if (abs(error) < DEAD_ZONE_MM) error = 0;
    steer = KP_WALL * error;
  } else {
    steer = KP_HEADING * (targetHeading - yaw);
  }
  int s = constrain((int)steer, -MAX_STEER, MAX_STEER);
  int lp = max(speed - s + TRIM, MIN_DRIVE_PWM);
  int rp = max(speed + s - TRIM, MIN_DRIVE_PWM);
  setMotors(lp, rp);
}

void advance(int mm) {
  if (mm <= 0) return;
  unsigned long t0 = millis();
  unsigned long timeForDistance = (unsigned long)(mm * MS_PER_MM);
  bool useFront = front < FRONT_ODO_MAX_MM;
  int stopAt = max(front - mm, FRONT_STOP_MM);
  while (sensorsReady && !finished) {
    if (!readAll()) continue;
    if (finished) break;
    if (front <= FRONT_STOP_MM) break;
    if (useFront) {
      if (front <= stopAt) break;
      if (millis() - t0 > timeForDistance * 2 + 500) break;
    } else {
      if (millis() - t0 >= timeForDistance) break;
    }
    driveStep(BASE_SPEED, false);
  }
}

void approachWall() {
  unsigned long t0 = millis();
  while (sensorsReady && !finished && millis() - t0 < APPROACH_MAX_MS) {
    if (!readAll()) continue;
    if (finished) break;
    if (front <= CLOSE_TURN_MM) break;
    driveStep(BASE_SPEED, false);
  }
  if (finished) return;
  setMotors(0, 0);
  brake();
  waitWithGyro(250);
  setMotors(0, 0);
  Serial.print(F("Close to wall: front "));
  Serial.println(front);
}

void parkAndStop() {
  Serial.println(F("Black tape found"));
  if (PARK_ADVANCE_MM > 0) {
    Serial.println(F("Driving into the parking zone"));
    motorsLocked = false;
    unsigned long dur = (unsigned long)(PARK_ADVANCE_MM * MS_PER_MM);
    unsigned long t0 = millis();
    bool wasFinished = finished;
    finished = false;
    tapeArmed = false;
    while (millis() - t0 < dur) {
      if (!readAll()) break;
      if (front <= PARK_FRONT_MIN_MM) break;
      driveStep(BASE_SPEED, false);
    }
    finished = wasFinished;
  }
  brake();
  motorsLocked = true;
  Serial.println(F("PARKED"));
}

void setup() {
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TAPE, INPUT);
  setMotors(0, 0);

  Serial.begin(115200);
  Serial.println(F("=== ARDUINO STARTED (if you see this mid-run, it reset: check the battery) ==="));
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  delay(1000);

  loadGain();
  startGyro();

  long sum = 0;
  for (byte i = 0; i < 20; i++) { sum += readTape(); delay(5); }
  if (TAPE_DIGITAL) floorValue = (sum >= 10) ? 1 : 0;
  else floorValue = sum / 20;
  Serial.print(F("Floor reading on tape sensor: "));
  Serial.println(floorValue);

  findAxisAndDirection();
  if (abs(yaw) > FINE_TOLERANCE_DEG) turnTo(0);
  targetHeading = 0;
  tapeSince = 0;
  tapeArmed = true;
}

void loop() {
  if (finished) {
    if (!parked) {
      parked = true;
      parkAndStop();
    }
    brake();
    while (true) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(500);
    }
  }

  if (!sensorsReady) {
    setMotors(0, 0);
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    byte failed = startAllSensors();
    if (failed == 0) {
      Serial.println(F("All sensors OK"));
      sensorsReady = true;
      failCount = 0;
      lastMicros = micros();
      settleSensors();
    } else {
      Serial.print(F("Waiting for sensor "));
      Serial.println(failed == 1 ? F("FRONT") : failed == 2 ? F("LEFT") : F("RIGHT"));
      delay(100);
    }
    return;
  }

  if (!readAll()) return;
  if (finished) return;

  bool seesOpening = sideOpen(1) || sideOpen(-1);
  digitalWrite(PIN_LED, seesOpening ? LOW : HIGH);

  if (++printCount >= 12) {
    printCount = 0;
    Serial.print(F("F ")); Serial.print(front);
    Serial.print(F("  L ")); Serial.print(left);
    Serial.print(F("  R ")); Serial.print(right);
    Serial.print(F("  tape ")); Serial.print(readTape());
    Serial.print(F("  angle ")); Serial.print(yaw, 0);
    Serial.print(F("  boost ")); Serial.println(boost);
  }

  if (seesOpening) {
    int side = sideOpen(1) ? 1 : -1;
    if (USE_CLOSE_TURN && front <= NEAR_WALL_MM) {
      approachWall();
    } else {
      advance(JUNCTION_ADVANCE_MM);
    }
    if (finished) return;
    turnBy(90.0 * side);
    if (finished) return;
    disarmSides();
    settleSensors();
    advance(POST_TURN_ADVANCE_MM);
    resetCounts();
    return;
  }

  if (frontCnt >= FRONT_CONFIRM) {
    if (USE_CLOSE_TURN) approachWall();
    else hardStop();
    if (finished) return;
    settleSensors();
    if (sideOpenRaw(1))       turnBy(90.0);
    else if (sideOpenRaw(-1)) turnBy(-90.0);
    else                      turnBy(180.0);
    if (finished) return;
    disarmSides();
    settleSensors();
    advance(POST_TURN_ADVANCE_MM);
    resetCounts();
    return;
  }

  driveStep(BASE_SPEED, true);
}
