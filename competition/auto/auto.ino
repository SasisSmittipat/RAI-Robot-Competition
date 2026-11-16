#include <Wire.h>
#include <VL53L0X.h>

const byte MODE = 1;

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
const byte PIN_LED  = 13;

const bool LEFT_MOTOR_REVERSED  = false;
const bool RIGHT_MOTOR_REVERSED = false;
const bool GYRO_FLIPPED = false;

const int BASE_SPEED = 130;
const int SLOW_SPEED = 100;
const int TURN_SPEED = 130;
const int TURN_SLOW  = 95;
const int MIN_PWM    = 70;

const int WALL_MM              = 150;
const int FRONT_STOP_MM        = 50;
const int FRONT_SLOW_MM        = 130;
const int JUNCTION_ADVANCE_MM  = 120;
const int POST_TURN_ADVANCE_MM = 70;
const int FRONT_ODO_MAX_MM     = 600;
const float MS_PER_MM          = 6.0;

const byte OPEN_CONFIRM  = 3;
const byte FRONT_CONFIRM = 2;
const byte TAPE_CONFIRM  = 4;
const int  TAPE_MARGIN   = 150;

const float KP_HEADING     = 6.0;
const float KD_HEADING     = 0.15;
const float K_WALL         = 0.25;
const float MAX_WALL_ANGLE = 12.0;
const float K_TRIM         = 0.02;
const float TURN_STOP_EARLY = 4.0;

const unsigned long START_DELAY_MS = 1000;

VL53L0X tofFront, tofLeft, tofRight;

const byte MPU_ADDR = 0x68;

int frontMM = 9999, leftMM = 9999, rightMM = 9999;
byte leftOpenCnt = 0, rightOpenCnt = 0, frontCnt = 0;
byte freshMask = 0;

float yaw = 0;
float gyroRate = 0;
float gyroBias = 0;
float loopDt = 0;
unsigned long lastGyroMicros = 0;

float targetHeading = 0;
float sideTarget = 45;

int floorValue = 0;
int tapeValue = 0;
byte tapeCnt = 0;
unsigned long lastTapeMs = 0;
bool finished = false;

void fail(byte code, const __FlashStringHelper *msg) {
  stopMotors();
  Serial.println(msg);
  while (true) {
    for (byte i = 0; i < code; i++) {
      digitalWrite(PIN_LED, HIGH); delay(200);
      digitalWrite(PIN_LED, LOW);  delay(200);
    }
    delay(1000);
  }
}

void setOneMotor(byte pinEn, byte pinA, byte pinB, int power, bool reversed) {
  if (reversed) power = -power;
  if (power >= 0) { digitalWrite(pinA, HIGH); digitalWrite(pinB, LOW); }
  else            { digitalWrite(pinA, LOW);  digitalWrite(pinB, HIGH); }
  analogWrite(pinEn, constrain(abs(power), 0, 255));
}

void setMotors(int leftPower, int rightPower) {
  setOneMotor(PIN_ENA, PIN_IN1, PIN_IN2, leftPower,  LEFT_MOTOR_REVERSED);
  setOneMotor(PIN_ENB, PIN_IN3, PIN_IN4, rightPower, RIGHT_MOTOR_REVERSED);
}

void stopMotors() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

void brake() {
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, 255);
  analogWrite(PIN_ENB, 255);
}

void mpuWrite(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readGyroZ(int16_t &raw) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (byte)2) != 2) return false;
  byte hi = Wire.read();
  byte lo = Wire.read();
  raw = (int16_t)((hi << 8) | lo);
  return true;
}

void setupGyro() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) fail(4, F("MPU6050 not found - check wiring"));
  mpuWrite(0x6B, 0x00);
  mpuWrite(0x1B, 0x08);
  mpuWrite(0x1A, 0x03);
  delay(100);
}

void calibrateGyro() {
  long sum = 0;
  int count = 0;
  for (int i = 0; i < 500; i++) {
    int16_t raw;
    if (readGyroZ(raw)) { sum += raw; count++; }
    delay(3);
  }
  if (count == 0) fail(4, F("MPU6050 gives no data"));
  gyroBias = (float)sum / count;
  yaw = 0;
  lastGyroMicros = micros();
}

void updateGyro() {
  unsigned long now = micros();
  loopDt = (now - lastGyroMicros) * 1e-6;
  lastGyroMicros = now;
  int16_t raw;
  if (readGyroZ(raw)) {
    gyroRate = (raw - gyroBias) / 65.5;
    if (GYRO_FLIPPED) gyroRate = -gyroRate;
  }
  yaw += gyroRate * loopDt;
}

void startOneTof(VL53L0X &sensor, byte xshutPin, byte address, byte errorCode) {
  pinMode(xshutPin, INPUT);
  delay(10);
  sensor.setTimeout(100);
  if (!sensor.init()) fail(errorCode, F("A VL53L0X did not start - check wiring"));
  sensor.setAddress(address);
  sensor.startContinuous();
}

void setupTof() {
  pinMode(PIN_XSHUT_FRONT, OUTPUT); digitalWrite(PIN_XSHUT_FRONT, LOW);
  pinMode(PIN_XSHUT_LEFT,  OUTPUT); digitalWrite(PIN_XSHUT_LEFT,  LOW);
  pinMode(PIN_XSHUT_RIGHT, OUTPUT); digitalWrite(PIN_XSHUT_RIGHT, LOW);
  delay(20);
  startOneTof(tofFront, PIN_XSHUT_FRONT, 0x30, 1);
  startOneTof(tofLeft,  PIN_XSHUT_LEFT,  0x31, 2);
  startOneTof(tofRight, PIN_XSHUT_RIGHT, 0x32, 3);
}

bool pollTof(VL53L0X &sensor, int &mm) {
  if ((sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
  uint16_t r = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || r > 2000) r = 2000;
  mm = (int)r;
  return true;
}

void updateTof() {
  if (pollTof(tofFront, frontMM)) {
    freshMask |= 1;
    if (frontMM <= FRONT_STOP_MM) { if (frontCnt < 200) frontCnt++; } else frontCnt = 0;
  }
  if (pollTof(tofLeft, leftMM)) {
    freshMask |= 2;
    if (leftMM > WALL_MM) { if (leftOpenCnt < 200) leftOpenCnt++; } else leftOpenCnt = 0;
  }
  if (pollTof(tofRight, rightMM)) {
    freshMask |= 4;
    if (rightMM > WALL_MM) { if (rightOpenCnt < 200) rightOpenCnt++; } else rightOpenCnt = 0;
  }
}

bool leftOpen()     { return leftOpenCnt  >= OPEN_CONFIRM; }
bool rightOpen()    { return rightOpenCnt >= OPEN_CONFIRM; }
bool frontBlocked() { return frontCnt     >= FRONT_CONFIRM; }

void resetConfirmations() {
  leftOpenCnt = 0;
  rightOpenCnt = 0;
  frontCnt = 0;
}

void calibrateFloor() {
  long sum = 0;
  for (byte i = 0; i < 20; i++) { sum += analogRead(PIN_TAPE); delay(5); }
  floorValue = sum / 20;
}

void updateTape() {
  if (millis() - lastTapeMs < 10) return;
  lastTapeMs = millis();
  tapeValue = analogRead(PIN_TAPE);
  if (abs(tapeValue - floorValue) > TAPE_MARGIN) { if (tapeCnt < 200) tapeCnt++; }
  else tapeCnt = 0;
  if (MODE == 0 && tapeCnt >= TAPE_CONFIRM) finished = true;
}

void update() {
  updateGyro();
  updateTof();
  updateTape();
}

void settle(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) update();
}

void driveStep(int speed, bool useWalls) {
  float wallErr = 0;
  if (useWalls) {
    bool wallL = leftMM  <= WALL_MM;
    bool wallR = rightMM <= WALL_MM;
    if (wallL && wallR) {
      wallErr = (leftMM - rightMM) / 2.0;
      float k = loopDt * 2.0; if (k > 1) k = 1;
      sideTarget += ((leftMM + rightMM) / 2.0 - sideTarget) * k;
      targetHeading += K_TRIM * wallErr * loopDt;
    } else if (wallL) {
      wallErr = leftMM - sideTarget;
    } else if (wallR) {
      wallErr = sideTarget - rightMM;
    }
  }
  float wanted = targetHeading + constrain(K_WALL * wallErr, -MAX_WALL_ANGLE, MAX_WALL_ANGLE);
  float steer = KP_HEADING * (wanted - yaw) - KD_HEADING * gyroRate;
  int leftPower  = constrain((int)(speed - steer), MIN_PWM, 255);
  int rightPower = constrain((int)(speed + steer), MIN_PWM, 255);
  setMotors(leftPower, rightPower);
}

void advance(int mm) {
  unsigned long t0 = millis();
  unsigned long timeForDistance = (unsigned long)(mm * MS_PER_MM);
  bool useFront = frontMM < FRONT_ODO_MAX_MM;
  int stopAt = max(frontMM - mm, FRONT_STOP_MM);
  while (true) {
    update();
    if (finished) return;
    if (frontMM <= FRONT_STOP_MM) break;
    if (useFront) {
      if (frontMM <= stopAt) break;
      if (millis() - t0 > timeForDistance * 2 + 500) break;
    } else {
      if (millis() - t0 >= timeForDistance) break;
    }
    driveStep(SLOW_SPEED, false);
  }
}

void turnBy(float degrees) {
  brake();
  settle(150);
  if (finished) return;
  float goal = targetHeading + degrees;
  int dir = (degrees > 0) ? 1 : -1;
  unsigned long t0 = millis();
  while (millis() - t0 < 4000) {
    update();
    if (finished) return;
    float remaining = (goal - yaw) * dir;
    if (remaining <= TURN_STOP_EARLY) break;
    int power = (remaining > 25) ? TURN_SPEED : TURN_SLOW;
    setMotors(-dir * power, dir * power);
  }
  brake();
  settle(200);
  targetHeading = goal;
}

void mazeStep() {
  update();
  if (finished) return;

  if (leftOpen()) {
    advance(JUNCTION_ADVANCE_MM);
    turnBy(90);
    advance(POST_TURN_ADVANCE_MM);
    resetConfirmations();
    return;
  }

  if (frontBlocked()) {
    brake();
    resetConfirmations();
    settle(250);
    if (finished) return;
    if (rightOpen()) turnBy(-90);
    else             turnBy(180);
    advance(POST_TURN_ADVANCE_MM);
    resetConfirmations();
    return;
  }

  driveStep(frontMM < FRONT_SLOW_MM ? SLOW_SPEED : BASE_SPEED, true);
}

void printSensors() {
  static unsigned long lastPrint = 0;
  update();
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();
  Serial.print(F("Front ")); Serial.print(frontMM);
  Serial.print(F("  Left ")); Serial.print(leftMM);
  Serial.print(F("  Right ")); Serial.print(rightMM);
  Serial.print(F("  | Angle ")); Serial.print(yaw, 1);
  Serial.print(F("  | Tape ")); Serial.print(tapeValue);
  Serial.print(F(" (floor ")); Serial.print(floorValue);
  Serial.println(tapeCnt >= TAPE_CONFIRM ? F(")  BLACK") : F(")"));
}

void turnTest() {
  turnBy(90);
  Serial.print(F("After left turn: "));  Serial.println(yaw, 1);
  stopMotors(); settle(1500);
  turnBy(-90);
  Serial.print(F("After right turn: ")); Serial.println(yaw, 1);
  stopMotors(); settle(1500);
}

void straightTest() {
  static bool done = false;
  update();
  if (done) return;
  if (frontBlocked()) { brake(); settle(300); stopMotors(); done = true; return; }
  driveStep(frontMM < FRONT_SLOW_MM ? SLOW_SPEED : BASE_SPEED, true);
}

void setup() {
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  stopMotors();

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  digitalWrite(PIN_LED, HIGH);
  setupTof();
  setupGyro();
  calibrateGyro();
  calibrateFloor();

  unsigned long t0 = millis();
  while (freshMask != 7) {
    update();
    if (millis() - t0 > 2000) fail(5, F("A VL53L0X is not giving readings"));
  }
  if (leftMM <= WALL_MM && rightMM <= WALL_MM) sideTarget = (leftMM + rightMM) / 2.0;

  settle(START_DELAY_MS);
  yaw = 0;
  targetHeading = 0;
  resetConfirmations();
  digitalWrite(PIN_LED, LOW);
  Serial.println(F("Ready"));
}

void loop() {
  if (finished) {
    brake();
    delay(300);
    stopMotors();
    while (true) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(150);
    }
  }
  switch (MODE) {
    case 0: mazeStep();     break;
    case 1: printSensors(); break;
    case 2: turnTest();     break;
    case 3: straightTest(); break;
  }
}
