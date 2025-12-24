#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

// --- Pins ---
#define encAL 2
#define encBL 4
#define encAR 3
#define encBR 12
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
#define ENA 6
#define ENB 5

// --- Encoder State ---
volatile long rCnt = 0, lCnt = 0;
volatile int lastAR = LOW, lastAL = LOW;

// --- IMU ---
Adafruit_LSM6DS3TRC imu;
bool imuOK = false;

// --- PID ---
float Kp_spd = 3.0, Ki_spd = 1.0, Kd_spd = 0.4;  // Slightly increased for faster correction
float Kp_hdg = 8.0, Ki_hdg = 1.5, Kd_hdg = 1.0;  // Increased heading gains for stronger correction
float spdErr = 0, spdErrLast = 0, spdErrSum = 0;
float hdgErr = 0, hdgErrLast = 0, hdgErrSum = 0;

// --- Control ---
int baseSpd = 255;  // Full speed
int maxSpd = 255;   // Full power available
int startBoost = 255;  // Max power to overcome static friction
int startDur = 500;  // Longer boost duration

// --- Heading ---
float heading = 0.0;
unsigned long lastGyroT = 0, lastPIDT = 0;
const int PID_INT = 20;

long lastRC = 0, lastLC = 0;

// ========== ISR ==========
void cntR() {
  int a = digitalRead(encAR);
  int b = digitalRead(encBR);
  if (a != lastAR) {
    (a == HIGH) ? ((b == LOW) ? rCnt-- : rCnt++) : ((b == HIGH) ? rCnt-- : rCnt++);
  }
  lastAR = a;
}

void cntL() {
  int a = digitalRead(encAL);
  int b = digitalRead(encBL);
  if (a != lastAL) {
    (a == HIGH) ? ((b == LOW) ? lCnt++ : lCnt--) : ((b == HIGH) ? lCnt++ : lCnt--);
  }
  lastAL = a;
}

// ========== Motor ==========
void setMotors(int l, int r) {
  l = constrain(l, -maxSpd, maxSpd);
  r = constrain(r, -maxSpd, maxSpd);
  
  // Left
  if (l >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, l);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -l);
  }
  
  // Right (reversed)
  if (r >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, r);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -r);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  spdErrSum = 0;
  hdgErrSum = 0;
  spdErrLast = 0;
  hdgErrLast = 0;
}

// ========== Gyro ==========
void updHdg() {
  if (!imuOK) return;
  
  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);
  
  unsigned long now = millis();
  float dt = (now - lastGyroT) / 1000.0;
  
  if (lastGyroT > 0 && dt < 0.1) {
    heading += g.gyro.z * 57.2958 * dt;
    if (heading > 180) heading -= 360;
    if (heading < -180) heading += 360;
  }
  lastGyroT = now;
}

// ========== PID ==========
void runPID() {
  unsigned long now = millis();
  if (now - lastPIDT < PID_INT) return;
  
  float dt = (now - lastPIDT) / 1000.0;
  lastPIDT = now;
  
  updHdg();
  
  long rDelta = rCnt - lastRC;
  long lDelta = lCnt - lastLC;
  lastRC = rCnt;
  lastLC = lCnt;
  
  // Speed PID
  spdErr = rDelta - lDelta;
  spdErrSum += spdErr * dt;
  spdErrSum = constrain(spdErrSum, -100, 100);
  float spdD = (spdErr - spdErrLast) / dt;
  spdErrLast = spdErr;
  float spdCorr = Kp_spd * spdErr + Ki_spd * spdErrSum + Kd_spd * spdD;
  
  // Heading PID
  hdgErr = -heading;
  hdgErrSum += hdgErr * dt;
  hdgErrSum = constrain(hdgErrSum, -50, 50);
  float hdgD = (hdgErr - hdgErrLast) / dt;
  hdgErrLast = hdgErr;
  float hdgCorr = Kp_hdg * hdgErr + Ki_hdg * hdgErrSum + Kd_hdg * hdgD;
  
  int l = baseSpd + spdCorr/2 + hdgCorr;
  int r = baseSpd - spdCorr/2 - hdgCorr;
  
  setMotors(l, r);
}

// ========== Drive ==========
void go(long ticks, int spd = 160) {
  baseSpd = spd;
  rCnt = lCnt = 0;
  lastRC = lastLC = 0;
  heading = 0;
  lastGyroT = millis();
  lastPIDT = millis();
  unsigned long start = millis();
  
  spdErrSum = hdgErrSum = 0;
  spdErrLast = hdgErrLast = 0;
  
  Serial.println("GO");
  
  while (abs(rCnt) < ticks && abs(lCnt) < ticks) {
    if (millis() - start < startDur) {
      setMotors(startBoost, startBoost);
      delay(1);
      continue;
    }
    runPID();
    delay(1);
  }
  
  stopMotors();
  
  Serial.print("Done L:");
  Serial.print(lCnt);
  Serial.print(" R:");
  Serial.print(rCnt);
  Serial.print(" Hdg:");
  Serial.println(heading);
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("PID Control");
  
  // I2C scan
  Wire.begin();
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C: 0x");
      Serial.println(i, HEX);
    }
  }
  
  // IMU init - Re-enabled with proper configuration
  if (imu.begin_I2C(0x6A)) {
    Serial.println("IMU OK at 0x6A");
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    imuOK = true;
    
    // Calibrate gyro - let it settle for 1 second
    Serial.println("Calibrating gyro - keep robot still...");
    delay(1000);
    Serial.println("Calibration done!");
  } else if (imu.begin_I2C()) {
    Serial.println("IMU OK at default");
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    imuOK = true;
    
    Serial.println("Calibrating gyro - keep robot still...");
    delay(1000);
    Serial.println("Calibration done!");
  } else {
    Serial.println("IMU FAIL - encoder only");
    imuOK = false;
  }
  
  // Pins
  pinMode(encAL, INPUT_PULLUP);
  pinMode(encBL, INPUT_PULLUP);
  pinMode(encAR, INPUT_PULLUP);
  pinMode(encBR, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encAR), cntR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAL), cntL, CHANGE);
  
  Serial.println("Ready");
}

// ========== Loop ==========
void loop() {
  // Test: just run immediately
  static bool ran = false;
  if (!ran) {
    Serial.println("Testing LEFT motor only...");
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
    delay(2000);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
    
    Serial.println("Testing RIGHT motor only...");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 255);
    delay(2000);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    
    Serial.println("Both motors together at FULL speed...");
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 255);
    delay(2000);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
    
    Serial.println("Starting in 3 sec...");
    delay(3000);
    Serial.println("GO!");
    go(4800, 255);  // Full speed
    delay(2000);
    Serial.println("Test complete. Reboot to run again.");
    ran = true;
  }
}