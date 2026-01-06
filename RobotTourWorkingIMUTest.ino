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
float Kp_spd = 3.0, Ki_spd = 1.0, Kd_spd = 0.4;
float Kp_hdg = 25.0, Ki_hdg = 5.0, Kd_hdg = 3.0;  // Much stronger heading correction
float spdErr = 0, spdErrLast = 0, spdErrSum = 0;
float hdgErr = 0, hdgErrLast = 0, hdgErrSum = 0;

// --- Control ---
int baseSpd = 180;   // Reduced from 255
int maxSpd = 255;
int startBoost = 200;  // Reduced from 255
int startDur = 200;    // Reduced from 500ms
int rightBias = -18;
// --- Heading ---
float heading = 0.0;
float targetHeading = 0.0;  // Target heading to maintain
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
  
  if (lastGyroT > 0 && dt < 1.0) {  // Increased from 0.1 to 1.0 second max
    heading += g.gyro.z * 57.2958 * dt;
    
    // Normalize heading to [-180, 180]
    while (heading > 180) heading -= 360;
    while (heading < -180) heading += 360;
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
  
  // Heading PID - track error from target heading
  hdgErr = targetHeading - heading;
  // Wrap error to [-180, 180]
  while (hdgErr > 180) hdgErr -= 360;
  while (hdgErr < -180) hdgErr += 360;
  
  hdgErrSum += hdgErr * dt;
  hdgErrSum = constrain(hdgErrSum, -100, 100);  // Increased integral limit
  float hdgD = (hdgErr - hdgErrLast) / dt;
  hdgErrLast = hdgErr;
  float hdgCorr = Kp_hdg * hdgErr + Ki_hdg * hdgErrSum + Kd_hdg * hdgD;
  
  int l = baseSpd + spdCorr/2 + hdgCorr;
  int r = baseSpd + rightBias - spdCorr/2 - hdgCorr;  // Add bias to right motor
  
  setMotors(l, r);
}

// ========== Turn ==========
void turnLeft(float degrees, int spd = 120) {
  baseSpd = spd;
  rCnt = lCnt = 0;
  lastRC = lastLC = 0;
  float startHeading = heading;
  lastGyroT = millis();
  lastPIDT = millis();
  
  spdErrSum = hdgErrSum = 0;
  spdErrLast = hdgErrLast = 0;
  
  Serial.print("TURN LEFT ");
  Serial.print(degrees);
  Serial.print(" from ");
  Serial.println(startHeading);
  
  // Track total rotation
  float rotated = 0;
  float lastHeading = heading;
  
  // Turn in place with proportional speed control
  while (rotated < degrees - 1.0) {
    updHdg();
    
    // Calculate change in heading, handling wrap-around
    float delta = heading - lastHeading;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    rotated += delta;
    lastHeading = heading;
    
    float remaining = degrees - rotated;
    int turnSpd = spd;
    
    // Slow down in last 15 degrees
    if (remaining < 15) {
      turnSpd = max(60, (int)(spd * remaining / 15.0));
    }
    
    setMotors(turnSpd+10, -turnSpd - rightBias/2-10);
    delay(1);
  }
  
  stopMotors();
  delay(50);  // Let it settle
  
  // Normalize and update target heading
  while (heading > 180) heading -= 360;
  while (heading < -180) heading += 360;
  targetHeading = heading;
  
  Serial.print("Done Hdg:");
  Serial.print(heading);
  Serial.print(" Target:");
  Serial.println(targetHeading);
}

void turnRight(float degrees, int spd = 120) {
  baseSpd = spd;
  rCnt = lCnt = 0;
  lastRC = lastLC = 0;
  float startHeading = heading;
  lastGyroT = millis();
  lastPIDT = millis();
  
  spdErrSum = hdgErrSum = 0;
  spdErrLast = hdgErrLast = 0;
  
  Serial.print("TURN RIGHT ");
  Serial.print(degrees);
  Serial.print(" from ");
  Serial.println(startHeading);
  
  // Track total rotation
  float rotated = 0;
  float lastHeading = heading;
  
  // Turn in place with proportional speed control
  while (rotated < degrees - 1.0) {
    updHdg();
    
    // Calculate change in heading, handling wrap-around
    float delta = lastHeading - heading;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    rotated += delta;
    lastHeading = heading;
    
    float remaining = degrees - rotated;
    int turnSpd = spd;
    
    // Slow down in last 15 degrees
    if (remaining < 15) {
      turnSpd = max(60, (int)(spd * remaining / 15.0));
    }
    
    setMotors(-turnSpd-10,(turnSpd + rightBias/2)+10);
    delay(1);
  }
  
  stopMotors();
  delay(50);  // Let it settle
  
  // Normalize and update target heading
  while (heading > 180) heading -= 360;
  while (heading < -180) heading += 360;
  targetHeading = heading;
  
  Serial.print("Done Hdg:");
  Serial.print(heading);
  Serial.print(" Target:");
  Serial.println(targetHeading);
}

void stop() {
  stopMotors();
  Serial.println("STOPPED");
}

// ========== Drive ==========
void go(long ticks, int spd = 160) {
  baseSpd = spd;
  rCnt = lCnt = 0;
  lastRC = lastLC = 0;
  // Don't reset heading - maintain continuous tracking
  lastGyroT = millis();
  lastPIDT = millis();
  unsigned long start = millis();
  
  spdErrSum = hdgErrSum = 0;
  spdErrLast = hdgErrLast = 0;
  
  Serial.print("GO - Target Hdg: ");
  Serial.println(targetHeading);
  
  while (abs(rCnt) < ticks && abs(lCnt) < ticks) {
    if (millis() - start < startDur) {
      setMotors(startBoost, startBoost + rightBias);
      updHdg();  // Still track heading during boost!
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
  Serial.print(heading);
  Serial.print(" Target:");
  Serial.println(targetHeading);
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
  
  // IMU init
  if (imu.begin_I2C(0x6A)) {
    Serial.println("IMU OK at 0x6A");
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    imuOK = true;
    
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
  // Encoder test mode - uncomment to debug
  /*
  Serial.print("L A:");
  Serial.print(digitalRead(encAL));
  Serial.print(" B:");
  Serial.print(digitalRead(encBL));
  Serial.print(" | R A:");
  Serial.print(digitalRead(encAR));
  Serial.print(" B:");
  Serial.print(digitalRead(encBR));
  Serial.print(" | Counts L:");
  Serial.print(lCnt);
  Serial.print(" R:");
  Serial.println(rCnt);
  delay(100);
  return;
  */
  
  // Example sequence - customize as needed
  for(int i = 0; i<4; i++){
  go(3000);
  delay(1000);
  
  turnRight(84.5);
  delay(1000);
  }
 
  
  stop();
  delay(1000);  // Wait 5 seconds before repeating
}
