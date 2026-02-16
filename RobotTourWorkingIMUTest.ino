// Gyro calibration
const float GYRO_SCALE_FACTOR = 40.96;  // Adjust this if turns are wrong
                                         // Larger = slower turn reading
                                         // If 90° reads as X°, multiply by X/90// ========== Arduino Nano Robot with BMI160 IMU ==========
// Using DFRobot BMI160 library

#include <Wire.h>
#include <DFRobot_BMI160.h>

// ========== PIN DEFINITIONS ==========
#define ENC1_A 2
#define ENC2_A 3
#define ENC1_B 4
#define ENC2_B 5
#define STBY 6
#define AIN1 7
#define AIN2 8
#define PWMA 9
#define PWMB 10
#define BIN1 12
#define BIN2 11

// ========== CALIBRATION CONSTANTS ==========
const float WHEEL_DIAMETER_MM = 60.325;
const int PULSES_PER_REV = 7 * 50;
const float WHEEL_BASE_MM = 113.5;

const int MAX_SPEED = 150;
const int CRUISE_SPEED = 120;
const int MIN_SPEED = 55;
const int TURN_SPEED = 80;

const int ACCEL_TIME_MS = 2000;
const float DECEL_START_MM = 250.0;
const int TURN_ACCEL_MS = 1000;
const float TURN_DECEL_DEGREES = 50.0;

const float MOTOR_A_BIAS = 1.3;
const float MOTOR_B_BIAS = 1.0;

const float KP_STRAIGHT = 1.5;
const float KI_STRAIGHT = 0.0;
const float KD_STRAIGHT = 0.25;

const float KP_TURN = 1.0;
const float KI_TURN = 0.0;
const float KD_TURN = 0.0;

const float KP_HEADING = 0.8;
const float KI_HEADING = 0.0;
const float KD_HEADING = 0.0;

const float TURN_MULTIPLIER_RIGHT = 0.53;
const float TURN_MULTIPLIER_LEFT = 0.55;

// ========== GLOBAL STATE ==========
volatile long enc1_cnt = 0, enc2_cnt = 0;
float mm_per_tick;

float error_sum = 0;
float last_error = 0;
unsigned long last_pid_time = 0;

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x68;
bool imu_available = false;
float current_heading = 0.0;
float target_heading = 0.0;
unsigned long last_imu_time = 0;
float gyro_z_offset = 0.0;

// Low-pass filter for gyro
float filtered_gyro_z = 0.0;
const float GYRO_FILTER_ALPHA = 0.3;  // 0.3 = heavy filtering, 1.0 = no filtering

// Deadband to ignore tiny gyro drift
const float GYRO_DEADBAND = 0.5;  // Ignore readings below ±0.5 deg/s

float heading_error_sum = 0;
float last_heading_error = 0;

// ========== ENCODER ISR ==========
void enc1_isr() {
  static int lastA = 0;
  int a = digitalRead(ENC1_A);
  int b = digitalRead(ENC1_B);
  if (a != lastA) {
    (a == HIGH) ? ((b == LOW) ? enc1_cnt++ : enc1_cnt--) : ((b == HIGH) ? enc1_cnt++ : enc1_cnt--);
  }
  lastA = a;
}

void enc2_isr() {
  static int lastA = 0;
  int a = digitalRead(ENC2_A);
  int b = digitalRead(ENC2_B);
  if (a != lastA) {
    (a == HIGH) ? ((b == LOW) ? enc2_cnt++ : enc2_cnt--) : ((b == HIGH) ? enc2_cnt++ : enc2_cnt--);
  }
  lastA = a;
}

// ========== IMU FUNCTIONS ==========
void calibrateGyro() {
  Serial.println("Calibrating gyro (10 samples, keep still)...");
  
  float sum = 0;
  int samples = 0;
  
  for (int i = 0; i < 10; i++) {
    int16_t accelGyro[6] = {0};
    
    if (bmi160.getAccelGyroData(accelGyro) == BMI160_OK) {
      // accelGyro[0-2] = gyro XYZ, [3-5] = accel XYZ
      int16_t gyro_z_raw = accelGyro[2];
      
      Serial.print("  Sample ");
      Serial.print(i + 1);
      Serial.print(": Z=");
      Serial.print(gyro_z_raw);
      Serial.print(" (");
      Serial.print(gyro_z_raw * 3.14 / 180.0, 3);
      Serial.println(" rad/s)");
      
      sum += gyro_z_raw;
      samples++;
    }
    delay(50);
  }
  
  if (samples > 0) {
    gyro_z_offset = sum / samples;
    Serial.print("Gyro Z offset: ");
    Serial.print(gyro_z_offset, 2);
    Serial.println(" (raw)");
  }
}

void updateHeading() {
  if (!imu_available) return;
  
  int16_t accelGyro[6] = {0};
  
  if (bmi160.getAccelGyroData(accelGyro) == BMI160_OK) {
    unsigned long now = millis();
    float dt = (now - last_imu_time) / 1000.0;
    
    if (last_imu_time > 0 && dt < 1.0 && dt > 0) {
      int16_t gyro_z_raw = accelGyro[2] - gyro_z_offset;
      float gyro_z_deg = gyro_z_raw / GYRO_SCALE_FACTOR;
      
      // Apply deadband to ignore tiny drift
      if (abs(gyro_z_deg) < GYRO_DEADBAND) {
        gyro_z_deg = 0.0;
      }
      
      // Apply low-pass filter to reduce noise
      filtered_gyro_z = (GYRO_FILTER_ALPHA * gyro_z_deg) + ((1.0 - GYRO_FILTER_ALPHA) * filtered_gyro_z);
      filtered_gyro_z *= 1.5;
      current_heading += filtered_gyro_z * dt;
      
      while (current_heading > 180) current_heading -= 360;
      while (current_heading < -180) current_heading += 360;
    }
    
    last_imu_time = now;
  }
}

void resetHeading() {
  current_heading = 0.0;
  target_heading = 0.0;
  last_imu_time = millis();
  heading_error_sum = 0;
  last_heading_error = 0;
  filtered_gyro_z = 0.0;
}

// ========== MOTOR CONTROL ==========
void setMotors(int motor_a, int motor_b) {
  digitalWrite(STBY, HIGH);
  
  motor_a = motor_a;
  motor_b = motor_b * MOTOR_B_BIAS;
  
  motor_a = constrain(motor_a, -MAX_SPEED, MAX_SPEED);
  motor_b = constrain(motor_b, -MAX_SPEED, MAX_SPEED);
  
  if (motor_a <= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, -motor_a);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, motor_a);
  }
  
  if (motor_b <= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, abs(motor_b));
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, motor_b);
  }
}

void stopMotors() {
  digitalWrite(STBY, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  
  error_sum = 0;
  last_error = 0;
  last_pid_time = 0;
  heading_error_sum = 0;
  last_heading_error = 0;
}

// ========== SPEED RAMPING ==========
int getRampedSpeed(unsigned long elapsed_ms, float remaining_mm, int cruise_speed, int min_speed) {
  int accel_speed = map(elapsed_ms, 0, ACCEL_TIME_MS, min_speed, cruise_speed);
  accel_speed = constrain(accel_speed, min_speed, cruise_speed);
  
  int decel_speed = cruise_speed;
  if (remaining_mm < DECEL_START_MM) {
    decel_speed = map((int)remaining_mm, 0, (int)DECEL_START_MM, min_speed, cruise_speed);
    decel_speed = constrain(decel_speed, min_speed, cruise_speed);
  }
  
  return min(accel_speed, decel_speed);
}

int getTurnRampedSpeed(unsigned long elapsed_ms, float remaining_degrees, int cruise_speed, int min_speed) {
  int accel_speed = map(elapsed_ms, 0, TURN_ACCEL_MS, min_speed, cruise_speed);
  accel_speed = constrain(accel_speed, min_speed, cruise_speed);
  
  int decel_speed = cruise_speed;
  if (remaining_degrees < TURN_DECEL_DEGREES) {
    decel_speed = map((int)remaining_degrees, 0, (int)TURN_DECEL_DEGREES, min_speed, cruise_speed);
    decel_speed = constrain(decel_speed, min_speed, cruise_speed);
  }
  
  return min(accel_speed, decel_speed);
}

// ========== GO STRAIGHT ==========
void goStraight(float distance_mm, bool use_imu = true) {
  long target_ticks = (long)abs(distance_mm / mm_per_tick);
  target_ticks *= (50/44.7)*1.02;
  Serial.print("GO ");
  Serial.print(distance_mm);
  Serial.print("mm ");
  Serial.println(use_imu && imu_available ? "[IMU]" : "[ENC]");
  
  enc1_cnt = 0;
  enc2_cnt = 0;
  error_sum = 0;
  last_error = 0;
  last_pid_time = millis();
  
  if (use_imu && imu_available) {
    target_heading = current_heading;
    heading_error_sum = 0;
    last_heading_error = 0;
  }
  
  unsigned long start_time = millis();
  
  while (abs(enc1_cnt) < target_ticks && abs(enc2_cnt) < target_ticks) {
    unsigned long now = millis();
    unsigned long elapsed = now - start_time;
    float dt = (now - last_pid_time) / 1000.0;
    last_pid_time = now;
    
    if (use_imu && imu_available) {
      updateHeading();
    }
    
    long avg_ticks = (abs(enc1_cnt) + abs(enc2_cnt)) / 2;
    long remaining_ticks = target_ticks - avg_ticks;
    float remaining_mm = remaining_ticks * mm_per_tick;
    
    int base_speed = getRampedSpeed(elapsed, remaining_mm, CRUISE_SPEED, MIN_SPEED);
    
    float correction = 0;
    
    if (use_imu && imu_available) {
      float heading_error = target_heading - current_heading;
      while (heading_error > 180) heading_error -= 360;
      while (heading_error < -180) heading_error += 360;
      
      if (dt > 0 && dt < 0.1) {
        heading_error_sum += heading_error * dt;
        heading_error_sum = constrain(heading_error_sum, -100, 100);
        
        float heading_derivative = (heading_error - last_heading_error) / dt;
        last_heading_error = heading_error;
        
        correction = KP_HEADING * heading_error + 
                    KI_HEADING * heading_error_sum + 
                    KD_HEADING * heading_derivative;
                    correction *=1;
                    correction = constrain(correction, -15,15);
      }
    } else {
      long position_error = 0;
      if (enc1_cnt > 0) {
        position_error = enc1_cnt + enc2_cnt;
      } else {
        position_error = -enc1_cnt - enc2_cnt;
      }
      
      if (dt > 0 && dt < 0.1) {
        error_sum += position_error * dt;
        error_sum = constrain(error_sum, -300, 300);
        
        float derivative = (position_error - last_error) / dt;
        last_error = position_error;
        
        correction = KP_STRAIGHT * position_error + 
                    KI_STRAIGHT * error_sum + 
                    KD_STRAIGHT * derivative;
      }
    }
     int motor_a_speed = 0;
      int motor_b_speed = 0;
    if(avg_ticks>5){
     motor_a_speed = base_speed - (correction / 2);
     motor_b_speed = base_speed + (correction / 2);
    } else{
       motor_a_speed = base_speed;// - (correction / 2);
     motor_b_speed = base_speed;// + (correction / 2);
    }
    motor_a_speed*=MOTOR_A_BIAS;
    if (distance_mm > 0) {
      
      setMotors(motor_a_speed, motor_b_speed);
    } else {
      setMotors(-motor_a_speed, -motor_b_speed);
    }
    delay(5);
  }
  
  stopMotors();
  
  Serial.print("Initial stop. Enc1:");
  Serial.print(enc1_cnt);
  Serial.print(" Enc2:");
  Serial.print(enc2_cnt);
  
  // ========== SETTLING PERIOD ==========
  // 1 second PID control to settle at target position
  const long SETTLE_DEADZONE = 5;  // ticks - ignore errors smaller than this
  const int SETTLE_MAX_SPEED = 40;  // low speed for fine corrections
  const float SETTLE_KP = 2.0;
  
  unsigned long settle_start = millis();
  error_sum = 0;
  last_error = 0;
  last_pid_time = millis();
  
  while (millis() - settle_start < 1000) {  // 1 second settling period
    long avg_ticks = (abs(enc1_cnt) + abs(enc2_cnt)) / 2;
    long position_error = target_ticks - avg_ticks;
    
    // Exit early if we're within deadzone
    if (abs(position_error) <= SETTLE_DEADZONE) {
      delay(10);
      continue;
    }
    
    unsigned long now = millis();
    float dt = (now - last_pid_time) / 1000.0;
    last_pid_time = now;
    
    if (dt > 0 && dt < 0.1) {
      // Simple P control for settling
      float correction = SETTLE_KP * position_error;
      int settle_speed = constrain((int)correction, -SETTLE_MAX_SPEED, SETTLE_MAX_SPEED);
      
      if (distance_mm > 0) {
        setMotors(settle_speed * MOTOR_A_BIAS, settle_speed);
      } else {
        setMotors(-settle_speed * MOTOR_A_BIAS, -settle_speed);
      }
    }
    
    delay(10);
  }
  
  stopMotors();
  
  Serial.print("After settling. Enc1:");
  Serial.print(enc1_cnt);
  Serial.print(" Enc2:");
  Serial.print(enc2_cnt);
  Serial.print(" (target: ");
  Serial.print(target_ticks);
  Serial.print(")");
  if (imu_available) {
    Serial.print(" Heading:");
    Serial.print(current_heading, 1);
  }
  Serial.println();
  
  delay(500);
}

// ========== TURN LEFT ==========
void turnLeft(float degrees, bool use_imu = true) {
  //degrees*=1.17; imu
  degrees *= 1.82;
  Serial.print("TURN LEFT ");
  Serial.print(degrees);
  Serial.print("° ");
  Serial.println(use_imu && imu_available ? "[IMU]" : "[ENC]");
  
  if (use_imu && imu_available) {
    target_heading = current_heading + degrees;
    while (target_heading < -180) target_heading += 360;
    while (target_heading > 180) target_heading -= 360;
    
    unsigned long start_time = millis();
    float start_heading = current_heading;
    
    while (true) {
      updateHeading();
      
      float rotated = current_heading - start_heading;
      while (rotated < -180) rotated += 360;
      while (rotated > 180) rotated -= 360;
      
      if (rotated >= degrees - 1.0) break;
      
      unsigned long elapsed = millis() - start_time;
      float remaining = degrees - rotated;
      
      int turn_speed = getTurnRampedSpeed(elapsed, remaining, TURN_SPEED, MIN_SPEED);
      
      setMotors(-turn_speed, turn_speed);
      delay(5);
    }
  } else {
    float turn_circumference = PI * WHEEL_BASE_MM;
    float arc_distance = (degrees / 360.0) * turn_circumference * TURN_MULTIPLIER_LEFT;
    long target_ticks = (long)(arc_distance / mm_per_tick);
    
    enc1_cnt = 0;
    enc2_cnt = 0;
    error_sum = 0;
    last_error = 0;
    last_pid_time = millis();
    unsigned long start_time = millis();
    
    while (abs(enc2_cnt) < target_ticks || abs(enc1_cnt) < target_ticks) {
      unsigned long now = millis();
      unsigned long elapsed = now - start_time;
      float dt = (now - last_pid_time) / 1000.0;
      last_pid_time = now;
      
      long avg_ticks = (abs(enc1_cnt) + abs(enc2_cnt)) / 2;
      long remaining_ticks = target_ticks - avg_ticks;
      float remaining_degrees = (remaining_ticks * mm_per_tick / turn_circumference) * 360.0 / TURN_MULTIPLIER_LEFT;
      
      int turn_speed = getTurnRampedSpeed(elapsed, remaining_degrees, TURN_SPEED, MIN_SPEED);
      
      long position_error = abs(enc2_cnt) - abs(enc1_cnt);
      
      if (dt > 0 && dt < 0.1) {
        error_sum += position_error * dt;
        error_sum = constrain(error_sum, -100, 100);
        
        float derivative = (position_error - last_error) / dt;
        last_error = position_error;
        
        float correction = KP_TURN * position_error + 
                          KI_TURN * error_sum + 
                          KD_TURN * derivative;
        
        int motor_a_speed = -(turn_speed + correction);
        int motor_b_speed = turn_speed - correction;
        
        setMotors(motor_a_speed, motor_b_speed);
      }
      
      delay(5);
    }
    
    stopMotors();
    
    Serial.print("Initial stop. Enc1:");
    Serial.print(enc1_cnt);
    Serial.print(" Enc2:");
    Serial.print(enc2_cnt);
    
    // ========== SETTLING PERIOD FOR TURNS ==========
    const long TURN_SETTLE_DEADZONE = 3;  // ticks
    const int TURN_SETTLE_MAX_SPEED = 35;
    const float TURN_SETTLE_KP = 2.5;
    
    unsigned long settle_start = millis();
    error_sum = 0;
    last_error = 0;
    last_pid_time = millis();
    
    while (millis() - settle_start < 1000) {  // 1 second settling
      long avg_ticks = (abs(enc1_cnt) + abs(enc2_cnt)) / 2;
      long position_error = target_ticks - avg_ticks;
      
      if (abs(position_error) <= TURN_SETTLE_DEADZONE) {
        delay(10);
        continue;
      }
      
      unsigned long now = millis();
      float dt = (now - last_pid_time) / 1000.0;
      last_pid_time = now;
      
      if (dt > 0 && dt < 0.1) {
        float correction = TURN_SETTLE_KP * position_error;
        int settle_speed = constrain((int)correction, -TURN_SETTLE_MAX_SPEED, TURN_SETTLE_MAX_SPEED);
        
        // Continue turning in same direction to reach target
        setMotors(-settle_speed, settle_speed);
      }
      
      delay(10);
    }
    
    stopMotors();
    
    Serial.print("After settling. Enc1:");
    Serial.print(enc1_cnt);
    Serial.print(" Enc2:");
    Serial.print(enc2_cnt);
    Serial.print(" (target: ");
    Serial.print(target_ticks);
    Serial.println(")");
  }
  
  if (imu_available) {
    Serial.print("Done. Heading:");
    Serial.print(current_heading, 1);
    Serial.print(" Target:");
    Serial.println(target_heading, 1);
  }
  
  delay(500);
}

// ========== TURN RIGHT ==========
void turnRight(float degrees, bool use_imu = true) {
  Serial.print("TURN RIGHT ");
  Serial.print(degrees);
  Serial.print("° ");
  Serial.println(use_imu && imu_available ? "[IMU]" : "[ENC]");
  
  if (use_imu && imu_available) {
    target_heading = current_heading - degrees;
    while (target_heading < -180) target_heading += 360;
    while (target_heading > 180) target_heading -= 360;
    
    unsigned long start_time = millis();
    float start_heading = current_heading;
    
    while (true) {
      updateHeading();
      
      float rotated = start_heading - current_heading;
      while (rotated < -180) rotated += 360;
      while (rotated > 180) rotated -= 360;
      
      if (rotated >= degrees - 1.0) break;
      
      unsigned long elapsed = millis() - start_time;
      float remaining = degrees - rotated;
      
      int turn_speed = getTurnRampedSpeed(elapsed, remaining, TURN_SPEED, MIN_SPEED);
      
      setMotors(turn_speed, -turn_speed);
      delay(5);
    }
  } else {
    float turn_circumference = PI * WHEEL_BASE_MM;
    float arc_distance = (degrees / 360.0) * turn_circumference * TURN_MULTIPLIER_RIGHT;
    long target_ticks = (long)(arc_distance / mm_per_tick);
    
    enc1_cnt = 0;
    enc2_cnt = 0;
    error_sum = 0;
    last_error = 0;
    last_pid_time = millis();
    unsigned long start_time = millis();
    
    while (abs(enc1_cnt) < target_ticks || abs(enc2_cnt) < target_ticks) {
      unsigned long now = millis();
      unsigned long elapsed = now - start_time;
      float dt = (now - last_pid_time) / 1000.0;
      last_pid_time = now;
      
      long avg_ticks = (abs(enc1_cnt) + abs(enc2_cnt)) / 2;
      long remaining_ticks = target_ticks - avg_ticks;
      float remaining_degrees = (remaining_ticks * mm_per_tick / turn_circumference) * 360.0 / TURN_MULTIPLIER_RIGHT;
      
      int turn_speed = getTurnRampedSpeed(elapsed, remaining_degrees, TURN_SPEED, MIN_SPEED);
      
      long position_error = abs(enc1_cnt) - abs(enc2_cnt);
      
      if (dt > 0 && dt < 0.1) {
        error_sum += position_error * dt;
        error_sum = constrain(error_sum, -100, 100);
        
        float derivative = (position_error - last_error) / dt;
        last_error = position_error;
        
        float correction = KP_TURN * position_error + 
                          KI_TURN * error_sum + 
                          KD_TURN * derivative;
        
        int motor_a_speed = turn_speed + correction;
        int motor_b_speed = -(turn_speed - correction);
        
        setMotors(motor_a_speed, -motor_b_speed);
      }
      
      delay(5);
    }
  }
  
  stopMotors();
  
  if (imu_available) {
    Serial.print("Done. Heading:");
    Serial.print(current_heading, 1);
    Serial.print(" Target:");
    Serial.println(target_heading, 1);
  }
  
  delay(500);
}

void wait(float seconds) {
  delay((int)(seconds * 1000));
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("=== Arduino Nano Robot ===");
  
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  digitalWrite(STBY, LOW);
  
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2_isr, CHANGE);
  
  // Initialize IMU
  Serial.println("Initializing BMI160...");
  
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("BMI160 reset failed!");
    imu_available = false;
  } else if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("BMI160 init failed!");
    imu_available = false;
  } else {
    Serial.println("BMI160 initialized!");
    imu_available = true;
    delay(100);
    calibrateGyro();
    resetHeading();
  }
  
  // Calculate conversion factor
  float wheel_circumference = PI * WHEEL_DIAMETER_MM;
  mm_per_tick = wheel_circumference / PULSES_PER_REV;
  
  Serial.print("MM per tick: ");
  Serial.println(mm_per_tick, 4);
  Serial.println("Ready!");
  delay(2000);
}

// ========== LOOP ==========
void loop() {
  /*
  Serial.println("\n=== Test Sequence ===");
  enc1_cnt = 0;
  enc2_cnt = 0;
  setMotors(-125,-125);
  wait(2);
  Serial.println(enc1_cnt);
  Serial.println(enc2_cnt);
  setMotors(0,0);   
  wait(1.5);
  Serial.println(enc1_cnt);
  Serial.println(enc2_cnt);
  enc1_cnt = 0;
  enc2_cnt = 0;
  setMotors(125,125);
  wait(2);
  Serial.println(enc1_cnt);
  Serial.println(enc2_cnt);
  setMotors(0,0);   
  while(true){
    delay(23);
  }
  */
   for(int i = 0;i<4;i++){
    goStraight(500, false);
  wait(2);
  turnLeft(90, false);
  wait(2);
   }
  // Test individual movements first
  Serial.println("Testing 90 degree turn...");
  
  
  Serial.print("Final heading: ");
  Serial.println(current_heading, 1);
  
  Serial.println("Testing straight 500mm...");
  
  
  Serial.println("=== Complete ===\n");
  
  while (true) {
    delay(1000);
  }
}
