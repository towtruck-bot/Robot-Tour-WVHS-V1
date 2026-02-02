// ========== CALIBRATION CONSTANTS ==========
const float WHEEL_DIAMETER_MM = 60.325;
const int PULSES_PER_REV = 14 * 50;  // 700 pulses per revolution
const float WHEEL_BASE_MM = 113.5;

// Speed settings
const int MAX_SPEED = 255;
const int CRUISE_SPEED = 125;
const int MIN_SPEED = 70;
const int TURN_SPEED = 140;

// Acceleration/Deceleration
const int ACCEL_TIME_MS = 400;        // Time to reach cruise speed
const float DECEL_START_MM = 100.0;    // Start slowing down this far from target
const int TURN_ACCEL_MS = 300;
const float TURN_DECEL_DEGREES = 15.0; // Start slowing this many degrees before target

// Motor correction
const float RIGHT_MOTOR_BIAS = 0.98;  // Your existing bias

// PID for straight driving
const float KP_STRAIGHT = 16.0;        // Much more aggressive correction
const float KI_STRAIGHT = 2.5;        // Strong integral to fight motor differences
const float KD_STRAIGHT = 2.0;        // Damping

// PID for turning (sync both wheels)
const float KP_TURN = 3.0;
const float KI_TURN = 0.8;
const float KD_TURN = 1.0;

// Turn calibration
const float TURN_MULTIPLIER_RIGHT = 0.626;
const float TURN_MULTIPLIER_LEFT = 0.52;

// ========== PINS ==========
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

// ========== STATE ==========
volatile long rCnt = 0, lCnt = 0;
float mm_per_tick;
float error_sum = 0;
float last_error = 0;
unsigned long last_pid_time = 0;

// ========== ENCODER ISR ==========
void cntR() {
  static int lastAR = 0;
  int a = digitalRead(encAR);
  int b = digitalRead(encBR);
  if (a != lastAR) {
    (a == HIGH) ? ((b == LOW) ? rCnt-- : rCnt++) : ((b == HIGH) ? rCnt-- : rCnt++);
  }
  lastAR = a;
}

void cntL() {
  static int lastAL = 0;
  int a = digitalRead(encAL);
  int b = digitalRead(encBL);
  if (a != lastAL) {
    (a == HIGH) ? ((b == LOW) ? lCnt++ : lCnt--) : ((b == HIGH) ? lCnt++ : lCnt--);
  }
  lastAL = a;
}

// ========== MOTOR CONTROL ==========
void setMotors(int l, int r) {
  l = constrain(l, -MAX_SPEED, MAX_SPEED);
  r = constrain(r, -MAX_SPEED, MAX_SPEED);
  
  // Left motor (ENB, IN3, IN4)
  if (l >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, l);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -l);
  }
  
  // Right motor (ENA, IN1, IN2)
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
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Reset PID state
  error_sum = 0;
  last_error = 0;
  last_pid_time = 0;
}

// ========== SPEED RAMPING ==========
int getRampedSpeed(unsigned long elapsed_ms, float remaining_mm, int cruise_speed, int min_speed) {
  // Acceleration phase
  int accel_speed = map(elapsed_ms, 0, ACCEL_TIME_MS, min_speed, cruise_speed);
  accel_speed = constrain(accel_speed, min_speed, cruise_speed);
  
  // Deceleration phase
  int decel_speed = cruise_speed;
  if (remaining_mm < DECEL_START_MM) {
    decel_speed = map((int)remaining_mm, 0, (int)DECEL_START_MM, min_speed, cruise_speed);
    decel_speed = constrain(decel_speed, min_speed, cruise_speed);
  }
  
  // Return whichever is slower (smoother)
  return min(accel_speed, decel_speed);
}

int getTurnRampedSpeed(unsigned long elapsed_ms, float remaining_degrees, int cruise_speed, int min_speed, int accel_ms, float decel_degrees) {
  // Acceleration phase
  int accel_speed = map(elapsed_ms, 0, accel_ms, min_speed, cruise_speed);
  accel_speed = constrain(accel_speed, min_speed, cruise_speed);
  
  // Deceleration phase
  int decel_speed = cruise_speed;
  if (remaining_degrees < decel_degrees) {
    decel_speed = map((int)remaining_degrees, 0, (int)decel_degrees, min_speed, cruise_speed);
    decel_speed = constrain(decel_speed, min_speed, cruise_speed);
  }
  
  return min(accel_speed, decel_speed);
}

// ========== GO STRAIGHT ==========
void goStraight(float distance_mm) {
  long target_ticks = (long)(distance_mm / mm_per_tick);
  
  Serial.print("GO ");
  Serial.print(distance_mm);
  Serial.print("mm (");
  Serial.print(target_ticks);
  Serial.println(" ticks)");
  
  rCnt = 0;
  lCnt = 0;
  error_sum = 0;
  last_error = 0;
  last_pid_time = millis();
  
  unsigned long start_time = millis();
  
  while (abs(rCnt) < target_ticks && abs(lCnt) < target_ticks) {
    unsigned long now = millis();
    unsigned long elapsed = now - start_time;
    float dt = (now - last_pid_time) / 1000.0;
    last_pid_time = now;
    
    // Calculate remaining distance
    long avg_ticks = (abs(rCnt) + abs(lCnt)) / 2;
    long remaining_ticks = target_ticks - avg_ticks;
    float remaining_mm = remaining_ticks * mm_per_tick;
    
    // Get ramped speed
    int base_speed = getRampedSpeed(elapsed, remaining_mm, CRUISE_SPEED, MIN_SPEED);
    
    // PID to keep motors synchronized
    long position_error = lCnt - rCnt;  // Positive = left is ahead
    
    if (dt > 0 && dt < 0.1) {  // Sanity check on dt
      error_sum += position_error * dt;
      error_sum = constrain(error_sum, -300, 300);  // Even larger integral limit
      
      float derivative = (position_error - last_error) / dt;
      last_error = position_error;
      
      float correction = KP_STRAIGHT * position_error + 
                        KI_STRAIGHT * error_sum + 
                        KD_STRAIGHT * derivative;
      
      // Important: Apply correction SYMMETRICALLY to both motors
      // Don't apply bias inside the PID loop
      int left_speed = base_speed - (correction / 2);
      int right_speed = base_speed + (correction / 2);
      
      // Apply bias AFTER PID correction
      right_speed = right_speed * RIGHT_MOTOR_BIAS;
      
      setMotors(left_speed, right_speed);
    }
    
    delay(5);  // Fast 5ms loop
  }
  
  stopMotors();
  
  Serial.print("Done. L:");
  Serial.print(lCnt);
  Serial.print(" R:");
  Serial.print(rCnt);
  Serial.print(" Diff:");
  Serial.println(lCnt - rCnt);
  
  delay(500);
}

// ========== TURN RIGHT ==========
void turnRight(float degrees) {
  float turn_circumference = PI * WHEEL_BASE_MM;
  float arc_distance = (degrees / 360.0) * turn_circumference * TURN_MULTIPLIER_RIGHT;
  long target_ticks = (long)(arc_distance / mm_per_tick);
  
  Serial.print("TURN RIGHT ");
  Serial.print(degrees);
  Serial.print("° (");
  Serial.print(target_ticks);
  Serial.println(" ticks)");
  
  rCnt = 0;
  lCnt = 0;
  error_sum = 0;
  last_error = 0;
  last_pid_time = millis();
  
  unsigned long start_time = millis();
  
  // Right turn: Left forward, Right backward
  while (abs(lCnt) < target_ticks || abs(rCnt) < target_ticks) {
    unsigned long now = millis();
    unsigned long elapsed = now - start_time;
    float dt = (now - last_pid_time) / 1000.0;
    last_pid_time = now;
    
    // Calculate remaining turn
    long avg_ticks = (abs(lCnt) + abs(rCnt)) / 2;
    long remaining_ticks = target_ticks - avg_ticks;
    float remaining_degrees = (remaining_ticks * mm_per_tick / turn_circumference) * 360.0 / TURN_MULTIPLIER_RIGHT;
    
    // Get ramped speed
    int turn_speed = getTurnRampedSpeed(elapsed, remaining_degrees, TURN_SPEED, MIN_SPEED, TURN_ACCEL_MS, TURN_DECEL_DEGREES);
    
    // PID to sync both wheels
    long position_error = abs(lCnt) - abs(rCnt);  // Both should travel same distance
    
    if (dt > 0 && dt < 0.1) {
      error_sum += position_error * dt;
      error_sum = constrain(error_sum, -100, 100);
      
      float derivative = (position_error - last_error) / dt;
      last_error = position_error;
      
      float correction = KP_TURN * position_error + 
                        KI_TURN * error_sum + 
                        KD_TURN * derivative;
      
      // Right turn: left forward, right backward
      int left_speed = turn_speed + correction;
      int right_speed = -(turn_speed - correction) * RIGHT_MOTOR_BIAS;
      
      setMotors(left_speed, right_speed);
    }
    
    //delay(5);
  }
  
  stopMotors();
  
  Serial.print("Done. L:");
  Serial.print(lCnt);
  Serial.print(" R:");
  Serial.print(rCnt);
  Serial.print(" Diff:");
  Serial.println(abs(lCnt) + abs(rCnt) - 2*target_ticks);
  
  delay(500);
}

// ========== TURN LEFT ==========
void turnLeft(float degrees) {
  float turn_circumference = PI * WHEEL_BASE_MM;
  float arc_distance = (degrees / 360.0) * turn_circumference * TURN_MULTIPLIER_LEFT;
  long target_ticks = (long)(arc_distance / mm_per_tick);
  
  Serial.print("TURN LEFT ");
  Serial.print(degrees);
  Serial.print("° (");
  Serial.print(target_ticks);
  Serial.println(" ticks)");
  
  rCnt = 0;
  lCnt = 0;
  error_sum = 0;
  last_error = 0;
  last_pid_time = millis();
  
  unsigned long start_time = millis();
  
  // Left turn: Right forward, Left backward
  while (abs(rCnt) < target_ticks || abs(lCnt) < target_ticks) {
    unsigned long now = millis();
    unsigned long elapsed = now - start_time;
    float dt = (now - last_pid_time) / 1000.0;
    last_pid_time = now;
    
    // Calculate remaining turn
    long avg_ticks = (abs(lCnt) + abs(rCnt)) / 2;
    long remaining_ticks = target_ticks - avg_ticks;
    float remaining_degrees = (remaining_ticks * mm_per_tick / turn_circumference) * 360.0 / TURN_MULTIPLIER_LEFT;
    
    // Get ramped speed
    int turn_speed = getTurnRampedSpeed(elapsed, remaining_degrees, TURN_SPEED, MIN_SPEED, TURN_ACCEL_MS, TURN_DECEL_DEGREES);
    
    // PID to sync both wheels
    long position_error = abs(rCnt) - abs(lCnt);  // Both should travel same distance
    
    if (dt > 0 && dt < 0.1) {
      error_sum += position_error * dt;
      error_sum = constrain(error_sum, -100, 100);
      
      float derivative = (position_error - last_error) / dt;
      last_error = position_error;
      
      float correction = KP_TURN * position_error + 
                        KI_TURN * error_sum + 
                        KD_TURN * derivative;
      
      // Left turn: right forward, left backward
      int right_speed = (turn_speed + correction) * RIGHT_MOTOR_BIAS;
      int left_speed = -(turn_speed - correction);
      
      setMotors(left_speed, right_speed);
    }
    
    delay(5);
  }
  
  stopMotors();
  
  Serial.print("Done. L:");
  Serial.print(lCnt);
  Serial.print(" R:");
  Serial.print(rCnt);
  Serial.print(" Diff:");
  Serial.println(abs(lCnt) + abs(rCnt) - 2*target_ticks);
  
  delay(500);
}

// ========== WAIT ==========
void wait(float seconds) {
  delay((int)(seconds * 1000));
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
 
  
  // Configure pins
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
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encAR), cntR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAL), cntL, CHANGE);
  
  stopMotors();
  
  // Calculate conversion factor
  float wheel_circumference = PI * WHEEL_DIAMETER_MM;
  mm_per_tick = wheel_circumference / PULSES_PER_REV;

}

// ========== LOOP ==========
void loop() {
 
  // Test straight driving with detailed feedback
  
  setMotors(-200,212);
  while(rCnt<1000){Serial.print(rCnt);}
  setMotors(0,0);
  delay(1000);
  Serial.println(" ");
  Serial.print(lCnt);
  Serial.print(" - ");
  Serial.println(rCnt);
  rCnt =0;
  lCnt = 0;
  
  delay(1000);
  
}
