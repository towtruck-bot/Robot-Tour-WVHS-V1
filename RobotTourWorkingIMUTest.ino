// ========== COMBINED ROBOT CONTROL ==========
// Adjusted for 113.5mm wheel-to-wheel diameter

// ========== CALIBRATION ==========
const float WHEEL_DIAMETER_MM = 60.325;
const int PULSES_PER_REV = 435;
const float WHEEL_BASE_MM = 113.5;  // NEW: Updated wheel-to-wheel diameter

// Straight driving - YOUR ORIGINAL SETTINGS with 95 speed
const int BASE_SPEED = 95;
const int MAX_SPEED = 255;

// Straight PID - YOUR ORIGINAL
const float KP = 8.0;
const float KI = 0.5;
const float KD = 2.0;
const int POSITION_TOLERANCE = 2;

// Turn speed - BOOSTED for left turn
const int TURN_SPEED_MAX = 140;
const int TURN_SPEED_MIN = 110;

// Left turn boost - extra power
const int LEFT_TURN_SPEED_MAX = 200;
const int LEFT_TURN_SPEED_MIN = 160;

// Motor bias - YOUR ORIGINAL
const float RIGHT_MOTOR_BIAS = 1.15;

// Turn calibration multipliers (adjusted for 113.5mm wheelbase)
const float TURN_MULTIPLIER_RIGHT = 0.626;  // Adjusted: 0.643 * (113.5/110.5)
const float TURN_MULTIPLIER_LEFT = 0.52;   // Adjusted: 0.6365 * (113.5/110.5)

// Ramping - YOUR ORIGINAL
const int TURN_ACCEL_MS = 400;
const int TURN_DECEL_TICKS = 30;

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
volatile int lastAR = LOW, lastAL = LOW;

float mm_per_tick;
float error_sum = 0;
float last_error = 0;

// ========== ENCODERS ==========
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

// ========== MOTORS ==========
void setMotors(int l, int r) {
  l = constrain(l, -MAX_SPEED, MAX_SPEED);
  r = constrain(r, -MAX_SPEED, MAX_SPEED);
  
  // LEFT MOTOR (ENB, IN3, IN4)
  if (l >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, l);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -l);
  }
  
  // RIGHT MOTOR (ENA, IN1, IN2)
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
}

// ========== WAIT FUNCTION ==========
void wait(float seconds) {
  delay((int)(seconds * 1000));
}

// ========== GO STRAIGHT - YOUR ORIGINAL CODE ==========
void goStraight(float distance_mm) {
  long target = (long)(distance_mm / mm_per_tick);
  
  rCnt = 0;
  lCnt = 0;
  error_sum = 0;
  last_error = 0;
  
  unsigned long last_update = millis();
  
  while (abs(rCnt) < target && abs(lCnt) < target) {
    
    // Calculate position error
    long position_error = lCnt - rCnt;  // Positive = left is ahead
    
    // Check if we're outside tolerance
    if (abs(position_error) > POSITION_TOLERANCE) {
      // We're drifting apart - correct aggressively
      
      // Time-based PID
      unsigned long now = millis();
      float dt = (now - last_update) / 1000.0;
      last_update = now;
      
      if (dt > 0) {
        error_sum += position_error * dt;
        error_sum = constrain(error_sum, -100, 100);
        
        float derivative = (position_error - last_error) / dt;
        last_error = position_error;
        
        float correction = KP * position_error + KI * error_sum + KD * derivative;
        
        // Apply correction
        int left_speed = BASE_SPEED - correction;
        int right_speed = BASE_SPEED + correction;
        
        setMotors(left_speed, right_speed);
      }
    } else {
      // Within tolerance - just go at base speed
      setMotors(BASE_SPEED, BASE_SPEED);
      last_error = position_error;
    }
    
    delay(5);  // Check very frequently (every 5ms)
  }
  
  stopMotors();
  delay(500);
}

// ========== TURN RIGHT - YOUR ORIGINAL CODE ==========
void turnRight(float degrees) {
  // In-place turn: left wheel forward, right wheel backward
  float turn_circumference = PI * WHEEL_BASE_MM;
  float arc_distance = (degrees / 360.0) * turn_circumference * TURN_MULTIPLIER_RIGHT;
  long target_ticks = (long)(arc_distance / mm_per_tick);
  
  rCnt = 0;
  lCnt = 0;
  error_sum = 0;
  last_error = 0;
  
  unsigned long start_time = millis();
  unsigned long last_update = millis();
  
  // Keep turning until BOTH wheels reach target
  while (abs(lCnt) < target_ticks || abs(rCnt) < target_ticks) {
    unsigned long elapsed = millis() - start_time;
    long avg_ticks = (abs(lCnt) + abs(rCnt)) / 2;
    long remaining = target_ticks - avg_ticks;
    
    // Ramp speed up and down
    int current_speed = TURN_SPEED_MAX;
    
    // Acceleration
    if (elapsed < TURN_ACCEL_MS) {
      current_speed = map(elapsed, 0, TURN_ACCEL_MS, TURN_SPEED_MIN, TURN_SPEED_MAX);
      current_speed = constrain(current_speed, TURN_SPEED_MIN, TURN_SPEED_MAX);
    }
    
    // Deceleration
    if (remaining < TURN_DECEL_TICKS) {
      int decel_speed = map(remaining, 0, TURN_DECEL_TICKS, TURN_SPEED_MIN, TURN_SPEED_MAX);
      current_speed = min(current_speed, decel_speed);
      current_speed = constrain(current_speed, TURN_SPEED_MIN, TURN_SPEED_MAX);
    }
    
    // Sync correction: both wheels should travel same distance
    long position_error = abs(lCnt) - abs(rCnt);
    
    unsigned long now = millis();
    float dt = (now - last_update) / 1000.0;
    last_update = now;
    
    if (dt > 0) {
      error_sum += position_error * dt;
      error_sum = constrain(error_sum, -50, 50);
      
      float derivative = (position_error - last_error) / dt;
      last_error = position_error;
      
      float correction = 3.0 * position_error + 0.4 * error_sum + 0.8 * derivative;
      correction = constrain(correction, -50, 50);
      
      // RIGHT TURN: Left backward (-), Right forward (+) - SWAPPED!
      int left_speed = -(current_speed + correction);  // Left goes BACKWARD
      int right_speed = (current_speed - correction) * RIGHT_MOTOR_BIAS;  // Right goes FORWARD with boost
      
      setMotors(left_speed, right_speed);
    }
    
    delay(10);
  }
  
  stopMotors();
  delay(1000);
}

// ========== TURN LEFT - YOUR ORIGINAL CODE WITH POWER BOOST ==========
void turnLeft(float degrees) {
  // In-place turn: left wheel backward, right wheel forward
  float turn_circumference = PI * WHEEL_BASE_MM;
  float arc_distance = (degrees / 360.0) * turn_circumference * TURN_MULTIPLIER_LEFT;
  long target_ticks = (long)(arc_distance / mm_per_tick);
  
  rCnt = 0;
  lCnt = 0;
  error_sum = 0;
  last_error = 0;
  
  unsigned long start_time = millis();
  unsigned long last_update = millis();
  
  // Keep turning until BOTH wheels reach target
  while (abs(lCnt) < target_ticks || abs(rCnt) < target_ticks) {
    unsigned long elapsed = millis() - start_time;
    long avg_ticks = (abs(lCnt) + abs(rCnt)) / 2;
    long remaining = target_ticks - avg_ticks;
    
    // Ramp speed up and down - USING BOOSTED SPEEDS
    int current_speed = LEFT_TURN_SPEED_MAX;
    
    // Acceleration
    if (elapsed < TURN_ACCEL_MS) {
      current_speed = map(elapsed, 0, TURN_ACCEL_MS, LEFT_TURN_SPEED_MIN, LEFT_TURN_SPEED_MAX);
      current_speed = constrain(current_speed, LEFT_TURN_SPEED_MIN, LEFT_TURN_SPEED_MAX);
    }
    
    // Deceleration
    if (remaining < TURN_DECEL_TICKS) {
      int decel_speed = map(remaining, 0, TURN_DECEL_TICKS, LEFT_TURN_SPEED_MIN, LEFT_TURN_SPEED_MAX);
      current_speed = min(current_speed, decel_speed);
      current_speed = constrain(current_speed, LEFT_TURN_SPEED_MIN, LEFT_TURN_SPEED_MAX);
    }
    
    // Sync correction: both wheels should travel same distance
    long position_error = abs(lCnt) - abs(rCnt);
    
    unsigned long now = millis();
    float dt = (now - last_update) / 1000.0;
    last_update = now;
    
    if (dt > 0) {
      error_sum += position_error * dt;
      error_sum = constrain(error_sum, -50, 50);
      
      float derivative = (position_error - last_error) / dt;
      last_error = position_error;
      
      float correction = 3.0 * position_error + 0.4 * error_sum + 0.8 * derivative;
      correction = constrain(correction, -50, 50);
      
      // LEFT TURN: Left forward (+), Right backward (-)
      // Since right motor is weaker, when it goes backward it needs the bias
      int left_speed = (current_speed + correction);  // Left goes FORWARD
      int right_speed = -(current_speed - correction) * RIGHT_MOTOR_BIAS;  // Right goes BACKWARD with boost
      
      setMotors(left_speed, right_speed);
    }
    
    delay(10);
  }
  
  stopMotors();
  delay(1000);
}

// ========== SETUP ==========
void setup() {
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
  
  stopMotors();
  
  float wheel_circumference = PI * WHEEL_DIAMETER_MM;
  mm_per_tick = wheel_circumference / PULSES_PER_REV;
  
  delay(3000);
}

// ========== LOOP ==========
void loop() {
  // Example usage:
  int T = 500*1.0;//calibrate
  goStraight(3.5*T);
  wait(1);             // wait 2 seconds
  turnRight(90);       // 90° right
  wait(1);  
  turnRight(90);       // 90° right
  wait(1);  
  turnRight(90);       // 90° right
  wait(1);  
  turnRight(90);       // 90° right
  wait(1);  
  /*
  goStraight(0.5*T);    // 100cm
  wait(1);             // wait 2 seconds
  turnRight(90);       // 90° right
  wait(1);     
  turnRight(90);       // 90° right
  wait(1);     
  turnRight(90);       // 90° right
  wait(1);       
  goStraight(1*T);    // 100cm
  wait(1);  
  turnRight(90);       // 90° right
  wait(1); 
  goStraight(3*T);   
  wait(1);
  turnRight(90);       // 90° right
  wait(1); 
  goStraight(4*T);   
  wait(1);
  turnRight(90);       // 90° right
  wait(1); 
  goStraight(2*T);   
  wait(1);
  turnRight(90);       // 90° right
  wait(1);
  goStraight(2*T);   
  wait(1);
  turnRight(90);       // 90° right
  wait(1);
  goStraight(1*T);   
  wait(1);
  turnRight(90);       // 90° right
  wait(1);
  goStraight(1*T);
 
  


  //FAILING RUNNING THING
  //goStraight(1000000);
  */

              // wait 1.5 seconds
       // 90° left
        //Potential time waste?
        /*
  for(int i = 0; i<=5; i++){
    turnRight(90);       // 90° right
  wait(1);
  turnRight(90);       // 90° right
  wait(1);
  turnRight(90);       // 90° right
  wait(1);
  turnRight(90);       // 90° right
  wait(1);
  }
  */
  while(true) {
    delay(1000);
  }
}
