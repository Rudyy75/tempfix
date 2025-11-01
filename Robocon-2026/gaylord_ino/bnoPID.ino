#include "include_all.cpp"
#include <Wire.h>
#include <Adafruit_BNO055.h>


#define M1_PWM 32
#define M1_DIR 33
#define M2_LPWM 13
#define M2_RPWM 12
#define M3_PWM 14
#define M3_DIR 27
#define M4_PWM 26
#define M4_DIR 25

#define TRIG_PIN 17
#define ECHO_PIN 16

#define SDA_PIN 21
#define SCL_PIN 22

float readings[10];
int counter = 0;
float value = 0.0;
float sum = 0.0;
float distance = 0.0;

struct MotorSpeeds {
  int m1, m2, m3, m4;
};

int isWallFound = 0;
float TARGET_DISTANCE_CM = 30.0;
int FORWARD_SPEED = 140;
float KP_DISTANCE = 3.0;
float KD_angle = 5.0;
const int SCAN_SPEED = 60;
const int turnDuration = 1500;
int SPEEED = 0;

const int NUM_READINGS = 10;
const int TRIGGER_INTERVAL_MS = 60;
float ref_angle;
bool refSet = false;
float k_angle = 5.0;
float e = 360;

int sign_matrix[4][3] = {
  {1, -1, 1},   // Motor 1 (Top-Left)
  {1, 1, 1},    // Motor 2 (Top-Right)
  {-1, 1, 1},   // Motor 3 (Bottom-Left)
  {-1, -1, 1}   // Motor 4 (Bottom-Right)
};

MotorSpeeds calculateMotorVelocities(int vx, int vy, int vw) {
    MotorSpeeds motors;
    int velocity_vector[3] = {vx, vy, vw};
    int motor_velocities[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            motor_velocities[i] += sign_matrix[i][j] * velocity_vector[j];
        }
    }
    motors.m1 = motor_velocities[0];
    motors.m2 = motor_velocities[1];
    motors.m3 = motor_velocities[2];
    motors.m4 = motor_velocities[3];
    return motors;
}

float check_distance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH); 
    
    value = duration * 0.0343 / 2.0; 
    delay(5); 
    return value;
}

float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
} 
 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); 


void bno_pid(float target_angle) {
  sensors_event_t event;
  bno.getEvent(&event);
  float theta = event.orientation.x;
  float error = normalizeAngle(target_angle - theta);
  float w = k_angle * error;
  w = constrain((int)w, -255, 255);
  MotorSpeeds turnMotors = calculateMotorVelocities(0, 0, w);
  setMotorSpeeds(turnMotors);
}

void setMotorSpeeds(MotorSpeeds motor) {
  int v11 = constrain(motor.m1, -255, 255);
  int v22 = constrain(motor.m2, -255, 255);
  int v33 = constrain(motor.m3, -255, 255);
  int v44 = constrain(motor.m4, -255, 255);

  if (motor.m1 < 0) {
    digitalWrite(M1_DIR, LOW);
    analogWrite(M1_PWM, abs(v11));
  } else if (motor.m1 > 0) {
    digitalWrite(M1_DIR, HIGH);
    analogWrite(M1_PWM, v11);
  } else {
    analogWrite(M1_PWM, 0);
  }
  
  if (motor.m2 < 0) {
    analogWrite(M2_LPWM, 0);
    analogWrite(M2_RPWM, abs(v22));
  } else if (motor.m2 > 0) {
    analogWrite(M2_LPWM, abs(v22));
    analogWrite(M2_RPWM, 0);
  } else {
    analogWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, 0);
  }
  
  if (motor.m3 < 0) {
    digitalWrite(M3_DIR, LOW);
    analogWrite(M3_PWM, abs(v33));
  } else if (motor.m3 > 0) {
    digitalWrite(M3_DIR, HIGH);
    analogWrite(M3_PWM, v33);
  } else {
    analogWrite(M3_PWM, 0);
  }
  
  if (motor.m4 < 0) {
    digitalWrite(M4_DIR, LOW);
    analogWrite(M4_PWM, abs(v44));
  } else if (motor.m4 > 0) {
    digitalWrite(M4_DIR, HIGH);
    analogWrite(M4_PWM, v44);
  } else {
    analogWrite(M4_PWM, 0);
  }
}

float getStableDistanceCm() {
    value = check_distance();
    
    if (value != 0 && abs(value - sum / 10.0) < 20.0) { 
        sum = sum - readings[counter] + value;
        readings[counter] = value;
        counter++;
    }
    
    if (counter == 10) { 
        counter = 0;
    }
    
    distance = sum / 10.0; 
    delay(50); 
    return distance;
}

void followWall() {
    Serial.println("Following Wall");
    long startTime = millis();
    while (millis() - startTime < 8000) {
        float currentDist = getStableDistanceCm();
        float distError = TARGET_DISTANCE_CM - currentDist;
        sensors_event_t event;
        bno.getEvent(&event);
  
        float heading = event.orientation.x;
        float error_angle = ref_angle - heading;
        float SPEED_ANG = k_angle * (error_angle);
        SPEED_ANG = constrain(SPEED_ANG, -255, 255);
        int vy_correction = constrain(KP_DISTANCE * distError, -170, 170);
        
        MotorSpeeds speeds = calculateMotorVelocities(FORWARD_SPEED, vy_correction, SPEED_ANG);
        setMotorSpeeds(speeds);
        delay(20);
    }
    setMotorSpeeds({0, 0, 0, 0});
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  for (int i = 0; i < 10; i++) {
        readings[i] = check_distance();
        sum += readings[i];
  }

  delay(100);

  Serial.println("Initializing BNO055...");
  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring.");
    while (1) delay(10);
  }
  
  Serial.println("BNO055 initialized successfully!");
  bno.setExtCrystalUse(true);
  
  // Capture reference angle at startup
  sensors_event_t event;
  bno.getEvent(&event);
  ref_angle = event.orientation.x;
  
  Serial.print("Reference angle set to: ");
  Serial.println(ref_angle);

  delay(2000);
}

void loop() {
  // Continuously maintain the reference angle
  bno_pid(ref_angle);
  
  // Optional: Print current angle and error for debugging
  sensors_event_t event;
  bno.getEvent(&event);
  float current_angle = event.orientation.x;
  float error = normalizeAngle(ref_angle - current_angle);
  
  Serial.print("Target: ");
  Serial.print(ref_angle);
  Serial.print(" | Current: ");
  Serial.print(current_angle);
  Serial.print(" | Error: ");
  Serial.println(error);
  
  delay(20);  // Fast update rate for stable control
}