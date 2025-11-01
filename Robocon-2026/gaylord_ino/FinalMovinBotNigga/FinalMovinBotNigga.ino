//#include "include_all.cpp"
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

float distance = 0.0;
float minAng = 0.0;

struct MotorSpeeds {
  int m1, m2, m3, m4;
};

int isWallFound = 0;
float TARGET_DISTANCE_CM = 80.0;
int FORWARD_SPEED = 50;
float KP_DISTANCE = 0.3;

const int NUM_READINGS = 10;
float readings[NUM_READINGS];
int readIdx = 0;  // Changed from 'index'
float total = 0.0;

float ref_angle;
bool refSet = false;
float k_angle = 3.0;
float e = 360;

int sign_matrix[4][3] = {
  {1, 1, 1},   // Motor 1 (Top-Left)
  {1, -1, 1},    // Motor 2 (Top-Right)
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
  
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration == 0) return 0.0;
  
  return duration * 0.0343 / 2.0;
}

float getDistance() {
  float newDist = check_distance();
  
  // Ignore bad readings
  if (newDist == 0.0 || newDist > 400.0) {
    return total / NUM_READINGS;
  }
  
  // Update moving average
  total = total - readings[readIdx] + newDist;
  readings[readIdx] = newDist;
  readIdx = (readIdx + 1) % NUM_READINGS;
  
  return total / NUM_READINGS;
}

float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
} 
 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); 


int bno_pid(float target_angle) {
  sensors_event_t event;
  bno.getEvent(&event);
  float theta = event.orientation.x;
  float error = normalizeAngle(target_angle - theta);
  float w = k_angle * error;
  w = constrain((int)w, -255, 255);
  // MotorSpeeds turnMotors = calculateMotorVelocities(0, 0, -w);
  // setMotorSpeeds(turnMotors);
  return w;

}

void setMotorSpeeds(MotorSpeeds motor) {
  int v11 = constrain(motor.m1, -120, 120);
  int v22 = constrain(motor.m2, -120, 120);
  int v33 = constrain(motor.m3, -120, 120);
  int v44 = constrain(motor.m4, -120, 120);

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



void findWall(){
  Serial.println("Finding the damn wall");
  float turnang = 10.0;

  sensors_event_t event;
  bno.getEvent(&event);
  float currref = event.orientation.x;

  int wleft = bno_pid(currref - turnang);

  MotorSpeeds turnLeft = calculateMotorVelocities(0, 0, -wleft);
  setMotorSpeeds(turnLeft);

  delay(500);

  MotorSpeeds Stop1 = calculateMotorVelocities(0,0,0);
  setMotorSpeeds(Stop1);

  delay(500);

  Serial.println("Finished turning left.");

  Serial.println("Starting to scan");
  float mindist = 1000.0;

  MotorSpeeds turnRight = calculateMotorVelocities(0,0,100);
  setMotorSpeeds(turnRight);

  float targetAng = currref + turnang;
  float minAng = currref - turnang;

  while (true){

    sensors_event_t event;
    bno.getEvent(&event);
    float angrn = event.orientation.x;

    float dist = getDistance();
    Serial.println("Distance: ");
    Serial.println(dist);

    if (dist<mindist){
      mindist = dist;
      minAng = angrn;
    }

    if (mindist < 1000.0 && dist - mindist > 1){
      MotorSpeeds Stop2 = calculateMotorVelocities(0,0,0);
      setMotorSpeeds(Stop2);
      break;
    }

    if (abs(normalizeAngle(angrn - targetAng)) < 0.5){
      MotorSpeeds Stop3 = calculateMotorVelocities(0,0,0);
      setMotorSpeeds(Stop3);
      break;
    }

    delay(50);

  }

  int wmin = bno_pid(minAng);
  MotorSpeeds turnMotors = calculateMotorVelocities(0, 0, -wmin);
  setMotorSpeeds(turnMotors);

  delay(500);

  MotorSpeeds Stop4 = calculateMotorVelocities(0,0,0);
  setMotorSpeeds(Stop4);

}


void followWall() {
    Serial.println("Following Wall");
    long startTime = millis();
    while (millis() - startTime < 10000) {

        float currentDist = getDistance();
        float distError = TARGET_DISTANCE_CM - currentDist;
        //Serial.println("distError");
        Serial.println(distError);
        int w1 = bno_pid(minAng);
        int vy_correction = constrain(KP_DISTANCE * distError, -80, 80);
        //Serial.println(vy);
  
        sensors_event_t event;
        bno.getEvent(&event);
        float current_angle = event.orientation.x;
        float error = normalizeAngle(minAng - current_angle);

        // Serial.print("Target: ");
        // Serial.print(ref_angle);
        // Serial.print(" | Current: ");
        // Serial.print(current_angle);
        // Serial.print(" | Error: ");
        // Serial.println(error);
        // Serial.println(" | Distance: ");
        // Serial.println(currentDist);
  
        MotorSpeeds speeds = calculateMotorVelocities(0, -vy_correction, 0);
        setMotorSpeeds(speeds);
        delay(20);
    }

    MotorSpeeds Stop5 = calculateMotorVelocities(0,0,0);
    setMotorSpeeds(Stop5);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
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

  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = check_distance();
    total += readings[i];
    delay(30);
  }


  Serial.println("Initializing BNO055...");
  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring.");
    while (1) delay(10);
  }
  
  Serial.println("BNO055 initialized successfully!");
  bno.setExtCrystalUse(true);
  
  // Capture reference angle at startup
  // sensors_event_t event;
  // bno.getEvent(&event);
  // ref_angle = event.orientation.x;
  
  // Serial.print("Reference angle set to: ");
  // Serial.println(ref_angle);

  followWall();

  delay(2000);
}

void loop() {
  // Continuously maintain the reference angle
  //bno_pid(ref_angle);

  float dist1 = getDistance();
    Serial.println("Distance: ");
    Serial.println(dist1);

  
  
  // Optional: Print current angle and error for debugging
  // sensors_event_t event;
  // bno.getEvent(&event);
  // float current_angle = event.orientation.x;
  // float error = normalizeAngle(ref_angle - current_angle);
  
  // Serial.print("Target: ");
  // Serial.print(ref_angle);
  // Serial.print(" | Current: ");
  // Serial.print(current_angle);
  // Serial.print(" | Error: ");
  // Serial.println(error);
  
  delay(20);  // Fast update rate for stable control
}