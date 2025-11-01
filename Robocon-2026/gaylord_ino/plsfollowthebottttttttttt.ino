//#include "include_all.cpp"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <TFLI2C.h>


#define M1_PWM 32
#define M1_DIR 33
#define M2_LPWM 13
#define M2_RPWM 12
#define M3_LPWM 14
#define M3_RPWM 27
#define M4_DIR 26
#define M4_PWM 25

float distance = 0.0;

struct MotorSpeeds {
  int m1, m2, m3, m4;
};

float TARGET_DISTANCE_CM = 80.0;
float KP_DISTANCE = 6.5;
float KD_DISTANCE = 0.65;

float prevDisterr = 0;
unsigned long prevtime = 0;

float ref_angle;
float k_angle = 2.0;
float e = 360;

TFLI2C tflI2C;
int dist;

int16_t tfDist;
int16_t tfAddr = TFL_DEF_ADR;

int sign_matrix[4][3] = {
  {1, -1, 1},   // Motor 1 (Top-Left)
  {1, 1, 1},    // Motor 2 (Top-Right)
  {1, 1, -1},   // Motor 3 (Bottom-Left)
  {-1, 1, 1}   // Motor 4 (Bottom-Right)
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
  w = constrain((int)w, -250, 250);
  return w;

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
    digitalWrite(M3_LPWM, 0);
    analogWrite(M3_RPWM, abs(v33));
    
  } else if (motor.m3 > 0) {
    analogWrite(M3_LPWM, abs(v33));
    analogWrite(M3_RPWM, 0);
    
  } else {
    analogWrite(M3_LPWM, 0);
    analogWrite(M3_RPWM, 0);
  }
  
  if (motor.m4 < 0) {
    digitalWrite(M4_DIR, LOW);
    analogWrite(M4_PWM, abs(v44));
    
  } else if (motor.m4 > 0) {
    digitalWrite(M4_DIR, HIGH);
    analogWrite(M4_PWM, abs(v44));
  
  } else {
    analogWrite(M4_PWM, 0);
  }
}

void followWall() {
   
    float currentDist;
  
         if (tflI2C.getData(tfDist, tfAddr)){
           Serial.print("dist: ");
           Serial.println((String(tfDist) + " cm; "));
           currentDist = tfDist;
         }

        unsigned long currTime = millis();
        float dt = (currTime - prevtime)/1000.0;

        float distError = TARGET_DISTANCE_CM - currentDist;

        float errDerivative;
        if (dt > 0) {
          errDerivative = (distError - prevDisterr)/(dt);
        }
        // Serial.println(distError);
        // Serial.println(prevDisterr);
        // Serial.println(errDerivative);
        
        
        int vy_correction = constrain(KP_DISTANCE * distError + KD_DISTANCE * errDerivative, -250, 250);
        

        if(abs(distError)<2)
          {
            vy_correction = 0;
          }

        prevDisterr = distError;
        prevtime = currTime;
       
        int w1 = bno_pid(ref_angle);
      
        // Serial.print("Vy: ");
        // Serial.println(-vy_correction);

        // Serial.println(KP_DISTANCE * distError);
        // Serial.println(KD_DISTANCE * errDerivative);
        // Serial.println(-w1*k_angle);
        
        MotorSpeeds speeds = calculateMotorVelocities(75, -vy_correction, -w1*k_angle);
        setMotorSpeeds(speeds);
        //delay(20);

}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M3_RPWM, OUTPUT);
  pinMode(M3_LPWM, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT);

  Serial.println("Initializing BNO055...");
  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring.");
    while (1) delay(10);
  }
  
  Serial.println("BNO055 initialized successfully!");
  bno.setExtCrystalUse(true);
  
  sensors_event_t event;
  bno.getEvent(&event);
  ref_angle = event.orientation.x;

  delay(200);
}

void loop() {
  
  followWall();
   // Fast update rate for stable control
}