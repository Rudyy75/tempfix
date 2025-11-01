#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define TRIG_PIN 9
#define ECHO_PIN 10
float distance = 0.0;
float angle = 0.0;
float min_distance = 1000.0;
float min_angle = 0.0;
int flag = 0;
int prev_error = 0;
float sum =0.0 ;
float kp = 5;
float ki =1 ;
float kd = 1;
float dt = 0.1; //running main.cpp at 100 ms

//Top-Left Wheel
#define MOTOR1_PWM_PIN  32
#define MOTOR1_DIR_PIN  33  
#define SDA_PIN 21
#define SCL_PIN 22
//Top-Right Wheel
#define MOTOR2_LPWM_PIN  13
#define MOTOR2_RPWM_PIN  12

//Bottom-Left Wheel
#define MOTOR3_PWM_PIN  14
#define MOTOR3_DIR_PIN  27

//Bottom-Right Wheel
#define MOTOR4_PWM_PIN  26
#define MOTOR4_DIR_PIN  25


//Move this nigga to the creating struct which is ig constants part
int sign_matrix[4][3] = {
  {1, -1, 1},   // Motor 1 (Top-Left)
  {1, 1, 1},  // Motor 2 (Top-Right)
  {-1, 1, 1}, // Motor 3 (Bottom-Left)
  {-1, -1, 1}   // Motor 4 (Bottom-Right)
};

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

   if (!bno.begin()) {
    Serial.println("not working stopping");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

}

void loop() {
  
  if(!flag){
    for(int count = 0;count<20;count++){
    //rotate one dir for fixed time 
    check_distance();
    delay(10);
    if(min_distance> distance){
      min_distance = distance;
      //set speed as 0
      check_angle();
      min_angle = angle;
    }
    else{
      //turn other direction for fixed time 
    }
    }
  }

  if(flag){
    fix_angle();
    //go forward function for certain time
    fix_angle();
    fix_distance();
  }


}


void check_distance(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);

  float d = (duration * 0.0343) / 2;
  distance = d;
  delay(50);  

}

void check_angle(){
  sensors_event_t event;
  bno.getEvent(&event);
  angle = event.orientation.x;
}

float pid(float target,float current){
  float error = target - current;
  sum += error*dt;
  float output = kp*error + ki*sum + kd*(error-prev_error)/dt;
  prev_error = error;
  return output;
}

void fix_distance(){
  check_distance();
  while(!(fabs(distance - min_distance)<3)){
    check_distance();
    float dir_change = pid(min_distance, distance);
    //feed to motor
    delay(500);
  }
}

void fix_angle(){
  check_angle();
  while(!(fabs(angle - min_angle)<20)){
    check_angle();
    float angle_change = pid(min_angle, angle);
    //feed to motor to rotate
    delay(500);
  }
}