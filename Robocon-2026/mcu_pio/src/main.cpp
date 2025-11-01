#include "include_all.cpp"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); 
TFLI2C tflI2C;
int16_t tfDist;
int16_t tfAddr = TFL_DEF_ADR;
sensors_event_t event;


void setup(){

    // serial begin
    Serial.begin(115200);

    Serial.println("Initializing BNO055...");
  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring.");
    while (1) delay(10);
  }
  
  Serial.println("BNO055 initialized successfully!");
  bno.setExtCrystalUse(true);
  

  bno.getEvent(&event);
  ref_angle = event.orientation.x;

  delay(200);

pinMode(M1_DIR, OUTPUT);
pinMode(M1_PWM, OUTPUT);
pinMode(M2_LPWM, OUTPUT);
pinMode(M2_RPWM, OUTPUT);
pinMode(M3_RPWM, OUTPUT);
pinMode(M3_LPWM, OUTPUT);
pinMode(M4_DIR, OUTPUT);
pinMode(M4_PWM, OUTPUT);
}

void loop() {

    // recieves data in every loop 
    std::vector<uint8_t> payload = receive_data();
    store_data(payload);

    bno.getEvent(&event);
    bnoreading.current_angle = event.orientation.x;

    if (tflI2C.getData(tfDist, tfAddr)){

           lidar.distance = tfDist;
         }


    setMotorSpeeds(pwm);
    pwmCheck.m1 = pwm.m1;
    pwmCheck.m2 = pwm.m2;
    pwmCheck.m3 = pwm.m3;
    pwmCheck.m4 = pwm.m4;



    // stores data, called every loop
    // store_data(payload);
    
    // spin the callbacks
    t1.spin();
}
