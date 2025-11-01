const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
float readings[10];
int counter = 0;
float value = 0.0;
float sum = 0.0;
float distance = 0.0;

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

void setup() {
    Serial.begin(9600);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    for (int i = 0; i < 10; i++) {
        readings[i] = check_distance();
        sum += readings[i];
    }
}

void loop() {
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
    Serial.println(distance);
    delay(50); 
}
