#include <Arduino.h>

#define TRIG_PIN 17
#define ECHO_PIN 16

const int NUM_READINGS = 10;
float readings[NUM_READINGS];
int readIdx = 0;  // Changed from 'index'
float total = 0.0;

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

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Fill buffer with initial readings
  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = check_distance();
    total += readings[i];
    delay(30);
  }
}

void loop() {
  float dist = getDistance();
  
  Serial.print("Distance: ");
  Serial.print(dist);
  Serial.println(" cm");
  
  delay(50);
}