
#include <Wire.h>
//#include <Adafruit_BNO055.h>
//#include <Adafruit_Sensor.h>

#define M1_PWM 32
#define M1_DIR 33
#define M2_LPWM 13
#define M2_RPWM 12
#define M3_PWM 14
#define M3_DIR 27
#define M4_PWM 26
#define M4_DIR 25

#define TRIG_PIN 1
#define ECHO_PIN 3

struct MotorSpeeds {
  int m1, m2, m3, m4;
};

int isWallFound = 0;
float TARGET_DISTANCE_CM = 30.0;
int FORWARD_SPEED = 80;
float KP_DISTANCE = 3.0;
const int SCAN_SPEED = 60;
const int TURN_60_DEG_DURATION_MS = 600;

const int NUM_READINGS = 10;
const int TRIGGER_INTERVAL_MS = 60;

int sign_matrix[4][3] = {
  {-1, -1, 1},
  {1, -1, 1},
  {-1, 1, 1},
  {1, 1, 1}
};

float getRawDistanceCm();
MotorSpeeds calculateMotorVelocities(int vx, int vy, int vw);
void setMotorSpeeds(MotorSpeeds speeds);

void findWall() {
    float minDistance = 999.0;
    long minDistanceTime = 0;

    MotorSpeeds turnLeft = calculateMotorVelocities(0, 0, -SCAN_SPEED);
    setMotorSpeeds(turnLeft);
    delay(TURN_60_DEG_DURATION_MS);
    setMotorSpeeds({0, 0, 0, 0});
    delay(500);

    MotorSpeeds turnRight = calculateMotorVelocities(0, 0, SCAN_SPEED);
    setMotorSpeeds(turnRight);

    long scanStartTime = millis();
    while (millis() - scanStartTime < (TURN_60_DEG_DURATION_MS * 2)) {
        float currentDist = getRawDistanceCm();
        if (currentDist > 1 && currentDist < minDistance) {
            minDistance = currentDist;
            minDistanceTime = millis() - scanStartTime;
        }
        delay(20);
    }
    setMotorSpeeds({0, 0, 0, 0});
    delay(500);

    if (minDistance < 100.0) {
        long timeToTurnBack = (TURN_60_DEG_DURATION_MS * 2) - minDistanceTime;
        setMotorSpeeds(turnLeft);
        delay(timeToTurnBack);
        setMotorSpeeds({0, 0, 0, 0});
        isWallFound = 1;
    } else {
        isWallFound = 0;
    }
}

void followWall() {
    long startTime = millis();
    while (millis() - startTime < 5000) {
        float currentDist = getRawDistanceCm();
        float distError = TARGET_DISTANCE_CM - currentDist;
        int vy_correction = constrain(KP_DISTANCE * distError, -80, 80);

        MotorSpeeds speeds = calculateMotorVelocities(FORWARD_SPEED, vy_correction, 0);
        setMotorSpeeds(speeds);
        delay(20);
    }
    setMotorSpeeds({0, 0, 0, 0});
}

float getStableDistanceCm() {
  float sum = 0;
  int validReadingCount = 0;

  for (int i = 0; i < NUM_READINGS; i++) {
    float currentReading = getRawDistanceCm();
    if (currentReading > 0) {
      sum += currentReading;
      validReadingCount++;
    }
    delay(TRIGGER_INTERVAL_MS);
  }

  if (validReadingCount > 0) {
    return sum / validReadingCount;
  } else {
    return 0;
  }
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

  delay(100);

  findWall();

  delay(2000);
  if (isWallFound == 1) {
    followWall();
  }
}

void loop() {
  float distance = getStableDistanceCm();

  if (distance > 0) {
    //Serial.print("Stable distance: ");
    //Serial.print(distance, 2);
    //Serial.println(" cm");
  } else {
    //Serial.println("Failed to get a valid reading.");
  }
  delay(1000);
}

MotorSpeeds calculateMotorVelocities(int vx, int vy, int vw) {
    MotorSpeeds speeds;
    int velocity_vector[3] = {vx, vy, vw};
    int motor_velocities[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            motor_velocities[i] += sign_matrix[i][j] * velocity_vector[j];
        }
    }
    speeds.m1 = motor_velocities[0];
    speeds.m2 = motor_velocities[1];
    speeds.m3 = motor_velocities[2];
    speeds.m4 = motor_velocities[3];
    return speeds;
}

void setMotorSpeeds(MotorSpeeds speeds) {
  int v1 = constrain(speeds.m1, -255, 255);
  int v2 = constrain(speeds.m2, -255, 255);
  int v3 = constrain(speeds.m3, -255, 255);
  int v4 = constrain(speeds.m4, -255, 255);

  digitalWrite(M1_DIR, v1 >= 0);
  analogWrite(M1_PWM, abs(v1));

  analogWrite(M2_LPWM, v2 > 0 ? v2 : 0);
  analogWrite(M2_RPWM, v2 < 0 ? abs(v2) : 0);

  digitalWrite(M3_DIR, v3 >= 0);
  analogWrite(M3_PWM, abs(v3));

  digitalWrite(M4_DIR, v4 >= 0);
  analogWrite(M4_PWM, abs(v4));
}

float getRawDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 / 2;

  if (distance < 2.0) {
    return 0;
  }
  return distance;
}
