void setMotorSpeeds(Pwm motor) {
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