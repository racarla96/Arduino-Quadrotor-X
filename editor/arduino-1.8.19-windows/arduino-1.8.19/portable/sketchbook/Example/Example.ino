#include "Servo.h"
//#include "Servo_200Hz.h"

Servo myservo;

void setup() {
  Serial.begin(115200);

  myservo.attach(2);
  myservo.writeMicroseconds(2000);
  
  pinMode(13, INPUT);
}

void loop() {
  Serial.println(digitalRead(13));
  delayMicroseconds(500);  
}
