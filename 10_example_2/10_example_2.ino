#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;
int angle = 0;
int dir = 90;  // +90 올라감, -90 내려감

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.write(0);
  delay(1000);
}

void loop() {
//  myservo.write(angle);
//  delay(1000);
//
//  angle += dir;
//  if (angle >= 180) {
//    angle = 180;
//    dir = -90;
//  } else if (angle <= 0) {
//    angle = 0;
//    dir = 90;
//  }
}
