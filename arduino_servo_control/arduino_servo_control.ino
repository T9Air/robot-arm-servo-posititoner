#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  servo1.attach(10);
  servo2.attach(11);
}

void loop() {
  delay(15);
  servo1.write(180);
  servo2.write(0);
}
