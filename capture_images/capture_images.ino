#include <Servo.h>

Servo base;
Servo bottom1;
Servo bottom2;
Servo top;

void setup() {
  Serial.begin(9600);

  base.attach(6);
  bottom1.attach(9);
  bottom2.attach(10);
  top.attach(11);
  bottom1.write(90);
  bottom2.write(90);
  top.write(90);
}

void checker() {
  while (Serial.available() == 0) {}
}

void loop() {
  base.write(0);
  delay(1000);
  Serial.println("go");
  checker();
  base.write(90);
  delay(1000);
  Serial.println("go");
  checker();
  base.write(180);
  delay(1000);
  Serial.println("go");
  checker();
  base.write(0);
  delay(1000);
  for(int i = 500; i > 2500; i++){
    base.writeMicroseconds(i);
    delay(1);
    Serial.println("go");
    checker();
  }
  Serial.println("done");
  delay(100000);
}
