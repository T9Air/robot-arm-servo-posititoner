#include <Servo.h>

// Joint offsets
double _joint2_x; double _joint2_y; double _joint2_z;
double _joint3_x; double _joint3_y; double _joint3_z;
double _end_x; double _end_y; double _end_z;
// Arm length
double _arm1length; double _arm2length; double _totalXOffset;
// Array to hold angles or position
double angleorposition[3]; 

// Servos
Servo base; // Pin 6
Servo bottom1; // Pin 9
Servo bottom2; // Pin 10
Servo top; // Pin 11

// Servo min, max and center positions in microseconds
// Will not be the exact for the arm being used - just for testing
int baseMin = 1000; int baseMax = 2000; int baseCenter = 1500;
int bottom1Min = 1000; int bottom1Max = 2000; int bottom1Center = 1500;
// bottom2 min and max are reversed because it is moving the opposite way of bottom1
int bottom2Min = 2000; int bottom2Max = 1000; int bottom2Center = 1500;
int topMin = 1000; int topMax = 2000; int topCenter = 1500;

// Potentiometer for move amount
int pot = A0;

// Buttons
int up = 2;
int down = 3;
int right = 4;
int left = 5;
int forward = 7;
int backward = 8;

// Alarm led - tells if goal is unable to be reached
int alarmLED = 12;

// Current position
double current_x;
double current_y;
double current_z;

// Target position
double target_x;
double target_y;
double target_z;

// Move amount in mm
double move_amount = 1;

void setup() {
  // Set up servos
  base.attach(6);
  base.writeMicroseconds(baseCenter);
  bottom1.attach(9);
  bottom1.writeMicroseconds(bottom1Center);
  bottom2.attach(10);
  bottom2.writeMicroseconds(bottom2Center);
  top.attach(11);
  top.writeMicroseconds(topCenter);

  // Determine the current position of the arm
  int base_deg = baseMicroDeg(base.readMicroseconds());
  int bottom1_deg = bottom1MicroDeg(bottom1.readMicroseconds());
  int top_deg = topMicroDeg(top.readMicroseconds());

  // Set up buttons
  pinMode(up, INPUT);
  pinMode(down, INPUT);
  pinMode(right, INPUT);
  pinMode(left, INPUT);
  pinMode(forward, INPUT);
  pinMode(backward, INPUT);

  // Set up alarm led
  pinMode(alarmLED, OUTPUT);

  // Set the position of the arm joints
  // Will not be the exact for the arm being used - just for testing
  setJointOffsets(0,0,0, // Joint 1-2
                  10,0,100, // Joint 2-3
                  0,0,100); // Joint 3-end

  determinePosition(base_deg, bottom1_deg, top_deg);
  current_x = angleorposition[0];
  target_x = angleorposition[0];
  current_y = angleorposition[1];
  target_y = angleorposition[1];
  current_z = angleorposition[2];
  target_z = angleorposition[2];
}

void loop() {
  int potValue = analogRead(pot);
  move_amount = map(potValue, 0, 1023, .1, 10); // Map potentiometer value to move amount

  if (digitalRead(up) == HIGH) {
    target_z += move_amount;
  } else if (digitalRead(down) == HIGH) {
    target_z -= move_amount;
  } else if (digitalRead(right) == HIGH) {
    target_x += move_amount;
  } else if (digitalRead(left) == HIGH) {
    target_x -= move_amount;
  } else if (digitalRead(forward) == HIGH) {
    target_y += move_amount;
  } else if (digitalRead(backward) == HIGH) {
    target_y -= move_amount;
  }

  // Only adjust angles if target position is changed
  if (target_x != current_x || target_y != current_y || target_z != current_z) {
    determineAngles(target_x, target_y, target_z);
    if (angleorposition[0] != 1000) {
      
    } else {
      target_x = current_x;
      target_y = current_y;
      target_z = current_z;
      digitalWrite(alarmLED, HIGH);
      delay(2000);
      digitalWrite(alarmLED, LOW);
    }
  }
}

void setJointOffsets(double joint2_x, double joint2_y, double joint2_z,
                              double joint3_x, double joint3_y, double joint3_z,
                              double end_x, double end_y, double end_z) {
  // Set the offsets for each joint
  _joint2_x = joint2_x; _joint2_y = joint2_y; _joint2_z = joint2_z;
  _joint3_x = joint3_x; _joint3_y = joint3_y; _joint3_z = joint3_z;
  _end_x = end_x; _end_y = end_y; _end_z = end_z;
  _arm1length = sqrt(_sqr(_joint3_x) + _sqr(_joint3_z)); // Length of arm1
  _arm2length = sqrt(_sqr(_end_x) + _sqr(_end_z)); // Length of arm2 
  _totalXOffset = _joint2_x + _joint3_x + _end_x;
}

void determineAngles(double x, double y, double z){
  double radiusXY = sqrt(_sqr(x) + _sqr(y));
  double hypotenuseXY = sqrt(_sqr(radiusXY - x) + _sqr(y));
  double baseAngle = _lawOfCosines(hypotenuseXY, radiusXY, radiusXY);
  double extendedBaseLength = sqrt(_sqr(radiusXY) + _sqr(_totalXOffset));
  double extendedBaseAngle = asin(_totalXOffset / extendedBaseLength) * (180.0 / M_PI);

  double baseToEffector = sqrt(_sqr(radiusXY) + _sqr(z));
  double angleA = _lawOfCosines(_arm1length, baseToEffector, _arm2length);
  double angleB = _lawOfCosines(_arm2length, _arm1length, baseToEffector);
  double angleC = 180 - (angleA + angleB);
  
  double joint_1 = baseAngle + extendedBaseAngle;
  double joint_2 = 180 - (atan2(z, radiusXY) * (180.0 / M_PI) + angleB);
  double joint_3 = 270 - angleC;

  if (joint_1 > 180 || joint_1 < 0 || 
      joint_2 > 180 || joint_2 < 0 ||
      joint_3 > 180 || joint_3 < 0) {
    angleorposition[0] = 1000;
    angleorposition[1] = 1000;
    angleorposition[2] = 1000;
  } else {
    angleorposition[0] = joint_1;
    angleorposition[1] = joint_2;
    angleorposition[2] = joint_3;
  }
}

void determinePosition(double joint_1, double joint_2, double joint_3){
  double joint_3_rad = joint_3 * (M_PI / 180.0);
  double hypotenuseXZ = sqrt(_sqr(_arm1length) + _sqr(_arm2length) - (2*_arm1length*_arm2length*cos(joint_3_rad)));
  double baseAngle = _lawOfCosines(_arm1length, hypotenuseXZ, _arm2length);
  double outerAngle = (180 - (baseAngle + joint_2)) * (M_PI / 180.0);
  double radiusXY = hypotenuseXZ * sin(outerAngle);
  double z = hypotenuseXZ * cos(outerAngle);
  
  double extendedBaseLength = sqrt(_sqr(_totalXOffset) + _sqr(radiusXY));
  double extendedBaseAngle = asin(_totalXOffset / extendedBaseLength) * (180.0 / M_PI);
  joint_1 = joint_1 - extendedBaseAngle;
  double joint_1_rad = joint_1 * (M_PI / 180.0);
  double x = radiusXY * cos(joint_1_rad);
  double y = radiusXY * sin(joint_1_rad);

  x = x + _totalXOffset;
  z = z + _joint2_z;
  
  angleorposition[0] = x;
  angleorposition[1] = y;
  angleorposition[2] = z;
}

double _lawOfCosines(double a, double b, double c){
  double base = (_sqr(a) + _sqr(b) - _sqr(c)) / (2*a*b);
  if (base > 1.0) {
    base = 1.0;
  } else if (base < -1.0) {
    base = -1.0;
  }
  double angle = acos(base);
  double angleDegrees = angle * (180.0 / M_PI);
  return angleDegrees;
}

double _sqr(double num){
  return num*num;
}

// Turn microseconds into degrees
int baseMicroDeg(int micro){
  return map(micro, baseMin, baseMax, 0, 180);
}
int bottom1MicroDeg(int micro){
  return map(micro, bottom1Min, bottom1Max, 0, 180);
}
// Don't need to map bottom2 as it is the same as bottom1
int topMicroDeg(int micro){
  return map(micro, topMin, topMax, 0, 180);
}

// Turn degrees into microseconds
int baseDegMicro(int deg){
  return map(deg, 0, 180, baseMin, baseMax);
}
int bottom1DegMicro(int deg){
  return map(deg, 0, 180, bottom1Min, bottom1Max);
}
int bottom2DegMicro(int deg){
  return map(deg, 0, 180, bottom2Min, bottom2Max);
}
int topDegMicro(int deg){
  return map(deg, 0, 180, topMin, topMax);
}