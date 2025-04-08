// Joint offsets
double _joint2_x; double _joint2_y; double _joint2_z;
double _joint3_x; double _joint3_y; double _joint3_z;
double _end_x; double _end_y; double _end_z;
// Arm length
double _arm1length; double _arm2length; double _totalXOffset;
// Array to hold angles or position
double angleorposition[3]; 

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

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

double* determineAngles(double x, double y, double z){
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
  return angleorposition;
}

double* determinePosition(double joint_1, double joint_2, double joint_3){
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
  return angleorposition;
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