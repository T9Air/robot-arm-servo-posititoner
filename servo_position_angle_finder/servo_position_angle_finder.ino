// Joint offsets
double _joint2_x; double _joint2_y; double _joint2_z;
double _joint3_x; double _joint3_y; double _joint3_z;
double _end_x; double _end_y; double _end_z;
// Arm length
double _arm1length; double _arm2length;

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
  _arm1length = sqrt(_joint3_x*_joint3_x + _joint3_z*_joint3_z); // Length of arm1
  _arm2length = sqrt(_end_x*_end_x + _end_z*_end_z); // Length of arm2 
}

double* determineAngles(double x, double y, double z){
  double angles[3];
  
  double radiusXY = sqrt(x*x + y*y);
  double hypotenuseXY = sqrt(square(radiusXY - x) + square(y));
  double baseAngle = _lawOfCosines(hypotenuseXY, radiusXY, radiusXY);
  double totalXOffset = _joint2_x + _joint3_x + _end_x;
  double extendedBaseLength = sqrt(square(radiusXY) + square(totalXOffset));
  double extendedBaseAngle = asin(totalXOffset / extendedBaseLength);

  double baseToEffector = sqrt(square(radiusXY) + square(z));
  double angleA = _lawOfCosines(_arm1length, baseToEffector, _arm2length);
  double angleB = _lawOfCosines(_arm2length, _arm1length, baseToEffector);
  double angleC = 180 - (angleA + angleB);
  
  double joint_1 = baseAngle + extendedBaseAngle;
  double joint_2 = 180 - (tan(z / radiusXY) + angleB);
  double joint_3 = 270 - angleC;

  if (joint_1 > 180 || joint_1 < 0 || 
      joint_2 > 180 || joint_2 < 0 ||
      joint_3 > 180 || joint_3 < 0) {
    angles[0] = 1000;
    angles[1] = 1000;
    angles[2] = 1000;
  } else {
    angles[0] = joint_1;
    angles[1] = joint_2;
    angles[2] = joint_3;
  }
  return angles;
}

double _lawOfCosines(double a, double b, double c){
  double base = (square(a) + square(b) - square(c)) / (2*a*b);
  if (base > 1.0) {
    base = 1.0;
  } else if (base < -1.0) {
    base = -1.0;
  }
  double angle = acos(base);
  double angleDegrees = angle * (180.0 / M_PI);
  return angleDegrees;
}

double _square(double num){
  return num*num;
}