#include <Arduino.h>
#include <ArduinoEigen.h>
#include <robot_manipulator.h>

double posorangles[3];

// Set up joints
JointParameters joints[] = {
  {0.0, 90.0, 0.0, 0.0},  // Joint 1
  {36.2, 0.0, 119.2, 0.0},  // Joint 2
  {0.0, 0.0, 127.5, 0.0}   // Joint 3
};
RobotManipulator robot(joints, 3);


void setup() {}

void loop() {}

void forward(double j1, double j2, double j3) {
  double jointAngles[] = {j1, j2, j3};
  ForwardKinematicsResult fkResult = robot.forwardKinematics(jointAngles);
  posorangles[0] = fkResult.position[0];
  posorangles[1] = fkResult.position[1];
  posorangles[2] = fkResult.position[2];
}

void inverse(double x, double y, double z){
  Eigen::Vector3d targetPosition(x, y, z);
  Eigen::Matrix3d targetOrientation;
  targetOrientation << 1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;

  InverseKinematicsResult ikResult = robot.inverseKinematics(targetPosition, targetOrientation);
  if (ikResult.success) {
    posorangles[0] = ikResult.jointAngles[0];
    posorangles[1] = ikResult.jointAngles[1];
    posorangles[2] = ikResult.jointAngles[2];
  } else {
    posorangles[0] = 1000;
    posorangles[1] = 1000;
    posorangles[2] = 1000;
  }
}