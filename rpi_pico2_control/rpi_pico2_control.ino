#include <Arduino.h>
#include <ArduinoEigen.h>
#include <robot_manipulator.h>

double current_x;
double current_y;
double current_z;
double target_x;
double target_y;
double target_z;
double posorangles[3];

// Set up joints
JointParameters joints[] = {
  {0.0, 90.0, 0.0, 0.0},  // Joint 1
  {36.2, 0.0, 119.2, 0.0},  // Joint 2
  {0.0, 0.0, 127.5, 0.0}   // Joint 3
};
RobotManipulator robot(joints, 3);


void setup() {
  Serial.begin(9600);
  forward(90, 90, 90);
  current_x = posorangles[0]; target_x = posorangles[0];
  current_y = posorangles[1]; target_y = posorangles[1];
  current_z = posorangles[2]; target_z = posorangles[2];
  testKinematics();
}

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

void testKinematics() {
  // Example test positions (x, y, z)
  Serial.println(current_x);
  Serial.println(current_y);
  Serial.println(current_z);
  double testPositions[][3] = {
    {current_x, current_y, current_z - 30},
    {current_x, current_y + 10, current_z - 30},
    {current_x - 20, current_y + 10, current_z - 60},
    {current_x - 50, current_y - 30, current_z - 60}
  };
  int numTests = sizeof(testPositions) / sizeof(testPositions[0]);

  Serial.println("Testing Forward and Inverse Kinematics:");
  for (int i = 0; i < numTests; i++) {
    double x = testPositions[i][0];
    double y = testPositions[i][1];
    double z = testPositions[i][2];

    // Inverse kinematics: position -> angles
    inverse(x, y, z);
    double joint_1 = posorangles[0];
    double joint_2 = posorangles[1];
    double joint_3 = posorangles[2];

    Serial.print("Target (x,y,z): ");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z);

    if (joint_1 == 1000 || joint_2 == 1000 || joint_3 == 1000) {
      Serial.println(" -> Out of reach");
      continue;
    }

    Serial.print(" | Angles: ");
    Serial.print(joint_1, 2); Serial.print(", ");
    Serial.print(joint_2, 2); Serial.print(", ");
    Serial.print(joint_3, 2);

    // Forward kinematics: angles -> position
    forward(joint_1, joint_2, joint_3);
    double fx = posorangles[0];
    double fy = posorangles[1];
    double fz = posorangles[2];

    Serial.print(" | FK (x,y,z): ");
    Serial.print(fx, 2); Serial.print(", ");
    Serial.print(fy, 2); Serial.print(", ");
    Serial.print(fz, 2);

    Serial.println();
  }
}