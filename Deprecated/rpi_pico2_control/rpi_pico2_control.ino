#include <Arduino.h>
#include <ArduinoEigen.h>
#include <robot_manipulator.h>

double current_x;
double current_y;
double current_z;
double target_x;
double target_y;
double target_z;
double posorangles[3] = {0.0, 0.0, 0.0};

// Set up joints - check documentation for proper order of parameters
// Common format is: {d, theta, a, alpha} or similar
JointParameters joints[] = {
  {0.0, -90.0, 0.0, 0.0},        // Joint 1
  {36.2, 0.0, 119.2, 0.0},    // Joint 2
  {0.0, 90.0, 127.5, 0.0}      // Joint 3
};
RobotManipulator robot(joints, 3);

void setup() {
  Serial.begin(9600);
  // Start with a known good position within workspace
  // You might need to change these angles based on your robot
  forward(90.0, 90.0, 90.0);  
  current_x = posorangles[0]; target_x = posorangles[0];
  current_y = posorangles[1]; target_y = posorangles[1];
  current_z = posorangles[2]; target_z = posorangles[2];
  
  Serial.println("Initial position:");
  Serial.print("X: "); Serial.println(current_x);
  Serial.print("Y: "); Serial.println(current_y);
  Serial.print("Z: "); Serial.println(current_z);
  
  pinMode(25, OUTPUT);
  testKinematics();
}

void loop() {
  Serial.println("Starting new test cycle...");
  testKinematics();
  delay(2500);
}

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
    Serial.println("IK success!");
  } else {
    posorangles[0] = 1000;
    posorangles[1] = 1000;
    posorangles[2] = 1000;
    Serial.println("oh");
    Serial.println("IK failed - position unreachable");
  }
}

void testKinematics() {
  // Calculate workspace radius based on arm length
  double armLength = 119.2 + 127.5; // Sum of link lengths
  double safeRadius = armLength * 0.7; // 70% of max theoretical reach for safety
  
  // Generate test positions that are more likely to be within workspace
  double testPositions[][3] = {
    {current_x, current_y, current_z - 20},
    {current_x + 10, current_y, current_z - 20},
    {current_x, current_y + 10, current_z - 20},
    {current_x - 10, current_y, current_z - 20},
    {current_x, current_y - 10, current_z - 20}
  };
  
  int numTests = sizeof(testPositions) / sizeof(testPositions[0]);

  Serial.println("Testing Forward and Inverse Kinematics:");
  Serial.print("Current position: (");
  Serial.print(current_x); Serial.print(", ");
  Serial.print(current_y); Serial.print(", ");
  Serial.print(current_z); Serial.println(")");
  
  int successCount = 0;
  
  for (int i = 0; i < numTests; i++) {
    posorangles[0] = 0.0;
    posorangles[1] = 0.0;
    posorangles[2] = 0.0;
    double x = testPositions[i][0];
    double y = testPositions[i][1];
    double z = testPositions[i][2];
    
    Serial.print("Test #"); Serial.print(i+1);
    Serial.print(" - Target (x,y,z): ");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z);

    // Inverse kinematics: position -> angles
    inverse(x, y, z);
    double joint_1 = posorangles[0];
    double joint_2 = posorangles[1];
    double joint_3 = posorangles[2];

    if (joint_1 == 1000 || joint_2 == 1000 || joint_3 == 1000) {
      Serial.println(" -> Confirmed unreachable by IK algorithm");
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

    // // Check if FK result matches original target
    // bool success = (fx == x) && (fy == y) && (fz == z);
                  
    // if (success) {
    //   Serial.println(" -> SUCCESS");
    //   successCount++;
    //   digitalWrite(25, HIGH);
    //   delay(1000);
    //   digitalWrite(25, LOW);
    // } else {
    //   Serial.print(" -> FAILED (Errors: ");
    //   Serial.print(fabs(fx-x), 2); Serial.print(", ");
    //   Serial.print(fabs(fy-y), 2); Serial.print(", ");
    //   Serial.print(fabs(fz-z), 2); Serial.println(")");
      
    //   digitalWrite(25, HIGH);
    //   delay(125);
    //   digitalWrite(25, LOW);
    //   delay(125);      
    //   digitalWrite(25, HIGH);
    //   delay(125);
    //   digitalWrite(25, LOW);
    //   delay(125);
    //   digitalWrite(25, HIGH);
    //   delay(125);
    //   digitalWrite(25, LOW);
    //   delay(125);
    //   digitalWrite(25, HIGH);
    //   delay(250);
    //   digitalWrite(25, LOW);
    // }
  }
  
  Serial.print("Test summary: ");
  Serial.print(successCount);
  Serial.print(" successful out of ");
  Serial.println(numTests);
  Serial.println("---------------------------");
}