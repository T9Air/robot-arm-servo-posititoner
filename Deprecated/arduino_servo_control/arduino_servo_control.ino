// Adjust micros to what your servo motors can handle

#include <Servo.h>

Servo base;
Servo arm1;

// Variables to store servo positions
int basemove = 1500;  // Default position
int arm1move = 1500;  // Default position

void setup() {
  base.attach(3);
  arm1.attach(9);

  Serial.begin(9600);
  
  // Set initial positions
  base.writeMicroseconds(1500);
  arm1.writeMicroseconds(1500);
}

void loop() {
  while (Serial.available() == 0){}
  
  // Read the incoming string until newline
  String inputString = Serial.readStringUntil('\n');
    
  // Find the space character that separates the two values
  int spaceIndex = inputString.indexOf(' ');
    
  // Check if the space exists
  if (spaceIndex != -1) {
    // Extract the two substrings
    String baseString = inputString.substring(0, spaceIndex);
    String arm1String = inputString.substring(spaceIndex + 1);
    
    // Convert to integers
    int newBaseMove = baseString.toInt();
    int newArm1Move = arm1String.toInt();
     
    // Validate ranges
    if (newBaseMove >= 950 && newBaseMove <= 2150 && 
        newArm1Move >= 650 && newArm1Move <= 2350) {
      
      int oldbasemove = basemove;
      int oldarm1move = arm1move;
      basemove = newBaseMove;
      arm1move = map(newArm1Move, 650, 2350, 2350, 650);
      
      // Move servos to new positions
      if ((oldbasemove < 1500 && arm1move < oldarm1move) || (oldbasemove > 1500 && arm1move > oldarm1move)){
        base.writeMicroseconds(basemove);
        arm1.writeMicroseconds(arm1move);
      } else if ((oldbasemove < 1500 && arm1move > oldarm1move) || (oldbasemove > 1500 && arm1move < oldarm1move)){
        arm1.writeMicroseconds(arm1move);
        base.writeMicroseconds(basemove);
      } else {
        arm1.writeMicroseconds(arm1move);
        base.writeMicroseconds(basemove);
      }
      // Feedback to user
      Serial.print("Moving to: Base=");
      Serial.print(basemove);
      Serial.print(", Arm1=");
      Serial.println(arm1move);
    } else {
      Serial.println("Error: Base: 950-2150, Arm1: 560-2350");
    }
  } else {
    Serial.println("Error: Invalid format. Use: <base_micros> <arm1_micros>");
  }
}