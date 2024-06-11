#include <Arduino.h>
#include <AccelStepper.h>


#define STEPSIZE  8

// #define STEPS 200
#define motorInterfaceType 1

// Define stepper motor pin connections
const int dirPin = 2;
const int stepPin = 3;

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver
// Stepper stepper(STEPS, dirPin, stepPin); // Pin 2 connected to DIRECTION & Pin 3 connected to STEP Pin of Driver
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

// int angle = 0;
int steps = 0;
int stepperSpeed = 2000;           // 200 is default  50
int stepperAcceleration = 2000;    // 50 is default  200

// Convert angle to steps
int fixedAngle = 720 * 3;

// Rotation direction variable
String command = "CW";


void setup() {
    
  Serial.begin(115200);  // Start serial communication at 115200 baud rate

  steps = fixedAngle * ((200 * STEPSIZE) / 360.0);
    
  // set the maximum speed, acceleration factor,
	// initial speed and the target position
	myStepper.setMaxSpeed(stepperSpeed);
	myStepper.setAcceleration(stepperAcceleration);
	myStepper.setSpeed(stepperSpeed);
	// myStepper.moveTo(200);

}


void loop() {

  if (command == "CW") {
    // Rotate clockwise
    myStepper.move(2 * steps);
  } 
  else if (command == "CCW") {
    // Rotate counterclockwise
    myStepper.move(-2 * steps);
  }

  // Execute the movement
  while (myStepper.distanceToGo() != 0) {
      myStepper.run();
  }

  if (command == "CW") {
    // Change direction
    command = "CCW";
  } 
  else if (command == "CCW") {
    // Change direction
    command = "CW";
  }


}
