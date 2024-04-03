/**
  Motorized Commutator X-Translation MCU Code
  Name: MC_AccelStepper_Test
  Purpose: Test the non-blocking mode of Accelstepper library through serial.

  @author Ibrahim Oladepo
  @version 1.0  30-August-2023

  - This script receives serial data from the serial monitor.
  - The received data are motor commands.
  - Motor is driven using acceleration and decelerations to achieve the target position.
*/

#include <Stepper.h> 
#include <AccelStepper.h>

// #define STEPSIZE 2
#define motorInterfaceType 1

// Define pin connections
const int dirPin = 2;
const int stepPin = 3;

// Sweet spot: speed=2500, accel=4000
int steps = 0;
int motorSpeed = 1500;   
int speedmin = 0;
int speedmax = 4000;
int stepperAcceleration = 3000;    // 50 is default  1000

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver
// Stepper stepper(STEPS, dirPin, stepPin); // Pin 2 connected to DIRECTION & Pin 3 connected to STEP Pin of Driver
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);


void setup() {

  // Initialize and setup serial comms
  Serial.begin(115200);

  // Set serial timeout
  Serial.setTimeout(50);  

  // Set the maximum speed, acceleration factor,
	// initial speed and the target position
	myStepper.setMaxSpeed(motorSpeed);
	myStepper.setAcceleration(stepperAcceleration);
	myStepper.setSpeed(motorSpeed);

}

void loop() {

  if (Serial.available() > 0){
    // Read an integer from the serial buffer
    // steps = Serial.parseInt();
    String received = Serial.readStringUntil('\n');
    steps = received.toInt();  
    Serial.print("Steps is: ");
    Serial.println(steps);
    Serial.println("");

    // Check if stepper is running before sending new command
    // Stop the stepper motor if it is running
    if (myStepper.isRunning()){
      myStepper.stop();
    }

    // delay(50);

    // Set a new target position
    myStepper.move(steps);
  }

  if (myStepper.distanceToGo() != 0){
    myStepper.run();
  }

}