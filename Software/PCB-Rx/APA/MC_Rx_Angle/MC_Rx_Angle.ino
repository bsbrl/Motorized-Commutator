/**
  Motorized Commutator Angular-Rotation MCU Code
  Name: MC_Rx_Angle
  Purpose: Receives motor commands wirelessly and drives motor.

  @author Ibrahim Oladepo
  @version 2.0  10-February-2025

  - This script receives data wireless from the transmitter.
  - The received data are motor commands.
  - Motor is driven using acceleration and decelerations to achieve the target position.
  - This code has been updated to work for just TMC2209 stepper motor driver
  - Compatible with V2 PCB with silent motor driver.
*/

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AccelStepper.h>
#include <TMCStepper.h>


#define STEPSIZE        64     // 64 | 256
#define STEPS           200
#define SCALER          48
#define STEPS_PER_MM    80
#define REVOLUTION      (40 * (STEPSIZE / SCALER) * 3 * STEPS_PER_MM)

#define motorInterfaceType 1

#define DIR_PIN     16
#define STEP_PIN    17

#define MANUAL_LED  15
#define AUTO_LED    14

#define SERIAL_PORT Serial5 // HardwareSerial port | Default: Serial1 
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f

RF24 radio(9, 10); // CE, CSN

const byte address[6] = "00001";

struct Data_Package {
  int angle = 0;
  int xMove = 0;
  int yMove = 0;
  int zMove = 0;
  int autoMode = 0;
};

Data_Package data; //Create a variable with the above structure

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver
// Stepper stepper(STEPS, dirPin, stepPin); // Pin 2 connected to DIRECTION & Pin 3 connected to STEP Pin of Driver
AccelStepper myStepper(motorInterfaceType, STEP_PIN, DIR_PIN);

// Initialize the serial communication for TMC2209
// TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

int equiValue = 512;
int equiOffset = 350;

// The angleMapper variable is used to calibrate the stepper rotation to give
// 1 revolution rotation on the cable holder pulley.
float angleMapper = 3;

int fixedAngle = 1710;            // 1710 for 256 microsteps

// int angle = 0;
int steps = 0;
int stepperSpeed = 5000;           // 2500 is default  50
int stepperAcceleration = 5000;     // 4000 is default  200
// int vActual = MAX_SPEED / SPEED_DIVIDER;           // Default is 102400/2 for 256 microsteps

// Variable for changing motor direction via UART
bool shaft = false; 


void setup() {

  Serial.begin(115200);
  Serial.println("Initializing ...");

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  // **********************************************
  // CONFIGURE THE NRF24L01

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  // **********************************************
  // CONFIGURE THE TMC2209

  SERIAL_PORT.begin(115200);      // INITIALIZE UART TMC2209

  driver.begin();                // Initialize driver                        
  driver.toff(5);                // Enables driver in software
  driver.rms_current(600);       // Set motor RMS current | 1400 is max | 600 default
  driver.microsteps(STEPSIZE);           // Set microsteps to 256 (preferred) or 64 or 32
  driver.pwm_autoscale(true);   // Needed for stealthChop
  driver.en_spreadCycle(true);   // Toggle spreadCycle for smooth & silent operation
  // driver.VACTUAL(0); //SET initial SPEED OF MOTOR to zero

  // Set the maximum speed, acceleration factor,
	// initial speed and the target position
	myStepper.setMaxSpeed(24000);
	myStepper.setAcceleration(24000);
	myStepper.setSpeed(24000);
  myStepper.stop();

  myStepper.move(0);
  myStepper.stop();

  Serial.println( "*****************" );
  Serial.println( "Stepper Configured!" );

  // **********************************************

  Serial.println( "*****************" );
  Serial.println( "APA MC Rx angle is ready!" );

}

void loop() {

  if (radio.available()) {   // If the NRF240L01 module received data

    // Read radio buffer
    radio.read(&data, sizeof(Data_Package)); // Read the data and put it into character array
    
    Serial.print("Received angle value: ");
    Serial.println(data.angle);

    if (data.autoMode == 0){
      // MANUAL MODE
      digitalWrite(MANUAL_LED, HIGH);
      digitalWrite(AUTO_LED, LOW);
      
      if (data.angle > (equiValue + equiOffset)){

        // Debug print
        // Serial.println("Upward movement");

        // Change motor direction via UART
        shaft = true;
        driver.shaft(shaft);

        // Convert angle to steps
        steps = fixedAngle;
        Serial.print("Steps is: ");
        Serial.println(steps);
        Serial.println("");

        if (myStepper.isRunning()){
          myStepper.stop();
        }
        
        // myStepper.move(steps);
        myStepper.move(REVOLUTION);
        // myStepper.runToPosition();

      } else if (data.angle < (equiValue - equiOffset)){

        // Debug print
        // Serial.println("Downward movement");

        // Change motor direction via UART
        shaft = false;
        driver.shaft(shaft);
        
        // Convert angle to steps
        steps = fixedAngle;
        Serial.print("Steps is: ");
        Serial.println(steps);
        Serial.println("");

        if (myStepper.isRunning()){
          myStepper.stop();
        }
        
        myStepper.move(REVOLUTION);
        // myStepper.runToPosition();

      }

    }
     else if (data.autoMode == 1){
      
      // AUTOMATIC MODE
      digitalWrite(MANUAL_LED, LOW);
      digitalWrite(AUTO_LED, HIGH);

      // steps = data.angle;
      // Convert angle to steps
      // (40 * (STEPSIZE / SCALER) * 3 * STEPS_PER_MM) = 38400
      steps = (data.angle * 38400 * 4 ) / 360.0;  // *4 added for 256 microsteps
      // Serial.print("Steps is: ");
      // Serial.println(steps);
      // Serial.println("");

      // Add to the distanceToGo() steps
      // distanceToGo(): the distance from the current position to the target position in steps
      // Positive is clockwise from the current position
      steps += myStepper.distanceToGo();

      // Set a new target position
      myStepper.move(steps);
      myStepper.runToPosition();

    }
  }

  // Serial.print("angle: ");
  // Serial.print(data.angle);
  // Serial.print(" xMove: ");
  // Serial.print(data.xMove);
  // Serial.print(" yMove: ");
  // Serial.print(data.yMove);                          
  // Serial.print(" zMove: ");
  // Serial.print(data.zMove);
  // Serial.print(" autoMode: ");
  // Serial.println(data.autoMode);

  if (myStepper.distanceToGo() != 0){
    // Serial.print("Distance to go: ");
    // Serial.println(myStepper.distanceToGo());
    myStepper.run();
  }
  // myStepper.run();
  // delay(250);

}