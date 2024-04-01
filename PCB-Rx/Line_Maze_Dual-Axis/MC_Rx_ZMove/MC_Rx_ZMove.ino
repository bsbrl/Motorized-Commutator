/**
  Motorized Commutator Z-Translation MCU Code
  Name: MC_Rx_ZMove
  Purpose: Receives motor commands wirelessly and drives motor.

  @author Ibrahim Oladepo
  @version 1.0  4-October-2023

  - This script receives data wireless from the transmitter.
  - The received data are motor commands.
  - Motor is driven using acceleration and decelerations to achieve the target position.
*/

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AccelStepper.h>

#define STEPSIZE  8

#define DIR_PIN     2
#define STEP_PIN    3

#define MANUAL_LED  15
#define AUTO_LED    14

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

struct Data_Package {
  int angle = 0;
  int xMove = 0;
  int yMove = 0;
  int zMove = 0;
  int autoMode = 0;
};

Data_Package data; // Create a variable with the above structure

// #define STEPS 200
#define motorInterfaceType 1

// Define stepper motor pin connections
const int dirPin = 2;
const int stepPin = 3;

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver
// Stepper stepper(STEPS, dirPin, stepPin); // Pin 2 connected to DIRECTION & Pin 3 connected to STEP Pin of Driver
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

int equiValue = 512;
int equiOffset = 125;

int fixedAngle = 100;

// int angle = 0;
int steps = 0;
int stepperSpeed = 2500;           // 200 is default  50
int stepperAcceleration = 4000;    // 50 is default  200


void setup() {
  
  Serial.begin(115200);
  Serial.println("Initializing ...");
  
  radio.begin();

  //set the address
  radio.openReadingPipe(0, address);
  
  // régler puissance d'émission
  // radio.setPALevel( RF24_PA_MIN ); // -18dBm
  // radio.setPALevel( RF24_PA_LOW ); // -12dBm
  radio.setPALevel( RF24_PA_HIGH ); // -6dBm
  // radio.setPALevel( RF24_PA_MAX ); // 0dBm

  // changer the transfer rate as needed
  // radio.setDataRate( RF24_250KBPS ); // pour portée maximum
  // radio.setDataRate( RF24_1MBPS );
  radio.setDataRate( RF24_2MBPS );

  // Change the channel (transmit-receive frequency) as needed
  // channel = 0 to 125 correspond to the range 2,400GHz to 2,500GHz
  // NOTE: if DataRate is set to 2MBPS, the bandwidth
  // is 2 MHz and we must therefore skip every other channel 
  // to avoid overlaps
  radio.setChannel( 90 );

  radio.printDetails();

  //Set module as receiver
  radio.startListening();

  // set the maximum speed, acceleration factor,
	// initial speed and the target position
	myStepper.setMaxSpeed(stepperSpeed);
	myStepper.setAcceleration(stepperAcceleration);
	myStepper.setSpeed(stepperSpeed);
	// myStepper.moveTo(200);

  Serial.println( "*****************" );
  Serial.println( "Receiver is Ready" );
  
}


void loop() {

  if (radio.available()) {   // If the NRF240L01 module received data

    // Read radio buffer
    radio.read(&data, sizeof(Data_Package)); // Read the data and put it into character array
    
    Serial.print("Received angle value: ");
    Serial.println(data.zMove);

    if (data.autoMode == 0){
      // MANUAL MODE
      digitalWrite(MANUAL_LED, HIGH);
      digitalWrite(AUTO_LED, LOW);
      
      if (data.zMove > (equiValue + equiOffset)){

        // Debug print
        // Serial.println("Upward movement");

        // Convert angle to steps
        steps = fixedAngle * ((200*STEPSIZE) / 360.0);
        Serial.print("Steps is: ");
        Serial.println(steps);
        Serial.println("");

        if (myStepper.isRunning()){
          myStepper.stop();
        }
        
        myStepper.move(steps);
        // myStepper.runToPosition();

      } else if (data.zMove < (equiValue - equiOffset)){

        // Debug print
        // Serial.println("Downward movement");

        // Convert angle to steps
        steps = -1 * fixedAngle * ((200*STEPSIZE) / 360.0);
        Serial.print("Steps is: ");
        Serial.println(steps);
        Serial.println("");

        if (myStepper.isRunning()){
          myStepper.stop();
        }
        
        myStepper.move(steps);
        // myStepper.runToPosition();

      }

    }
    else if (data.autoMode == 1){
      
      // AUTOMATIC MODE
      digitalWrite(MANUAL_LED, LOW);
      digitalWrite(AUTO_LED, HIGH);

      steps = data.zMove;
      Serial.print("Steps is: ");
      Serial.println(steps);
      Serial.println("");

      // Check if stepper is running before sending new command
      // Stop the stepper motor if it is running
      if (myStepper.isRunning()){
        myStepper.stop();
      }

      // Set a new target position
      myStepper.move(steps);

    }
    
  }

  if (myStepper.distanceToGo() != 0){
    myStepper.run();
  }
  
}
