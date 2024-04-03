/**
  Motorized Commutator X-Stage MCU Homing and Python-Comms Code
  Name: MC_Rx_XMove_Homing_v1-0
  Purpose: Performs X stage homing, receives motor commands wirelessly and drives X motor.

  @author Ibrahim Oladepo
  @version 1.0  3-October-2023

  - This script does homing of the x-stage using the limit switches.
  - Some serial integration.
  - Fixing communication with Python script.
  - Python script sending two values.
*/

/*
 * HOMING CODE:
 * LINK: https://arduinogetstarted.com/tutorials/arduino-stepper-motor-and-limit-switch
 */

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ezButton.h>
#include <AccelStepper.h>


#define STEPSIZE    2

// Define the stepper motor connections
#define DIR_PIN     2
#define STEP_PIN    3

#define DIRECTION_CCW -1
#define DIRECTION_CW   1

#define STATE_CHANGE_DIR 1
#define STATE_MOVE       2
#define STATE_MOVING     3

#define MAX_POSITION 0x7FFFFFFF // maximum of position we can set (long type)

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

ezButton limitSwitch_1(6); // create ezButton object that attach to pin A0;
ezButton limitSwitch_2(7); // create ezButton object that attach to pin A1;

// Create an AccelStepper object
AccelStepper stepper(1, STEP_PIN, DIR_PIN);

int steps = 0;
int stepperState = STATE_MOVE;
int direction    = DIRECTION_CW;
long targetPos   = 0;

int gantryLength = 0;
uint8_t segmentCount = 8;
uint8_t segmentNumber = segmentCount / 2;
uint8_t prevSegmentNumber = segmentCount / 2;

uint16_t equiValue = 512;
uint8_t equiOffset = 125;   // DEFAULT: 200
int fixedXMove = 360;


void setup() {

  Serial.begin(115200);
  Serial.setTimeout(5);


  Serial.println("Initializing Radio ...");
  
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

  Serial.println( "*****************" );
  Serial.println( "Receiver is Ready" );


  limitSwitch_1.setDebounceTime(50); // set debounce time to 50 milliseconds
  limitSwitch_2.setDebounceTime(50); // set debounce time to 50 milliseconds

  stepper.setMaxSpeed(1500);   // set the maximum speed  | 500
  stepper.setAcceleration(1500); // set acceleration    | 1000
  stepper.setSpeed(1500);         // set initial speed   | 500
  stepper.setCurrentPosition(0); // set position

  getGantryLength();

}

void loop() {

  if (radio.available()) {   // If the NRF240L01 module received data

    // Read radio buffer
    radio.read(&data, sizeof(Data_Package)); // Read the data and put it into character array
    
    Serial.print("Received X value: ");
    Serial.println(data.xMove);

    if (data.autoMode == 0){

      // MANUAL MODE
      digitalWrite(MANUAL_LED, HIGH);
      digitalWrite(AUTO_LED, LOW);

      if (data.xMove > (equiValue + equiOffset)){

        // Convert angle to steps
        steps = -1 * fixedXMove * ((200 * STEPSIZE) / 360.0);
        Serial.print("Steps is: ");
        Serial.println(steps);
        Serial.println("");

        if (stepper.isRunning()){
          stepper.stop();
        }
        
        stepper.move(steps);
        // myStepper.runToPosition();

      } 
      else if (data.xMove < (equiValue - equiOffset)){

        // Convert angle to steps
        steps = fixedXMove * ((200 * STEPSIZE) / 360.0);
        Serial.print("Steps is: ");
        Serial.println(steps);
        Serial.println("");

        if (stepper.isRunning()){
          stepper.stop();
        }
        
        stepper.move(steps);
        // myStepper.runToPosition();

      }

    } 
    else if (data.autoMode == 1){

      // AUTOMATIC MODE
      digitalWrite(MANUAL_LED, LOW);
      digitalWrite(AUTO_LED, HIGH);

      // Move to segment
      segmentNumber = data.xMove;
      
      if (segmentNumber >= 1 || segmentNumber <= segmentCount){

        if (segmentNumber != prevSegmentNumber){

          if (stepper.isRunning()){
            stepper.stop();
          }

          // Serial.print("Current stepper position: ");
          // Serial.println(stepper.currentPosition());
          
          // stepper.moveTo(targetPos - (1500 * direction));       // move the motor to maximum position again
          int newTargetPosition = (gantryLength * (segmentNumber - 0.5)) / segmentCount;
          // Serial.print("New stepper position: ");
          // Serial.println(newTargetPosition);

          // Set motor to move to new target position
          stepper.moveTo(newTargetPosition);

          prevSegmentNumber = segmentNumber;

          // Serial.println("");
          // digitalWrite(13, LOW);

        }

      }

    }
    
  }

  // While blocks the code
  if (stepper.distanceToGo() != 0){
    stepper.run();
  }

}


void getGantryLength(){

  uint8_t directionCounter = 0;
  // int gantryLength = 0;
  // uint8_t lengthLoop = 1;

  // while ((lengthLoop == 1) && (directionCounter <= 2)){
  while (directionCounter <= 3){

    limitSwitch_1.loop(); // MUST call the loop() function first
    limitSwitch_2.loop(); // MUST call the loop() function first

    if (limitSwitch_1.isPressed()) {
      stepperState = STATE_CHANGE_DIR;
      Serial.println(F("The limit switch 1: TOUCHED"));
    }

    if (limitSwitch_2.isPressed()) {
      stepperState = STATE_CHANGE_DIR;
      Serial.println(F("The limit switch 2: TOUCHED"));
    }

    switch (stepperState) {
      case STATE_CHANGE_DIR:
        direction *= -1; // change direction
        // directionCounter += 1;  // Increment direction counter variable

        Serial.print(F("The direction -> "));
        if (direction == DIRECTION_CW)
          Serial.println(F("CLOCKWISE"));
        else
          Serial.println(F("ANTI-CLOCKWISE"));

        stepperState = STATE_MOVE; // after changing direction, go to the next state to move the motor
        break;

      case STATE_MOVE:
        targetPos = direction * MAX_POSITION;

        // Print current position before resetting
        Serial.print("STEPPER POSITION BEFORE RESET: ");
        Serial.println(stepper.currentPosition());
        Serial.println();

        if (directionCounter > 1){
          gantryLength += abs(stepper.currentPosition());
        }
        directionCounter += 1;  // Increment direction counter variable

        stepper.setCurrentPosition(0); // set position
        stepper.moveTo(targetPos);
        stepperState = STATE_MOVING; // after moving, go to the next state to keep the motor moving infinity
        break;

      case STATE_MOVING: // without this state, the move will stop after reaching maximum position
        if (stepper.distanceToGo() == 0) { // if motor moved to the maximum position
          stepper.setCurrentPosition(0);   // reset position to 0
          stepper.moveTo(targetPos);       // move the motor to maximum position again
        }
        break;
    }

    stepper.run(); // MUST be called in loop() function

  }

  gantryLength = gantryLength / 2;
  Serial.print("Average Gantry Length: ");
  Serial.println(gantryLength);

  // Move the xStage to the center of the gantry
  targetPos = direction * (gantryLength / 2);
  Serial.print("New target position for experiment: ");
  Serial.println(targetPos);

  stepper.setCurrentPosition(0);   // reset position to 0
  stepper.moveTo(targetPos);       // move the motor to maximum position again

  while (stepper.distanceToGo() != 0){
    stepper.run();
  }

  // Reset the current position to a positive value
  if (targetPos < 0){
    stepper.setCurrentPosition(targetPos * -1);
  }
  
}
