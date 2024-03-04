/**
  Motorized Commutator Angular-Rotation MCU Code
  Name: MC_Rx_Angle
  Purpose: Receives motor commands wirelessly and drives motor.

  @author Ibrahim Oladepo
  @version 1.0  28-February-2024

  - This script receives data wireless from the transmitter.
  - The received data are motor commands.
  - Motor is driven using acceleration and decelerations to achieve the target position.
  - This code has been updated to work for just TMC2209 stepper motor driver
*/

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

#define STEPSIZE  4

#define DIR_PIN     2
#define STEP_PIN    3

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

Data_Package data; // Create a variable with the above structure

// #define STEPS 200
#define motorInterfaceType 1

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
int stepperSpeed = 1000;           // 2500 is default  50
int stepperAcceleration = 2500;     // 4000 is default  200
int vActual = 102400 / 6;           // Default is 102400/2 for 256 microsteps

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

  //set the address
  radio.openReadingPipe(0, address);

  // Set power level
  // radio.setPALevel( RF24_PA_MIN ); // -18dBm
  // radio.setPALevel( RF24_PA_LOW ); // -12dBm
  radio.setPALevel( RF24_PA_HIGH ); // -6dBm

  // changer the transfer rate as needed
  // radio.setDataRate( RF24_250KBPS );
  // radio.setDataRate( RF24_1MBPS );
  radio.setDataRate( RF24_2MBPS );

  // Change the channel (transmit-receive frequency) as needed
  // channel = 0 to 125 correspond to the range 2,400GHz to 2,500GHz
  radio.setChannel( 90 );

  radio.printDetails();

  //Set module as receiver
  radio.startListening();

  // **********************************************
  // CONFIGURE THE TMC2209

  SERIAL_PORT.begin(115200);      // INITIALIZE UART TMC2209

  driver.begin();                // Initialize driver                        
  driver.toff(5);                // Enables driver in software
  driver.rms_current(1400);       // Set motor RMS current | 1400 is max | 600 default
  driver.microsteps(64);           // Set microsteps to 256 (preferred) or 64 or 32
  driver.pwm_autoscale(true);   // Needed for stealthChop
  driver.en_spreadCycle(true);   // Toggle spreadCycle for smooth & silent operation
  driver.VACTUAL(0); //SET initial SPEED OF MOTOR to zero

  // Set the maximum speed, acceleration factor,
	// initial speed and the target position
	myStepper.setMaxSpeed(stepperSpeed);
	myStepper.setAcceleration(stepperAcceleration);
	myStepper.setSpeed(stepperSpeed);
  myStepper.stop();

  if (myStepper.isRunning()){
    driver.VACTUAL(0);
    myStepper.stop();
  }

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
        
        myStepper.move(steps);
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
        
        myStepper.move(steps);
        // myStepper.runToPosition();

      }

    }
    else if (data.autoMode == 1){
      
      // AUTOMATIC MODE
      digitalWrite(MANUAL_LED, LOW);
      digitalWrite(AUTO_LED, HIGH);

      // steps = data.angle;
      // Convert angle to steps
      steps = data.angle * angleMapper * ((200*STEPSIZE) / 360.0);
      // Serial.print("Steps is: ");
      // Serial.println(steps);
      // Serial.println("");

      // Set a new target position
      myStepper.move(steps);
      myStepper.runToPosition();

    }
    
  }

  if (myStepper.distanceToGo() != 0){
    driver.VACTUAL(vActual);
    myStepper.run();
  }
  else{
    driver.VACTUAL(0);
  }
  
}
