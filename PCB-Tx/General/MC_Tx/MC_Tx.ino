/**
  Motorized Commutator Transmitter MCU Code
  Name: MC_Tx
  Purpose: Transmits motor commands wirelessly.

  @author Ibrahim Oladepo
  @version 2.0  22-March-2024

  - This script sends motor commands wirelessly to the receiver(s).
  - This code can switch between manual and automatic control mode.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Stepper.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ANALOG_STICK_PIN_X        A6
// #define ANALOG_STICK_PIN_Y A5
#define ANALOG_STICK_PIN_Z        A5
// #define ANALOG_STICK_PIN_ANGLE    A6
#define BUTTON_ANGLE_LEFT         5
#define BUTTON_ANGLE_RIGHT        18                 

#define MANUAL_LED  22
#define AUTO_LED    21

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

const int togglePin = 22;  // Variable to switch between manual and automatic mode

int equiValue = 512;
int equiOffset = 300;

int fixedAngle = 180;
int fixedXMove = 100;
int fixedYMove = 100;
int fixedZMove = 100;

int motorDelay = 350;   // 350

int analogStickValueX = 0;
int analogStickValueY = 0;
int analogStickValueZ = 0;
int analogStickValueAngle = 0;


void setup() {
  
  Serial.begin(115200);

  // Set serial timeout
  Serial.setTimeout(50);  

  // pinMode(analogStickPin, INPUT);
  pinMode(togglePin, INPUT);

  radio.begin();
  
  // **********************************************
  // CONFIGURE THE NRF24L01

  //set the address
  radio.openReadingPipe(0, address);

  // Set power level
  // radio.setPALevel( RF24_PA_MIN ); // -18dBm
  radio.setPALevel( RF24_PA_LOW ); // -12dBm
  // radio.setPALevel( RF24_PA_HIGH ); // -6dBm

  // changer the transfer rate as needed
  radio.setDataRate( RF24_250KBPS );
  // radio.setDataRate( RF24_1MBPS );
  // radio.setDataRate( RF24_2MBPS );

  // Change the channel (transmit-receive frequency) as needed
  // channel = 0 to 125 correspond to the range 2,400GHz to 2,500GHz
  radio.setChannel( 90 );

  radio.printDetails();
  
  //Set module as transmitter
  radio.stopListening();

  Serial.println( "*****************" );
  Serial.println( "APA MC Tx angle is ready!" );

  // Send initial wireless data
  data.autoMode = digitalRead(togglePin);
  data.xMove = 0;
  data.zMove = 0;
  data.yMove = 0;
  data.angle = 0;
  radio.write(&data, sizeof(Data_Package));
  
}


void loop() {

  if (digitalRead(togglePin) == LOW){
    
    // MANUAL MODE //
    digitalWrite(AUTO_LED, LOW);
    digitalWrite(MANUAL_LED, HIGH);

    // Debug print
    // Serial.println("*** Manual Mode ***");

    data.autoMode = 0;

    // Read analog stick values
    analogStickValueX = analogRead(ANALOG_STICK_PIN_X);
    Serial.print("X value: ");
    Serial.println(analogStickValueX);

    analogStickValueY = analogRead(ANALOG_STICK_PIN_Y);
    Serial.print("Y value: ");
    Serial.println(analogStickValueY);

    analogStickValueZ = analogRead(ANALOG_STICK_PIN_Z);
    Serial.print("Z value: ");
    Serial.println(analogStickValueZ);

    analogStickValueAngle = analogRead(ANALOG_STICK_PIN_ANGLE);
    Serial.print("ANGLE value: ");
    Serial.println(analogStickValueAngle);

    // Send value to receiver
    data.xMove = analogStickValueX;
    data.yMove = analogStickValueY;
    data.zMove = analogStickValueZ;
    data.angle = analogStickValueAngle;

    // Send wireless data
    radio.write(&data, sizeof(Data_Package));    

    delay(motorDelay);

  }
  else{

    // AUTOMATIC MODE
    digitalWrite(AUTO_LED, HIGH);
    digitalWrite(MANUAL_LED, LOW);

    // Debug print
    // Serial.println("*** Automatic Mode ***");

    data.autoMode = 1;
    
    // Check if there is Serial user input
    if (Serial.available() > 0){ 
      
      String received = Serial.readStringUntil('\n');
      data.xMove = received.substring(0, received.indexOf(',')).toInt();
      data.angle = received.substring(received.indexOf(',') + 1).toInt();

      // Set unused axis to zero
      data.xMove = 0;
      data.zMove = 0;
      data.yMove = 0;

      Serial.print(data.xMove);
      Serial.print(", ");
      Serial.println(data.angle);

      // Send angle value to the other NRF24L01 modile
      radio.write(&data, sizeof(Data_Package)); 
      // delay(50);

    }

  }
  
}
