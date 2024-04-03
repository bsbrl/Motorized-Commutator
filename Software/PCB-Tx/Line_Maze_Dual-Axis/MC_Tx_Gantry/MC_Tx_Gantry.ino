#include <Arduino.h>
#include <SPI.h>
#include <Stepper.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ANALOG_STICK_PIN_X A3
#define ANALOG_STICK_PIN_Y A4
#define ANALOG_STICK_PIN_Z A6
#define ANALOG_STICK_PIN_ANGLE A5

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
// const int analogStickPin = 32;

const int autoLED = 14;
const int manualLED = 15;
// const int LEDDelay = 25;

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

// const byte numChars = 32;
// char receivedChars[numChars];

// char *separatedStrings[2]; 
// char *ptr = NULL;

// boolean newData = false;
// int dataNumber = 0;             // new for this version

// int auto_rotation = 0;
// int auto_translation = 0;


void setup() {
  
  Serial.begin(115200);

  // Set serial timeout
  Serial.setTimeout(50);  

  // pinMode(analogStickPin, INPUT);
  pinMode(togglePin, INPUT);

  // pinMode(autoLED, OUTPUT);    // AUTO MODE LED
  // pinMode(manualLED, OUTPUT);    // MANUAL MODE LED

  radio.begin();
  
  //set the address
  radio.openWritingPipe(address);

  // Set transmission power
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
  
  //Set module as transmitter
  radio.stopListening();

  Serial.println( "*****************" );
  Serial.println( "Trasmitter is Ready" );

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

    // Debug print
    // Serial.println("*** Manual Mode ***");

    digitalWrite(autoLED, LOW);
    digitalWrite(manualLED, HIGH);

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

    // AUTOMATIC MODE //

    // Debug print
    // Serial.println("*** Automatic Mode ***");

    data.autoMode = 1;

    digitalWrite(autoLED, HIGH);
    digitalWrite(manualLED, LOW);
    
    if (Serial.available() > 0){ // If there is Serial user input
      
      String received = Serial.readStringUntil('\n');
      data.xMove = received.substring(0, received.indexOf(',')).toInt();
      data.angle = received.substring(received.indexOf(',') + 1).toInt();

      // Set UNUSED AXIS TO ZERO
      data.zMove = 0;
      data.yMove = 0;

      Serial.print(data.xMove);
      Serial.print(", ");
      Serial.println(data.angle);

      // Send angle value
      radio.write(&data, sizeof(Data_Package)); // Send the array data (X value) to the other NRF24L01 modile
      // delay(50);

    }

  }
  
}
