/*
  Arduino SPDT Limit Switch with Interrupt

  Description:
  This Arduino sketch demonstrates how to use an SPDT (Single Pole Double Throw) limit
  switch connected to digital pin 7 to toggle the built-in LED (on pin 13) when a button
  press event occurs. The limit switch is configured with an internal pull-up resistor
  to simplify the wiring.

  Hardware Connections:
  - Connect one terminal of the SPDT limit switch to ground (GND).
  - Connect the other terminal of the limit switch to digital pin 7.
  - Connect the built-in LED (on pin 13) to ground (GND).

  Operation:
  When the limit switch is pressed, an interrupt service routine (ISR) is triggered, and
  the built-in LED is toggled. The ISR is configured to trigger on the FALLING edge, so
  the LED toggles when the button is pressed, not when it's released.

  Author:
  [Your Name]
  
  Date:
  [Date]

  Version:
  1.0

  License:
  This code is released under the [License Type] license.
  [Include any licensing information or disclaimers here]

*/

// Include necessary libraries and define constants and variables
#include <Arduino.h>

const int limitSwitchPin = 7;  // Pin for the SPDT limit switch
const int ledPin = 13;         // Built-in LED pin (pin 13 on most Arduino boards)

// Initialize setup() function
void setup() {
  Serial.begin(115200);
  
  // Configure pins
  pinMode(limitSwitchPin, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(ledPin, OUTPUT);

  // Attach the interrupt handler for the limit switch
  attachInterrupt(digitalPinToInterrupt(limitSwitchPin), limitSwitchInterrupt, RISING);
}

// Initialize loop() function
void loop() {
  // Your main code goes here (if needed)
  // The LED toggling is handled by the interrupt handler
}

// Interrupt service routine for the limit switch
void limitSwitchInterrupt() {
  // Toggle the LED
  digitalWrite(ledPin, !digitalRead(ledPin));
  Serial.println("TOGGLED ...");
  // delay(350);
}
