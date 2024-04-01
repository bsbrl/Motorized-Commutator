/*
  Arduino Limit Switches and LED Toggle

  Description:
  This Arduino sketch demonstrates how to use two limit switches connected to pins 6 and 7
  to toggle the built-in LED (on pin 13) every time either of the switches is pressed.
  The limit switches are configured with internal pull-up resistors to simplify the wiring.

  Hardware Connections:
  - Connect one terminal of each limit switch to ground (GND).
  - Connect the other terminal of the first limit switch to digital pin 6.
  - Connect the other terminal of the second limit switch to digital pin 7.
  - Connect the built-in LED (on pin 13) to ground (GND).

  Operation:
  When either of the limit switches is pressed, the corresponding interrupt service routine
  (ISR) is triggered, setting a flag. In the main loop, we continuously check for these flags.
  If either flag is set, the built-in LED is toggled, providing visual feedback. A debounce
  delay is included to prevent multiple toggles from a single button press due to mechanical
  bouncing.

  Author:
  Ibrahim Oladepo
  
  Date:
  4 September 2023

  Version:
  1.0

  License:
  This code is released under the [License Type] license.
  [Include any licensing information or disclaimers here]

*/

const int switchPin1 = 6;  // Pin for the first limit switch
const int switchPin2 = 7;  // Pin for the second limit switch
const int ledPin = 13;     // Built-in LED pin (pin 13 on most Arduino boards)

volatile bool switch1Pressed = false;
volatile bool switch2Pressed = false;

void setup() {
  pinMode(switchPin1, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(switchPin2, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(switchPin1), switch1Interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(switchPin2), switch2Interrupt, FALLING);
}

void loop() {
  if (switch1Pressed || switch2Pressed) {
    digitalWrite(ledPin, !digitalRead(ledPin));  // Toggle the LED
    switch1Pressed = false;
    switch2Pressed = false;
    delay(500);  // Debounce delay
  }
}

void switch1Interrupt() {
  switch1Pressed = true;
}

void switch2Interrupt() {
  switch2Pressed = true;
}
