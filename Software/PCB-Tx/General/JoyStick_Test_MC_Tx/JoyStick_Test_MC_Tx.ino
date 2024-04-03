int joyVal = 0;

void setup() {
  
  // Initialize serial
  Serial.begin(115200);
  Serial.println("Initializing ...");

  // Configure pins
  pinMode(4, INPUT);

}

void loop() {
  
  // Read the digital pin
  joyVal = digitalRead(4);

  // Print value
  Serial.println(joyVal);

  // Add some delay
  delay(1000);

}
