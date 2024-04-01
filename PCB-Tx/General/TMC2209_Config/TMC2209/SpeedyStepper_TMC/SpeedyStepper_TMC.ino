#include <SpeedyStepper.h> //Simple & good stepper library, get it.

#include <TMCStepper.h>

#define DIR_PIN          2 // Direction
#define STEP_PIN         3 // Step
#define SERIAL_PORT Serial5 // HardwareSerial port | Default: Serial1 
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

bool shaft = false;  // ONLY NEEDED FOR CHANGING DIRECTION VIA UART, NO NEED FOR DIR PIN FOR THIS

TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);
SpeedyStepper stepper;
int accel = 0;

void setup() {

  stepper.connectToPins(STEP_PIN, DIR_PIN); // INITIALIZE SpeedyStepper
 
  SERIAL_PORT.begin(115200);      // INITIALIZE UART TMC2209
  Serial.begin(115200);
  delay(500);
  
  Serial.println(F("Serial Initialized"));

  driver.begin();                // Initialize driver                        
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(600);       // Set motor RMS current
  
  Serial.print("current set to:");
  Serial.println(driver.rms_current());
 
  driver.microsteps(256);           // Set microsteps to 1/2

  driver.pwm_autoscale(true);   // Needed for stealthChop
  driver.en_spreadCycle(true);   // Toggle spreadCycle for smooth & silent operation

  stepper.setCurrentPositionInSteps(0);                   // Set zero position
  stepper.setSpeedInStepsPerSecond(400);              //Set Speed
  stepper.setAccelerationInStepsPerSecondPerSecond(400);   //Set acceleration, smaller value for super smooth direction changing

}

void loop() {

  uint16_t msread = driver.microsteps();
  Serial.print(F("Read microsteps via UART to test UART receive : "));    
  Serial.println(msread); 

  //Serial.println(F("Move 6400 steps forward at 600ma"));
  driver.rms_current(600); 
  //stepper.moveToPositionInSteps(6400);

  //Serial.println(F("Wait 3sec and turn current low so you can turn the motor shaft"));
  //driver.rms_current(10); 
  //delay(3000);

  //Serial.println(F("Move back to 0 position at 300ma"));
  //driver.rms_current(300); 
  //stepper.moveToPositionInSteps(0);

  //MOVE MOTOR VIA UART AND CHANGE DIRECTION VIA SOFTWARE, IT RUNS AS LONG AS YOU LET IT... PROBABLY ONLY USEFUL WITH ENCODER. THE VALUE SETS ONLY THE SPEED.
  Serial.println("move under board control");
  //driver.TPWMTHRS(47);//transition threshold from one pwm mode to the other one, stealthchop to spreadcyccle I think, in clock periods per 1/256 microstep, the internal clock is 12 mhz 

  accel=500;

  Serial.print("accel:");
  Serial.println(accel);

  long int spd = 50800;    // Default: 102400
  Serial.print("spd: ");
  Serial.println(spd);
  
  for (long int i = 0; i <=spd; i = i + accel){
    
    driver.VACTUAL(i); //SET SPEED OF MOTOR
    Serial.print("vactual set to: ");
    Serial.println(driver.VACTUAL());
    delay(100);

  }


  for (long int i = spd; i >=0; i = i - accel){
  
    driver.VACTUAL(i); //SET SPEED OF MOTOR
    Serial.print("deccelerating, vactual set to: ");
    Serial.println(driver.VACTUAL());
    delay(100);

  }

  //Serial.println("try to move stepper using the speedystepper library method step dir pins");
  //stepper.moveToPositionInSteps(6400);
  //delay(7000); // MOTOR MOVES 2 SEC THEN STOPS
  //Serial.println("try again to move stepper using the speedystepper library method step dir pins");
  //stepper.moveToPositionInSteps(0);

  shaft = !shaft; // REVERSE DIRECTION
  driver.shaft(shaft); // SET DIRECTION

}