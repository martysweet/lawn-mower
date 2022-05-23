#include <Wire.h>
#include "TLE9879_Group.h"
TLE9879_Group *shields;

#define MAX_MOTOR_RPM 3500
#define COUNTS_PER_REV 6
#define ENC_A_PIN 3

bool doUpdate = false;
int desiredPercSpeed = 0;   // 0-100 of desired speed, maps to 0-MAX_MOTOR_RPM
int lastUpdateTime = 0;     // Safety watchdog from I2C write

volatile double motorCount = 0;
double prevMotorCount = 0;
double prevVelTime = 0;

int currentRPM = 0;



// I2C Functions
// Read Registers
// 0x00 | currentMotorSpeed (RPM - TODO: What scaling factor?)
// 0x01 | tiltSensorsError | 0=OK 1=ERR
// 0x02 | motorStallCount | int, count of how many times the controller has detected and restarted the motor

// Write 
// Single byte: Motor speed 0-100%

void setup() {
  Serial.begin(9600);
  // Start the I2C Bus
  Wire.begin(10); // AddressL 0x10
//  Wire.onRequest(onRequestOdometry);
  Wire.onReceive(onReceiveMotorSpeed);

  // Listen to encoder feedback, we don't care about direction so we only use a single pin
  // 6 counts = 1 revolution on the BLDC motor
  pinMode(ENC_A_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), readEncoder, RISING);

  // Setup the motor shield
  delay(500);
  shields = new TLE9879_Group(1);
  shields->setMode(FOC, BOARD1);
  shields->setMotorSpeed(0, BOARD1);
  shields->setMotorMode(STOP_MOTOR, BOARD1);
  
  shields->setMotorSpeed(3500, BOARD1);
  shields->setMotorMode(START_MOTOR, BOARD1);
}

void loop() {

  // Check the watchdog timer
  // If last update was more than 1.5 seconds ago, stop the motor as we may have lost
  // communication with the host
  if(millis() - lastUpdateTime >= 1500){
    //Serial.println("Communication timeout, forcing 0 speed");
    //desiredPercSpeed = 0;
    //doUpdate = true;
  }

  // Do an update over SPI if required
  if(doUpdate){
    // Set the speed
//    shields->setMotorSpeed(map(desiredPercSpeed, 0, 100, 0, MAX_MOTOR_RPM), BOARD1);

    // Ensure the motor is stopped or started
    if(desiredPercSpeed > 0){
      shields->setMotorMode(START_MOTOR, BOARD1);
    }else{
      shields->setMotorMode(STOP_MOTOR, BOARD1);
    }

    // Update done
    doUpdate = false;
  }

  // Calculate the current motor velocity
  long currentTime = micros();
  float delta = ((float) (currentTime-prevVelTime))/1.0e6;
  
  noInterrupts();
  long countI = motorCount;
  interrupts();

  currentRPM = ((countI - prevMotorCount)/delta)/COUNTS_PER_REV*60.0;    // Convert count/s to RPM

  Serial.println((String) (countI - prevMotorCount) + "," + (String) delta + "," + (String) currentRPM);


  // Save values for next cycle
  prevVelTime = currentTime;
  prevMotorCount = countI;


  // Delay for some delta for velocity measurements
  delay(100);
}

void onReceiveMotorSpeed(int byteCount){

    if(byteCount != 1){
      Serial.println("ERROR: I2C Invalid Payload. Expected 1 bytes.");
      while(Wire.available()){ 
        Wire.read(); // Clear the buffer
      }
      return;
    }

    // Write the new motor speeds
    uint8_t perc = (uint8_t) Wire.read();

    if(perc != desiredPercSpeed){
      desiredPercSpeed = perc;
      doUpdate = true;
    }

    // Give time of last update for watchdog/failsafe behaviour
    lastUpdateTime = millis();
}

void readEncoder(){
  motorCount++;
}
