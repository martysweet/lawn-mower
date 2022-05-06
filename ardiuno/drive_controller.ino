#include <Wire.h>

// Enable console logging
// Use the Ardiuno Graphing feature (Tools > Serial Plotter)
#define GRAPH true

// Use sequential pins (expecting 3 pins - H1,H2,PWM)
// H1, H2 are the pins controlling the H-Bridge Driver (sometimes labelled IN1,IN2,IN3,IN4 on LN298 modules)
// Swap these pins around, or the motor connection if the direction is incorrect
#define LEFT 3    // 3, 4, 5
#define RIGHT 6   // 6, 7, 8
int desiredLeft, desiredRight = 0;

// Feedback HALL sensors (SS49E)
// Only increases of voltage are read, based on SENS_THRESHOLD
// ENSURE the magnets are the correct orientation (otherwise the readings will go down from 1v)
#define LEFT_HALL A0
#define RIGHT_HALL A1
#define THRESHOLD 600  // Voltage reading, values above this define a magnet passing the Hall Sensor

// Variable declaration for Odometry
int lVolt, rVolt;           // Voltage reading from Analog Input
bool lHigh, rHigh = false;  // Flag if value is over THRESHOLD
uint8_t lCnt, rCnt = 0;         // Incremental counter when value returns under THRESHOLD
unsigned long lastUpdateTime = 0;

// I2C Specification
// Operations
//
// - Write Motor Direction/Speed (onReceive)
//    - Perform a 3-byte write to the device
//    - Byte 1, LSB LEFT direction, LSB+1 RIGHT direction, where 0=forward,1=reverse
//    - Byte 2, Byte 3, LEFT and RIGHT speed 0-255
//
// - Read odometry count (onRequest)
//    - Returns 1-byte for each counter (2-byte response)
//    - [Left, Right]
//    - Note - this should be read regularly (multiple times a second) to prevent overflow
bool doMotorUpdate = false;

// Function declartion to allow calls before function definitions
void setMotorSpeed(int, int);
void onRequestOdometry();
void onReceiveMotorSpeed(int);
void setup();
void loop();

// Setup the Pins
void setup() {

  // Left Motor
  for(int i=LEFT; i<LEFT+3; i++){
    pinMode(i, OUTPUT);
  }

  // Right Motor
  for(int i=RIGHT; i<RIGHT+3; i++){
    pinMode(i, OUTPUT);
  }

  // Initialise the motors
  setMotorSpeed(LEFT, 0);
  setMotorSpeed(RIGHT, 0);

  // Hall Sensors
  pinMode(LEFT_HALL, INPUT);
  pinMode(RIGHT_HALL, INPUT);

  // Start the I2C Bus
  Wire.begin(9); // AddressL 0x9, 0001001
  Wire.onRequest(onRequestOdometry);
  Wire.onReceive(onReceiveMotorSpeed);

  if(GRAPH) {
    Serial.begin(115200);
  }
}

// Main Loop, keep track of analog Hall Sensor Readings
void loop() {

  // If last update was more than 2 seconds ago, stop the motors as we may have lost
  // communication with the host
  if(millis() - lastUpdateTime >= 2000){
    desiredLeft = desiredRight = 0;
    doMotorUpdate = true;
  }
  

  // Update motor speeds if requested (via I2C) or safety timer
  if(doMotorUpdate){
    setMotorSpeed(LEFT, desiredLeft);
    setMotorSpeed(RIGHT, desiredRight);
    doMotorUpdate = false;
  }

  // Left Odometry
  lVolt = analogRead(A0);
  if(lVolt >= THRESHOLD && !lHigh){
    lHigh = true;
  }else if(lVolt < THRESHOLD - 15 && lHigh){  // THRESHOLD - 15 - Prevent counter being incremented due to bouncing
    lHigh = false;
    lCnt += 1; 
  }

  // Right Odometry
  rVolt = analogRead(A1);
  if(rVolt >= THRESHOLD && !rHigh){
    rHigh = true;
  }else if(rVolt < THRESHOLD - 15 && rHigh){ // THRESHOLD - 15 - Prevent counter being incremented due to bouncing
    rHigh = false;
    rCnt += 1; 
  }

  // Output for Graphing
  if(Serial && GRAPH){
    Serial.print(desiredLeft);
    Serial.print(",");
    Serial.print(desiredRight);
    Serial.print(",");
    Serial.print(lVolt);
    Serial.print(",");
    Serial.print(rVolt);
    Serial.print(",");
    Serial.print(lCnt);
    Serial.print(",");
    Serial.println(rCnt);
    delay(1);
  }
}

// Inturrupt Service Routine
// Returns the LEFT and RIGHT odometry and resets the counters
void onRequestOdometry(){
  Serial.println("REQUESTED DATA");
  Serial.print(lCnt);
  byte buffer[2] = {lCnt, rCnt};
  lCnt = rCnt = 0;
  // Write response to requester
  Wire.write(buffer, 2);
}

// Inturrupt Service Routine
// Write the signed integer to variables and sets the doMotorUpdate flag
// Expect
// 00000011 - Motor direction, 0 = FORWARD, 1 = REVERSE
// BYTE 0-255 speed
void onReceiveMotorSpeed(int byteCount){

    if(byteCount != 3){
      Serial.println("ERROR: I2C Invalid Payload. Expected 2 bytes.");
      return;
    }

    byte b[3];
    for(int i=0; i<byteCount; i++){
      b[i] = Wire.read();
//      Serial.println(b[i], HEX);
//      Serial.println((int8_t) b[i]);
    }

    // Write the new motor speeds
    desiredLeft = (uint8_t) b[1];
    desiredRight = (uint8_t) b[2];

    if(bitRead(b[0], 0) == 1)
      desiredLeft = desiredLeft * -1;

    if(bitRead(b[0], 1) == 1)
      desiredRight = desiredRight * -1;
    
    doMotorUpdate = true;
    lastUpdateTime = millis();
}

// This function should be passed the first pin of the motor, assuming H1, H2, PWM in sequential order.
void setMotorSpeed(int pinStart, int speed){
  if(speed == 0){
    digitalWrite(pinStart, HIGH);
    digitalWrite(pinStart + 1, HIGH);
  }else if(speed > 0){
    digitalWrite(pinStart, LOW);
    digitalWrite(pinStart + 1, HIGH);
  }else if(speed < 0){
    digitalWrite(pinStart, HIGH);
    digitalWrite(pinStart + 1, LOW);
  }
  analogWrite(pinStart + 2, abs(speed));
}
