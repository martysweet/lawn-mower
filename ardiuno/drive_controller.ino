#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library
#include <Wire.h>

// Use sequential pins (expecting 5 pins - H1,H2,PWM,ENCA,ENCB)
// H1, H2 are the pins controlling the H-Bridge Driver (sometimes labelled IN1,IN2,IN3,IN4 on LN298 modules)
// Swap these pins around, or the motor connection if the direction is incorrect
// PWM is the PWM output to the motor controller
// ENCA and ENCB are used for inturrupts of motor position
#define LEFT 3    // 3, 4, 5, 6, 7
#define RIGHT 8   // 8, 9, 10, 11, 12

// Use these to define the pin offset i.e. LEFT + H2 = Pin 4
#define H1 0
#define H2 1
#define PWM 2
#define ENCA 3
#define ENCB 4

// The number of encoder counts which equal a single revolution
#define COUNTS_PER_REV 1440
#define TUNING_MODE false                   // Allows Serial inputs to adjust RPM for monitoring graph output, ignores watchdog timer
#define PID_PWM_MIN -128                    // We are running 12v motors on 24v, so we half the PWM range to prevent excess current draw
#define PID_PWM_MAX 128


// Velocity and RPM Tracking
double leftTargetRPM, rightTargetRPM = 0;    // Input to the PID Controllers
double leftCurrentRPM, rightCurrentRPM = 0;  // Error to the PID Controllers
double leftPWM, rightPWM = 0;                // Output from the PID Controllers
volatile double leftCount, rightCount = 0;   // Hall Counter for RPM computation
double leftCountPrev, rightCountPrev = 0;    // Hall Counter for RPM computation
double previousVelTime = 0;                  // Main Loop Velocity Time for Deltas
int lastUpdateTime = 0;                      // Safety watchdog from I2C write

// PID
// Tuning process, set I=1, D=0
// Increase P until oscilation, half it
// https://pidexplained.com/how-to-tune-a-pid-controller/
double kP=1, kI=1.5, kD=0;

PID leftPID(&leftCurrentRPM, &leftPWM, &leftTargetRPM, kP, kI, kD, P_ON_M, DIRECT);
PID rightPID(&rightCurrentRPM, &rightPWM, &rightTargetRPM, kP, kI, kD, P_ON_M, DIRECT);

// I2C Specification
// Operations
//
// - Write Motor Direction/Speed in RPM (onReceive)
//    - Perform a 3-byte write to the device
//    - Byte 1, LSB LEFT direction, LSB+1 RIGHT direction, where 0=forward,1=reverse
//    - Byte 2, Byte 3, LEFT and RIGHT speed 0-255 RPM
//
// - Read odometry count (onRequest)
//    - Returns 1-byte for each counter (2-byte response) TODO Define fully
//    - [Left, Right]

// Function declartion to allow calls before function definitions
void setMotorPWM(int, double);
void onRequestOdometry();
void onReceiveMotorSpeed(int);
void readEncoderLeft();
void readEncoderRight();
void setup();
void loop();

// Setup the Pins
void setup() {

  // Left Motor
  for (int i = LEFT; i <= LEFT + PWM; i++) pinMode(i, OUTPUT);
  for (int i = LEFT + ENCA; i <= LEFT + ENCB; i++) pinMode(i, INPUT);

  // Right Motor
  for (int i = RIGHT; i <= RIGHT + PWM; i++) pinMode(i, OUTPUT);
  for (int i = RIGHT + ENCA; i <= RIGHT + ENCB; i++) pinMode(i, INPUT);

  // Initialise the motors
  setMotorPWM(LEFT, 0);
  setMotorPWM(RIGHT, 0);

  // Start the I2C Bus
  Wire.setSDA(16);  // RPI Pico
  Wire.setSCL(17);
  Wire.begin(9); // AddressL 0x9, 0001001
  Wire.onRequest(onRequestOdometry);
  Wire.onReceive(onReceiveMotorSpeed);

  // Configure the PID controllers
  leftPID.SetOutputLimits(PID_PWM_MIN, PID_PWM_MAX);
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(PID_PWM_MIN, PID_PWM_MAX);
  rightPID.SetMode(AUTOMATIC);


  // Attach inturrupts for counters, on ENCA RISING, we check ENCB to determine direction
  attachInterrupt(digitalPinToInterrupt(LEFT + ENCA), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT + ENCA), readEncoderRight, RISING);

  // Serial Output
  Serial.begin(115200);

  delay(2000);
}

// ISRs triggered on encoder rising
// These will be triggered COUNTS_PER_REV for each single wheel revolution, they must do minimal work
void readEncoderLeft() {
  if (digitalRead(LEFT + ENCB) > 0) {
    leftCount++;
  } else {
    leftCount--;
  }
}

void readEncoderRight() {
  if (digitalRead(RIGHT + ENCB) > 0) {
    rightCount++;
  } else {
    rightCount--;
  }
}

// Main Loop
// Keep track of wheel speeds and fetch new values from PID Controller
void loop() {

  // Allow for PID Tuning, sending manual RPMS to observe Integral behaviour
  if(TUNING_MODE && Serial.available()){
      leftTargetRPM = rightTargetRPM = (int) Serial.parseInt();
      while(Serial.available()) Serial.read(); // Read any leftover chars
  }

  // If last update was more than 2 seconds ago, stop the motors as we may have lost
  // communication with the host
  if(!TUNING_MODE && millis() - lastUpdateTime >= 2000){
    leftTargetRPM = rightTargetRPM = 0;
    setMotorPWM(LEFT, 0);
    setMotorPWM(RIGHT, 0);
    Serial.println("Communication timeout, skipping PID");
    return;
  }

  // Check if target RPM = 0
  // and disable the related PID controller to set PWM=0
  if(leftTargetRPM == 0) {
    leftPID.SetMode(MANUAL);
    leftPWM = 0;
  }else{
    leftPID.SetMode(AUTOMATIC);
  }

  if(rightTargetRPM == 0) {
    rightPID.SetMode(MANUAL);
    rightPWM = 0;
  }else{
    rightPID.SetMode(AUTOMATIC);
  }

  // Calculate loop differences
  long currentTime = micros();
  float delta = ((float) (currentTime - previousVelTime)) / 1.0e6;

  // Get LEFT/RIGHT counts without interruptions
  noInterrupts();
  long leftCountI = leftCount;          // Make a copy of leftCount incase we are inturrptuted during this calculation
  long rightCountI = rightCount;        // Make a copy of rightCount incase we are inturrptuted during this calculation
  interrupts();

  // Compute LEFT Velocity PID
  leftCurrentRPM = ((leftCountI - leftCountPrev) / delta) / COUNTS_PER_REV * 60.0; // Convert count/s to RPM
  leftPID.Compute();

  // Compute RIGHT Velocity PID
  rightCurrentRPM = ((rightCountI - rightCountPrev) / delta) / COUNTS_PER_REV * 60.0; // Convert count/s to RPM
  rightPID.Compute();

  // Save values for next cycle
  previousVelTime = currentTime;
  leftCountPrev = leftCountI;
  rightCountPrev = rightCountI;

  // Write PWM
  setMotorPWM(LEFT, leftPWM);
  setMotorPWM(RIGHT, rightPWM);

  // Output for debug
  if(TUNING_MODE){
    //Serial.println((String) rightTargetRPM + "," + (String) rightCurrentRPM + "," +  (String) rightPWM);
    Serial.println((String) leftTargetRPM + "," + (String) leftCurrentRPM + "," +  (String) leftPWM);
  }
  // Delay for some delta to allow for enough counts for velocity measurements
  delay(50);
}

// Inturrupt Service Routine
// Returns the LEFT and RIGHT odometry and resets the counters
void onRequestOdometry() {
  byte buffer[2] = {leftCurrentRPM, rightCurrentRPM}; // TODO: We can either pass velocity or absolute change back, which one is better?
  // Write response to requester
  Wire.write(buffer, 2);
}

// Inturrupt Service Routine
// Write the unsigned integer to variables to be used by the PID Controller
// Expect
// 00000011 - Motor direction, 0 = FORWARD, 1 = REVERSE
// BYTE 0-255 Target RPM
void onReceiveMotorSpeed(int byteCount) {

  if (byteCount != 3) {
    Serial.println("ERROR: I2C Invalid Payload. Expected 2 bytes.");
    while (Wire.available()) {
      Wire.read(); // Clear the buffer
    }
    return;
  }

  byte b[3];
  for (int i = 0; i < byteCount; i++) {
    b[i] = Wire.read();
  }

  // Write the new motor speeds
  leftTargetRPM = (uint8_t) b[1];
  if (bitRead(b[0], 0) == 1)
    leftTargetRPM = leftTargetRPM * -1;

  rightTargetRPM = (uint8_t) b[2];
  if (bitRead(b[0], 1) == 1)
    rightTargetRPM = rightTargetRPM * -1;

  // Give time of last update for watchdog/failsafe behaviour
  lastUpdateTime = millis();
}

// This function should be passed the first pin of the motor, assuming H1, H2, PWM in sequential order.
void setMotorPWM(int pinStart, double pwm) {
  if (pwm == 0) {
    digitalWrite(pinStart + H1, HIGH);
    digitalWrite(pinStart + H2, HIGH);
    analogWrite(pinStart + PWM, 255); // Enable Power Braking to stop rotation
  } else if (pwm > 0) {
    digitalWrite(pinStart + H1, LOW);
    digitalWrite(pinStart + H2, HIGH);
    analogWrite(pinStart + PWM, abs(pwm));
  } else if (pwm < 0) {
    digitalWrite(pinStart + H1, HIGH);
    digitalWrite(pinStart + H2, LOW);
    analogWrite(pinStart + PWM, abs(pwm));
  }
  
}
