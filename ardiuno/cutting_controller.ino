#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library
#include <Wire.h>

#define MAX_MOTOR_RPM 3500
#define COUNTS_PER_REV 1
#define ENC_A_PIN 3
#define PWM_PIN 5

#define PID_PWM_MIN 0
#define PID_PWM_MAX 128
#define TUNING_MODE true    // Allows Serial inputs to adjust RPM for monitoring graph output, ignores watchdog timer

int lastUpdateTime = 0;     // Safety watchdog from I2C write

volatile double motorCount = 0;
double prevMotorCount = 0;
double prevVelTime = 0;

double targetRPM = 0;
double currentRPM = 0;
double motorPWM = 0;

// PID
// Tuning process, set I=1, D=0
// Increase P until oscilation, half it
// https://pidexplained.com/how-to-tune-a-pid-controller/
double kP=1, kI=1.5, kD=0;

PID motorPID(&currentRPM, &motorPWM, &targetRPM, kP, kI, kD, P_ON_M, DIRECT);


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

  // Pin and Encoder Setup
  pinMode(ENC_A_PIN, INPUT_PULLUP); // We need to PullUp the hall encoder
  pinMode(PWM_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), readEncoder, RISING);

  // Setup the PID
  motorPID.SetOutputLimits(PID_PWM_MIN, PID_PWM_MAX);
  motorPID.SetMode(AUTOMATIC);

  // TODO: Remove
  targetRPM = 200;
  
}

void loop() {

  if(TUNING_MODE && Serial.available()){
    targetRPM = (int) Serial.parseInt();
    while(Serial.available()) Serial.read(); // Read any leftover chars
  }

  // If last update was more than 1.5 seconds ago, stop the motors as we may have lost communication with the host
  if(!TUNING_MODE && millis() - lastUpdateTime >= 1500){
    targetRPM = motorPWM = 0;
    writePwm();
  }

  // Disable the PID if we want 0 RPM
  if(targetRPM == 0) {
    motorPID.SetMode(MANUAL);
    motorPWM = 0;
  }else{
    motorPID.SetMode(AUTOMATIC);
  }

  // Calculate the current motor velocity
  long currentTime = micros();
  float delta = ((float) (currentTime-prevVelTime))/1.0e6;
  
  noInterrupts();
  long countI = motorCount;
  interrupts();

  currentRPM = ((countI - prevMotorCount)/delta)/COUNTS_PER_REV*60.0;    // Convert count/s to RPM

  Serial.println((String) (countI - prevMotorCount) + "," + (String) currentRPM + "," + (String) targetRPM);


  // Save values for next cycle
  prevVelTime = currentTime;
  prevMotorCount = countI;

  // Write PWM
  writePwm();

  // Delay for some delta for velocity measurements
  delay(200);
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
    targetRPM = map(perc, 0, 100, 0, MAX_MOTOR_RPM);

    // Give time of last update for watchdog/failsafe behaviour
    lastUpdateTime = millis();
}

void writePwm(){
    analogWrite(PWM_PIN, motorPWM);
}

void readEncoder(){
  motorCount++;
}
