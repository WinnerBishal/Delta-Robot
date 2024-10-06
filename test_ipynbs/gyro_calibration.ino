#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <AccelStepper.h>
#include <PID_v1.h>

// Function Declarations
float processMPU6050Data();
void calibrateStepperMotor();

// MPU6050 and Stepper Motor Setup
MPU6050 accelgyro;
AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);  // Pins 2 and 3 for step and direction

// Constants
const float stepAngle = 1.8;
const float zeroThreshold = 0.5;
const float gyroOffset = -3.2;
const int numReadings = 5;

// Global Variables
float compAngleX = 0;
float lastTime = 0;
float readings[numReadings] = {0};
int readIndex = 0;
float total = 0;
float average = 0;
bool isCalibrated = false;
int calibrationCounter = 0;

void setup() {
  Wire.begin();
  Serial.begin(38400);
  accelgyro.initialize();

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  delay(1000);  // Stabilization delay
  setpoint = 0;
  myPID.SetMode(AUTOMATIC);
}

float processMPU6050Data() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelAngleX = atan2(ay, az) * 180 / PI;
  float gyroXrate = gx / 131.0;
  compAngleX = 0.98 * (compAngleX + gyroXrate * dt) + 0.02 * accelAngleX;

  return compAngleX - gyroOffset;
}

void calibrateStepperMotor() {
  float error = processMPU6050Data();
  total -= readings[readIndex];
  readings[readIndex] = error;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;

  average = total / numReadings;

  Serial.println("Angle: " + String(average));
  
  if (abs(average) <= zeroThreshold) {
    stepper.stop();
    stepper.setCurrentPosition(0);
    calibrationCounter++;
    if (calibrationCounter > 200) {
      isCalibrated = true;
    }
    return;
  }

  if (abs(average) > 0.5) {  // Dead zone
    int stepCommand = -2 * (average / stepAngle);
    Serial.println("Steps: " + String(stepCommand));
    stepper.move(stepCommand);
  }

  stepper.run();
  delay(50);
}

void loop() {
  while(!isCalibrated){
    calibrateStepperMotor();
  }
  // while(!isCalibrated){
  //   input = processMPU6050Data();
  //   if ((abs(input) <= zeroThreshold) & calibrationCounter > 200){
  //     isCalibrated = 1;
  //   }
  //   Serial.println("Angle : " + String(input));
  //   myPID.Compute();
  //   Serial.println("Steps : " + String(output));
  //   stepper.move(output);
  //   stepper.run();
  //   calibrationCounter+=1;
  // }
  // delay(50);
}
