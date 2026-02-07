#include <TektiteRotEv.h>
#include <PID.h>

#include "util.h"
#include "motion.h"
#include "course.h"

RotEv rotev;

void setup() {
  rotev.begin();
  Serial.println("Hello, RotEv!");  // rotev.begin() automatically starts Serial
                                    // at 115200 baud
}

enum MotionType {
  MOTION_IDLE,
  MOTION_FORWARD,
  MOTION_TURN
};

struct Command {
  MotionType type;
  float value;   // meters for forward, degrees for turn
};

volatile MotionType motion = MOTION_IDLE;
bool going = false;
// Use a boolean variable to trigger the GO functionality only when the button
// is released
bool goPressed = false;

// Variable to see if the GO functionality is currently active
float startTime = 0.0f;

float tireRadius = 0.0301625f; // in meters (1.1875 inches)
float treadWidth = 0.119f;

float rightToLeftPercent = 0.92f;

volatile float x = 0.0f;
volatile float y = 0.0f;

volatile float yaw, yawGyro;

volatile float distanceTraveled;

// Global vars for wheel speed
volatile float wheelSpeedL, wheelSpeedR; 
float targetSpeed = 0.3f;

// Global PID controllers
PID leftPID(0, 1.0f, 0.0f, 0.0f, 0.0f);
PID rightPID(0, 1.0f, 0.0f, 0.0f, 0.0f);
PID headingPID(0, 2.0f, 0.0f, 0.0f, 0.0f);
float targetYaw = 0.0f;
PID distancePID(0, 1.5f, 0.0f, 0.0f, 0.0f);
float targetDistance = 1.0f; // meters


float angleDiff(float target, float current) {
  float diff = target - current;
  while (diff > PI)  diff -= 2.0f * PI;
  while (diff < -PI) diff += 2.0f * PI;
  return diff;
}

// Main loop
void loop() {
  updateOdometry();
  updateMotionState();
  runCommandQueue();

  // Handle button presses
  if (rotev.goButtonPressed()) {
    goPressed = true;
    distanceTraveled = 0.0f;
    x = 0.0f;
    y = 0.0f;
    yaw = 0.0f;
    yawGyro = 0.0f;
    targetYaw = 0.0f;

    leftPID.reset();
    rightPID.reset();
    headingPID.reset();
    distancePID.reset();

    targetYaw = yaw;

    lastCheck = (unsigned long)-1;
    rotev.ledWrite(0.0f, 0.1f, 0.0f);  // 10% green
  } else if (goPressed && !rotev.goButtonPressed()) {
    goPressed = false;

    buildCourse();
    currentCommand = 0;
    // Trigger the GO functionality
    going = true;
    startTime = millis();

    rotev.motorEnable(true);  // Enable the motor drivers

  } else if (rotev.stopButtonPressed()) {
    going = false;
    rotev.motorWrite1(0.0f);
    rotev.motorWrite2(0.0f);
    rotev.motorEnable(false);
    rotev.servoDetach();

    leftPID.reset();
    rightPID.reset();
    stopMotion();
    currentCommand = 0;

    // full reset
    distanceTraveled = 0.0f;
    x = 0.0f;
    y = 0.0f;
    yaw = 0.0f;
    yawGyro = 0.0f;
    targetYaw = 0.0f;
    
  } else {
    rotev.ledWrite(0.0f, 0.0f, 0.1f);  // 10% blue
  }

  // Motor writes
  if (going) {
    float voltage = rotev.getVoltage();
    if (voltage < 1.0f) voltage = 1.0f;  // safety

    float targetSpeed = 0.0f;

    // OUTER LOOP
    if (motion == MOTION_FORWARD) {
      float distanceError = targetDistance - distanceTraveled;
      targetSpeed = distancePID.compute(distanceError);
      targetSpeed = constrain(targetSpeed, -0.4f, 0.4f);
    }

    // MOTION_TURN → targetSpeed stays 0 (turn in place)

    // HEADING CONTROL
    float yawError = angleDiff(targetYaw, yaw);
    float turnCorrection = headingPID.compute(yawError);
    turnCorrection = constrain(turnCorrection, -0.25f, 0.25f);

    // WHEEL SPEED TARGETS
    float leftTarget  = targetSpeed - turnCorrection;
    float rightTarget = targetSpeed + turnCorrection;

    leftTarget  = constrain(leftTarget,  -0.5f, 0.5f);
    rightTarget = constrain(rightTarget, -0.5f, 0.5f);

    // INNER SPEED LOOP
    float errorL = leftTarget  - wheelSpeedL;
    float errorR = rightTarget - wheelSpeedR;

    float cmdL = leftPID.compute(errorL);
    float cmdR = rightPID.compute(errorR);

    // VOLTAGE NORMALIZATION 
    float motorCmdL = cmdL / voltage;
    float motorCmdR = cmdR / voltage;

    motorCmdL = constrain(motorCmdL, -1.0f, 1.0f);
    motorCmdR = constrain(motorCmdR, -1.0f, 1.0f);

    // APPLY 
    rotev.motorWrite1(motorCmdL);
    rotev.motorWrite2(motorCmdR);
  }
}

// SECOND CORE: The RP2350 processor has two cores. You can use the FIFO bus to
// transfer data or use global variables. If using global variables you may want
// to use mutexes.
void setup1() {
  // You can put setup code for the second core here
}

void loop2() {
  // I recommend doing sensor fusion on the second core, since this needs to
  // occur at a high rate. In this example it is printing out sensor data.
  float yawRateDeg = rotev.readYawRateDegrees();
  float enc1Angle = rotev.enc1Angle();
  float enc2Angle = rotev.enc2Angle();
  float curr1 = rotev.motorCurr1();
  float curr2 = rotev.motorCurr2();
  float voltage = rotev.getVoltage();

  // If you print in this format, you can use Arduino's Serial monitor to plot
  // the data over time
  // Serial.print("yawRate:");
  // Serial.print(yawRateDeg);

  // Serial.print(",enc1:");
  // Serial.print(enc1Angle);
  // Serial.print(",enc2:");
  // Serial.print(enc2Angle);

  // Serial.print(",curr1:");
  // Serial.print(curr1);
  // Serial.print(",curr2:");
  // Serial.print(curr2);

  // Serial.print(",voltage:");
  // Serial.print(voltage);

  // Serial.print("Yaw (deg): ");
  // Serial.println(yaw * (180.0f / 3.14159265f));  // Convert to degrees for printing
  // Serial.print("YawGyro (deg): ");
  // Serial.println(yawGyro * (180.0f / 3.14159265f));  // Convert to degrees for printing

  // Serial.println();

  // delay(50);  // This slows down the printing rate. Note that you would not want
              // a delay in your actual code, since sensor fusion must occur at
              // a high frequency
}


unsigned long lastCheck = -1;

float lastEnc1Angle = 0.0f;
float lastEnc2Angle = 0.0f;


void updateOdometry() {
  if (lastCheck == (unsigned long)-1) {
    lastCheck = millis();
    lastEnc1Angle = rotev.enc1Angle();
    lastEnc2Angle = rotev.enc2Angle();
    return;
  }

  unsigned long now = millis();
  float dt = (now - lastCheck);  
  
  if (dt >= 5) {
    lastCheck = now;

    float currEnc1Angle = rotev.enc1Angle();
    float currEnc2Angle = rotev.enc2Angle();

    float deltaEnc1 = angleDiff(currEnc1Angle, lastEnc1Angle);
    float deltaEnc2 = angleDiff(currEnc2Angle, lastEnc2Angle);

    lastEnc1Angle = currEnc1Angle;
    lastEnc2Angle = currEnc2Angle;

    float deltaL = (deltaEnc1 * (tireRadius));  // in meters
    float deltaR = (deltaEnc2 * (tireRadius));  // in meters

    float dtSec = dt / 1000.0f;
    if (dtSec <= 0.0f) return;

    wheelSpeedL = deltaL / dtSec;
    wheelSpeedR = deltaR / dtSec;

    float deltaYaw = (deltaR - deltaL) / treadWidth;  // in radians
    float prevYaw = yaw;
    yaw += deltaYaw;
    yawGyro += rotev.readYawRate() * dtSec;  // Approximate integration
    
    const float fusionFactor = 0.98f;
    yaw = fusionFactor * yawGyro + (1.0f - fusionFactor) * yaw;
    yaw = angleDiff(0.0f, yaw);

    float deltaS = (deltaL + deltaR) * 0.5f;
    x += deltaS * cos(yaw);
    y += deltaS * sin(yaw);
    distanceTraveled += fabs(deltaS);

    
  }
}