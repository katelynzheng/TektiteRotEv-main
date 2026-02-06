#include <TektiteRotEv.h>

#include "util.h"

RotEv rotev;

void setup() {
  rotev.begin();
  Serial.println("Hello, RotEv!");  // rotev.begin() automatically starts Serial
                                    // at 115200 baud
}

// Use a boolean variable to trigger the GO functionality only when the button
// is released
bool goPressed = false;

// Variable to see if the GO functionality is currently active
bool going = false;
float startTime = 0.0f;

float tireRadius = 0.0301625f; // in meters (1.1875 inches)
float treadWidth = 0.119f;

float rightToLeftPercent = 0.92f;

float yaw = 0.0f;
float x = 0.0f;
float y = 0.0f;

int speedUpTime = 500;

void run() {
  float voltage = rotev.getVoltage();  // Returns the voltage of the battery
  rotev.motorWrite1(-4.0f / voltage );  // Set motor 1 speed to 4Volts forward
  rotev.motorWrite2(4.0f / voltage * rightToLeftPercent); // Set motor 2 speed to 4Volts Forward

  delay(1000); // delays 1000 ms
}

// Main loop
void loop() {
  stabilizePosition();
  // Handle button presses
  if (rotev.goButtonPressed()) {
    goPressed = true;

    rotev.ledWrite(0.0f, 0.1f, 0.0f);  // 10% green
  } else if (goPressed && !rotev.goButtonPressed()) {
    goPressed = false;

    // Trigger the GO functionality
    going = true;
    startTime = millis();

    rotev.motorEnable(true);  // Enable the motor drivers

    run();


  } else if (rotev.stopButtonPressed()) {
    going = false;
    rotev.motorEnable(false);  // Disable the motor drivers
    rotev.servoDetach();       // Disables the servo

    rotev.ledWrite(0.1f, 0.0f, 0.0f);  // 10% red, since stop is pressed
  } else {
    rotev.ledWrite(0.0f, 0.0f, 0.1f);  // 10% blue
  }

  // Motor writes
  if (going) {
     // Set motor 2 voltage to 4V

    // if (millis() - startTime > 2000) {
    //   rotev.motorWrite1(0.0f);  // Stop motor 1
    //   rotev.motorWrite2(0.0f);  // Stop motor 2

    //   going = false;
    // }
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
  Serial.print("yawRate:");
  Serial.print(yawRateDeg);

  Serial.print(",enc1:");
  Serial.print(enc1Angle);
  Serial.print(",enc2:");
  Serial.print(enc2Angle);

  Serial.print(",curr1:");
  Serial.print(curr1);
  Serial.print(",curr2:");
  Serial.print(curr2);

  Serial.print(",voltage:");
  Serial.print(voltage);

  Serial.println();

  delay(50);  // This slows down the printing rate. Note that you would not want
              // a delay in your actual code, since sensor fusion must occur at
              // a high frequency
}


unsigned long lastCheck = -1;

float lastEnc1Angle = 0.0f;
float lastEnc2Angle = 0.0f;

float yaw2=0.0f;

void stabilizePosition() {
  if (lastCheck == (unsigned long)-1) {
    lastCheck = millis();
    lastEnc1Angle = rotev.enc1Angle();
    lastEnc2Angle = rotev.enc2Angle();
    return;
  }

  unsigned long now = millis();
  float dt = (now - lastCheck);  // Convert ms to s
  
  if (dt >= 5) {
    lastCheck = now;

    float currEnc1Angle = rotev.enc1Angle();
    float currEnc2Angle = rotev.enc2Angle();

    float deltaEnc1 = reduce_negative_180_to_180(currEnc1Angle - lastEnc1Angle); // ADD ANGLE REDUCTION TO POSITIVE
    float deltaEnc2 = reduce_negative_180_to_180(currEnc1Angle - lastEnc2Angle);

    lastEnc1Angle = currEnc1Angle;
    lastEnc2Angle = currEnc2Angle;

    float deltaL = (deltaEnc1 * (tireRadius));  // in meters
    float deltaR = (deltaEnc2 * (tireRadius));  // in meters

    float deltaYaw = (deltaR - deltaL) / treadWidth;  // in radians
    float prevYaw = yaw;
    yaw += deltaYaw;
    yaw2 += rotev.readYawRate() * 0.005f;  // Approximate integration

    Serial.print("Yaw (deg): ");
    Serial.println(yaw * (180.0f / 3.14159265f));  // Convert to degrees for printing
    // Serial.print("Yaw2 (deg): ");
    // Serial.println(yaw2 * (180.0f / 3.14159265f));  // Convert to degrees for printing

    float local_y;

    if (deltaYaw == 0) {
      local_y = deltaR;
    } else {
      local_y = (2*sin(deltaYaw/2))*((deltaR/deltaYaw)+treadWidth/2.0f);
    }

    if (local_y != 0) {

      float local_polar_angle = atan2(local_y, 0.0f);

      float globalAngleInv = local_polar_angle - prevYaw - deltaYaw/2;

      float delta_X = fabs(local_y) * cos(globalAngleInv);
      float delta_Y = fabs(local_y) * sin(globalAngleInv);

      x += delta_X;
      y += delta_Y;
    }

    Serial.print("X: ");
    Serial.println(x);
    Serial.print("Y: ");
    Serial.println(y);
  }
}