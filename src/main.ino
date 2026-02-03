#include <TektiteRotEv.h>

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

float tireRadius = 0.0301625; // in meters (1.1875 inches)
float treadWidth = 0.119;

float yaw = 0.0f;

// Main loop
void loop() {
  stabilizePosition();
  // Handle button presses
  if (rotev.goButtonPressed()) {
    goPressed = true;
    startTime = millis();

    rotev.ledWrite(0.0f, 0.1f, 0.0f);  // 10% green
  } else if (goPressed && !rotev.goButtonPressed()) {
    goPressed = false;

    // Trigger the GO functionality
    going = true;
    rotev.motorEnable(true);  // Enable the motor drivers
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
    float voltage = rotev.getVoltage();  // Returns the voltage of the battery
    rotev.motorWrite1(-4.0f / voltage);  // Set motor 1 speed to 50%
    rotev.motorWrite2(4.0f / voltage);   // Set motor 2 voltage to 4V

    if (millis() - startTime > 2000) {
      rotev.motorWrite1(0.0f);  // Stop motor 1
      rotev.motorWrite2(0.0f);  // Stop motor 2

      going = false;
    }
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


unsigned long lastCheck = millis();

float lastEnc1Angle = rotev.enc1Angle();
float lastEnc2Angle = rotev.enc2Angle();

float yaw2=0.0f;

void stabilizePosition() {
  unsigned long now = millis();
  float dt = (now - lastCheck);  // Convert ms to s
  
  if (dt >= 5) {
    lastCheck = now;

    float currEnc1Angle = rotev.enc1Angle();
    float currEnc2Angle = rotev.enc2Angle();

    float deltaEnc1 = currEnc1Angle - lastEnc1Angle;
    float deltaEnc2 = currEnc2Angle - lastEnc2Angle;

    lastEnc1Angle = currEnc1Angle;
    lastEnc2Angle = currEnc2Angle;

    float deltaL = (deltaEnc1 * (tireRadius));  // in meters
    float deltaR = (deltaEnc2 * (tireRadius));  // in meters

    float deltaYaw = (deltaR - deltaL) / treadWidth;  // in radians
    yaw += deltaYaw;
    yaw2 += rotev.readYawRate() * 0.005f;  // Approximate integration

    Serial.print("Yaw (deg): ");
    Serial.println(yaw * (180.0f / 3.14159265f));  // Convert to degrees for printing
    Serial.print("Yaw2 (deg): ");
    Serial.println(yaw2 * (180.0f / 3.14159265f));  // Convert to degrees for printing
  }
}