#ifndef GLOBALS_H
#define GLOBALS_H

#include "motion.h"  // for MotionType and Command

// Robot state
extern volatile MotionType motion;
extern bool going;

// Position and odometry
extern volatile float x;
extern volatile float y;
extern volatile float yaw;
extern volatile float yawGyro;
extern volatile float distanceTraveled;

// Wheel speeds
extern volatile float wheelSpeedL;
extern volatile float wheelSpeedR;

// Timing
extern unsigned long lastCheck;
extern float lastEnc1Angle;
extern float lastEnc2Angle;

#endif