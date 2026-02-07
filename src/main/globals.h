#ifndef GLOBALS_H
#define GLOBALS_H

#include "TektiteRotEv.h"
#include "PID.h"
#include "util.h"
#include "motion.h"  // for MotionType and Command

extern RotEv rotev;
// Robot state
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
extern float targetYaw;
extern float targetDistance;

extern PID leftPID;
extern PID rightPID;
extern PID headingPID;
extern PID distancePID;

// Course queue
extern Command course[];
extern int courseLength;
extern int currentCommand;

#endif