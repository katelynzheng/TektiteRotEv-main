#pragma once

enum MotionType {
  MOTION_IDLE,
  MOTION_FORWARD,
  MOTION_TURN
};

struct Command {
  MotionType type;
  float value;
};

extern volatile MotionType motion;
extern bool going;
extern float distanceTraveled;

void startForward(float meters);
void startTurn(float degrees);
void stopMotion();

void runCommandQueue();
void updateMotionState();