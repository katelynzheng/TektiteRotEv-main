#pragma once

struct Command {
  MotionType type;
  float value;
};

extern volatile MotionType motion;
extern bool going;

void startForward(float meters);
void startTurn(float degrees);
void stopMotion();

void runCommandQueue();
void updateMotionState();