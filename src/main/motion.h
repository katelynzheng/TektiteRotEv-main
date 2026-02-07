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

class Motion {
public:
    MotionType state;
    float targetDistance;
    float targetYaw;

    void startForward(float meters);
    void startTurn(float degrees);
    void stopMotion();
    void updateMotionState();
};


extern Motion motion;

void startForward(float meters);
void startTurn(float degrees);
void stopMotion();

void runCommandQueue();
void updateMotionState();