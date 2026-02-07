void startForward(float meters) {
  distanceTraveled = 0.0f;
  targetDistance = meters;

  targetYaw = yaw;   // hold heading

  distancePID.reset();
  headingPID.reset();

  motion = MOTION_FORWARD;
  going = true;

  rotev.motorEnable(true);
}

void startTurn(float degrees) {
  targetDistance = 0.0f;
  targetYaw = angleDiff(yaw + degrees * PI / 180.0f, 0.0f);

  headingPID.reset();
  distancePID.reset();

  motion = MOTION_TURN;
  going = true;

  rotev.motorEnable(true);
}

void stopMotion() {
  rotev.motorWrite1(0.0f);
  rotev.motorWrite2(0.0f);
  rotev.motorEnable(false);

  motion = MOTION_IDLE;
  going = false;
}

void runCommandQueue() {
  if (motion != MOTION_IDLE) return;
  if (currentCommand >= courseLength) return;

  Command &cmd = course[currentCommand++];

  if (cmd.type == MOTION_FORWARD) {
    startForward(cmd.value);
  }
  else if (cmd.type == MOTION_TURN) {
    startTurn(cmd.value);
  }
}

void updateMotionState() {
  if (!going) return;

  if (motion == MOTION_FORWARD) {
    float error = targetDistance - distanceTraveled;
    if (fabs(error) < 0.01f) {   // 1 cm tolerance
      stopMotion();
    }
  }

  else if (motion == MOTION_TURN) {
    float yawError = angleDiff(targetYaw, yaw);
    if (fabs(yawError) < (2.0f * PI / 180.0f)) { // 2 deg tolerance
      stopMotion();
    }
  }
}

void enqueueForward(float meters) {
  course[courseLength++] = { MOTION_FORWARD, meters };
}

void enqueueTurn(float degrees) {
  course[courseLength++] = { MOTION_TURN, degrees };
}