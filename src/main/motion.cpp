#include "globals.h"
#include "motion.h"
#include "math.h"

Motion motion;  // global object

void Motion::startForward(float meters) {
    distanceTraveled = 0.0f;
    targetDistance = meters;

    targetYaw = yaw; // hold heading

    distancePID.reset();
    headingPID.reset();

    state = MOTION_FORWARD;  // ✅ now it exists
    going = true;

    rotev.motorEnable(true);
}

void Motion::startTurn(float degrees) {
    targetDistance = 0.0f;
    targetYaw = angleDiff(yaw + degrees * 3.14159265 / 180.0f, 0.0f);

    headingPID.reset();
    distancePID.reset();

    state = MOTION_TURN; // ✅ now it exists
    going = true;

    rotev.motorEnable(true);
}

void Motion::stopMotion() {
    rotev.motorWrite1(0.0f);
    rotev.motorWrite2(0.0f);
    rotev.motorEnable(false);

    state = MOTION_IDLE; // ✅ now it exists
    going = false;
}

// ------------------------
// Queue functions
// ------------------------
void runCommandQueue() {
    if (motion.state != MOTION_IDLE) return;
    if (currentCommand >= courseLength) return;

    Command &cmd = course[currentCommand++];

    if (cmd.type == MOTION_FORWARD) {
        motion.startForward(cmd.value);
    } else if (cmd.type == MOTION_TURN) {
        motion.startTurn(cmd.value);
    }
}

void updateMotionState() {
    if (!going) return;

    if (motion.state == MOTION_FORWARD) {
        float error = targetDistance - distanceTraveled;
        if (fabs(error) < 0.01f) {
            motion.stopMotion();
        }
    } else if (motion.state == MOTION_TURN) {
        float yawError = angleDiff(targetYaw, yaw);
        if (fabs(yawError) < (2.0f * 3.14159265 / 180.0f)) {
            motion.stopMotion();
        }
    }
}

// ------------------------
// Enqueue commands
// ------------------------
void enqueueForward(float meters) {
    course[courseLength++] = { MOTION_FORWARD, meters };
}

void enqueueTurn(float degrees) {
    course[courseLength++] = { MOTION_TURN, degrees };
}
