#include "course.h"

Command course[MAX_COMMANDS];
int courseLength = 0;
int currentCommand = 0;

void forward(float meters) {
  if (courseLength < MAX_COMMANDS) {
    course[courseLength++] = { MOTION_FORWARD, meters };
  }
}

void turnLeft(float degrees) {
  if (courseLength < MAX_COMMANDS) {
    course[courseLength++] = { MOTION_TURN, degrees };
  }
}

void turnRight(float degrees) {
  if (courseLength < MAX_COMMANDS) {
    course[courseLength++] = { MOTION_TURN, -degrees };
  }
}

// NOTE: max # of commands is 32
void buildCourse() {
  courseLength = 0;
  currentCommand = 0;

  // START WRITING MOVEMENT HERE
  forward(1.0);
  turnRight(90);
  forward(0.5);
  turnLeft(45);
  forward(2.0);
}