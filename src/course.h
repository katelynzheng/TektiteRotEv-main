#pragma once
#include "motion.h"

#define MAX_COMMANDS 32

extern Command course[MAX_COMMANDS];
extern int courseLength;
extern int currentCommand;

// block-style helpers
void forward(float meters);
void turnLeft(float degrees);
void turnRight(float degrees);
