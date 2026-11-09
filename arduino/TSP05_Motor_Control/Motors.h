#pragma once
#include <Arduino.h>

extern bool sensorMode;
extern int servoAngle;
extern long currentStep;
extern long targetStep;

void setupMotors();
void setServoAngle(int angle);
void setSensorMode(bool enabled);
void updateStepper();
long degreesToSteps(float degrees);
float stepsToDegrees(long steps);
