#include "Motors.h"
#include "Config.h"
#include <Servo.h>
#include <math.h>

static Servo servo;

bool sensorMode = false;
int servoAngle = 90;
long currentStep = 0;
long targetStep = 0;
static unsigned long lastStepTime = 0;

long degreesToSteps(float degrees) {
  return lroundf(degrees * STEPS_PER_REV / 360.0f);
}

float stepsToDegrees(long steps) {
  return steps * 360.0f / STEPS_PER_REV;
}

void setServoAngle(int angle) {
  servoAngle = constrain(angle, SERVO_MIN, SERVO_MAX);
  servo.write(servoAngle);
}

void setSensorMode(bool enabled) {
  if (sensorMode == enabled) return;
  sensorMode = enabled;
  targetStep = currentStep;
}

void setupMotors() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  servo.attach(SERVO_PIN);
  setServoAngle(90);
}

void updateStepper() {
  if (currentStep == targetStep) return;
  if (micros() - lastStepTime < STEP_INTERVAL_US) return;

  int direction = targetStep > currentStep ? 1 : -1;
  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
  delayMicroseconds(10);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, LOW);

  currentStep += direction;
  lastStepTime = micros();
}
