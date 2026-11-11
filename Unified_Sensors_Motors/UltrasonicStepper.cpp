#include "UltrasonicStepper.h"
#include "Config.h"
#include <math.h>
#include <stdlib.h>

void UltrasonicStepper::begin() {
  pinMode(Config::STEP_PIN, OUTPUT);
  pinMode(Config::DIR_PIN, OUTPUT);
  pinMode(Config::ULTRASONIC_PIN, INPUT);
  digitalWrite(Config::STEP_PIN, LOW);
}

long UltrasonicStepper::degreesToSteps(float value) {
  return lroundf(value * Config::STEPS_PER_REV / 360.0f);
}

float UltrasonicStepper::stepsToDegrees(long value) {
  return value * 360.0f / Config::STEPS_PER_REV;
}

void UltrasonicStepper::hold() {
  target = position;
}

bool UltrasonicStepper::moveRelative(float value) {
  long delta = degreesToSteps(value);
  long next = position + delta;
  if (next < -Config::STEPPER_POSITION_LIMIT || next > Config::STEPPER_POSITION_LIMIT) {
    return false;
  }
  moveStart = position;
  roundedMoveDegrees = stepsToDegrees(delta);
  target = next; // relative to current position, replacing any unfinished move
  return true;
}

float UltrasonicStepper::movedDegrees() const {
  return stepsToDegrees(position - moveStart);
}

void UltrasonicStepper::updateSensor(bool sensorControl) {
  unsigned long now = millis();
  if (now - lastSample < Config::ULTRASONIC_INTERVAL_MS) return;
  lastSample = now;
  adc = analogRead(Config::ULTRASONIC_PIN);
  rawCm = adc * 1.27f;
  filteredCm = initialized ? 0.1f * rawCm + 0.9f * filteredCm : rawCm;
  initialized = true;
  if (!sensorControl) return;
  float bounded = constrain(filteredCm, Config::ULTRASONIC_MIN_CM,
                            Config::ULTRASONIC_MAX_CM);
  float angle = (bounded - Config::ULTRASONIC_MIN_CM) * 180.0f /
               (Config::ULTRASONIC_MAX_CM - Config::ULTRASONIC_MIN_CM);
  long next = degreesToSteps(angle);
  if (labs(next - target) >= Config::STEPPER_SENSOR_DEADBAND) {
    moveStart = position;
    target = next;
    roundedMoveDegrees = stepsToDegrees(target - position);
  }
}

void UltrasonicStepper::updateMotor() {
  if (position == target || micros() - lastStep < Config::STEP_INTERVAL_US) return;
  int direction = target > position ? 1 : -1;
  digitalWrite(Config::DIR_PIN, direction > 0 ? HIGH : LOW);
  delayMicroseconds(10);
  digitalWrite(Config::STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(Config::STEP_PIN, LOW);
  position += direction;
  lastStep = micros(); // never emit a burst of catch-up steps
}
