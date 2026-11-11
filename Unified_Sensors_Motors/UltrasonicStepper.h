#pragma once
#include <Arduino.h>

class UltrasonicStepper {
public:
  void begin();
  void updateSensor(bool sensorControl);
  void updateMotor();
  void hold();
  bool moveRelative(float degrees);
  static long degreesToSteps(float degrees);
  static float stepsToDegrees(long steps);
  float movedDegrees() const;
  int adc = 0;
  float rawCm = 0;
  float filteredCm = 0;
  long position = 0; // commanded step count, not encoder feedback
  long target = 0;
  float roundedMoveDegrees = 0;
private:
  bool initialized = false;
  long moveStart = 0;
  unsigned long lastSample = 0;
  unsigned long lastStep = 0;
};
