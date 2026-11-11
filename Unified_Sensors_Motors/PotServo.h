#pragma once
#include <Arduino.h>
#include <Servo.h>

class PotServo {
public:
  void begin();
  void update(bool sensorControl);
  void setAngle(float degrees);
  int adc = 0;
  float degrees = 0;
  float filteredDegrees = 0;
  float commandDegrees = 0;
private:
  Servo servo;
  bool initialized = false;
  unsigned long lastSample = 0;
};
