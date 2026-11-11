#pragma once
#include <Arduino.h>

class DcMotor {
public:
  enum Mode { IDLE, POSITION, VELOCITY, IR };
  void begin();
  void update();
  void stop();
  void moveRelative(float degrees);
  void setVelocity(float rpm);
  void setIrPwm(int pwm);
  long getCount() const;
  float angle() const;
  Mode mode = IDLE;
  float targetAngle = 0;
  float targetRPM = 0;
  float actualRPM = 0;
  int pwm = 0;
private:
  void drive(int value, int minimum, int maximum);
  void resetVelocity();
  void positionPID(float dt);
  void velocityPI(float dt);
  float posIntegral = 0;
  float previousAngle = 0;
  float velIntegral = 0;
  long previousVelocityCount = 0;
  unsigned long previousPositionTime = 0;
  unsigned long previousVelocityTime = 0;
};

class IrSensor {
public:
  bool update(); // true on a completed ten-sample batch
  float adc = 0;
  float volts = 0;
  float cm = -1;
  bool detected = false;
  int requestedPwm = 0;
private:
  byte level = 5;
  byte count = 0;
  long sum = 0;
  bool sampling = false;
  unsigned long batchStarted = 0;
  unsigned long lastSample = 0;
};
