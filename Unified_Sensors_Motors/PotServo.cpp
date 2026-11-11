#include "PotServo.h"
#include "Config.h"

void PotServo::begin() {
  servo.attach(Config::SERVO_PIN, Config::SERVO_MIN_US, Config::SERVO_MAX_US);
  setAngle(0);
}

void PotServo::setAngle(float value) {
  commandDegrees = constrain(value, Config::SERVO_MIN_DEG, Config::SERVO_MAX_DEG);
  // Preserve the original calibrated endpoint interpolation, including truncation.
  float us = Config::SERVO_MIN_US + (commandDegrees - Config::SERVO_MIN_DEG) *
      (Config::SERVO_MAX_US - Config::SERVO_MIN_US) /
      (Config::SERVO_MAX_DEG - Config::SERVO_MIN_DEG);
  servo.writeMicroseconds((int)us);
}

void PotServo::update(bool sensorControl) {
  unsigned long now = millis();
  if (now - lastSample < Config::POT_INTERVAL_MS) return;
  lastSample = now;
  long sum = 0;
  for (byte i = 0; i < 8; ++i) sum += analogRead(Config::POT_PIN);
  float average = sum / 8.0f;
  adc = (int)average;
  degrees = Config::POT_M * average + Config::POT_B;
  filteredDegrees = initialized ? Config::POT_ALPHA * degrees +
      (1 - Config::POT_ALPHA) * filteredDegrees : degrees;
  initialized = true;
  if (sensorControl) {
    float bounded = constrain(filteredDegrees, Config::POT_MIN_DEG, Config::POT_MAX_DEG);
    setAngle(Config::SERVO_MIN_DEG + (bounded - Config::POT_MIN_DEG) *
        (Config::SERVO_MAX_DEG - Config::SERVO_MIN_DEG) /
        (Config::POT_MAX_DEG - Config::POT_MIN_DEG));
  }
}
