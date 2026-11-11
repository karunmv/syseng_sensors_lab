#pragma once
#include <Arduino.h>
#include <VL53L1X.h>
#include "ExponentialFilter.h"

class TofSensor {
public:
  void begin();
  void update();
  bool online = false;
  bool valid = false;
  int status = -1; // -1 init/bus failure, -2 no fresh sample; otherwise library range status
  int rawMm = -1;
  float filteredMm = -1;
  unsigned long sequence = 0; // increments only when a valid pair enters the filter
  unsigned long sampleMs = 0;
private:
  VL53L1X sensor;
  ExponentialFilter filter{0.1f, 10};
  unsigned long lastPoll = 0;
  unsigned long lastRead = 0;
  unsigned long lastReady = 0;
};
