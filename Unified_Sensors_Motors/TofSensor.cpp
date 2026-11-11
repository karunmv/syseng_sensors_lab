#include "TofSensor.h"
#include "Config.h"
#include <Wire.h>

void TofSensor::begin() {
  Wire.begin();
  Wire.setWireTimeout(Config::I2C_TIMEOUT_US, true);
  Wire.clearWireTimeoutFlag();
  sensor.setTimeout(500);
  online = sensor.init();
  if (online) {
    online = sensor.setDistanceMode(VL53L1X::Long) &&
             sensor.setMeasurementTimingBudget(50000);
  }
  if (online) {
    sensor.startContinuous(50);
    online = sensor.last_status == 0 && !Wire.getWireTimeoutFlag();
  }
  lastReady = millis();
  status = online ? -2 : -1;
  // A failed ToF must not stop the other sensors and motors.
}

void TofSensor::update() {
  if (!online) return;
  unsigned long now = millis();
  if (now - lastReady > Config::TOF_STALE_MS) {
    valid = false;
    status = -2;
    filter.reset();
    filteredMm = -1;
  }
  if (now - lastRead < Config::TOF_INTERVAL_MS ||
      now - lastPoll < Config::TOF_POLL_MS) return;
  lastPoll = now;
  Wire.clearWireTimeoutFlag();
  bool ready = sensor.dataReady();
  if (sensor.last_status != 0 || Wire.getWireTimeoutFlag()) {
    online = false;
    valid = false;
    status = -1;
    return;
  }
  if (!ready) return;
  lastRead = now;
  rawMm = sensor.read(false); // already ready: do not wait for a conversion
  if (sensor.last_status != 0 || Wire.getWireTimeoutFlag() || sensor.timeoutOccurred()) {
    online = false;
    valid = false;
    status = -1;
    return;
  }
  lastReady = now;
  status = sensor.ranging_data.range_status;
  valid = status == VL53L1X::RangeValid;
  if (valid) {
    filteredMm = filter.update(rawMm);
    sampleMs = now;
    ++sequence;
  }
}
