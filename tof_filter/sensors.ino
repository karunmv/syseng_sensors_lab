
#include <Wire.h>
#include <VL53L1X.h>
#include "ExponentialFilter.h"

// ==========================
// Filter configuration
// ==========================
const float ALPHA = 0.1; // relative weight of the newer data
const int WINDOW_MS = 1000;
const int SAMPLE_INTERVAL_MS = 100;

const int WINDOW_SAMPLES =
    (WINDOW_MS + SAMPLE_INTERVAL_MS - 1)
    / SAMPLE_INTERVAL_MS;

// Create filter object
ExponentialFilter filter(ALPHA, WINDOW_SAMPLES);

// ==========================
// ToF sensor
// ==========================
VL53L1X tof;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  tof.setTimeout(500);

  if (!tof.init()) {
    Serial.println("ToF initialization failed");
    while (1);
  }

  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(50);

  Serial.println("ToF initialized successfully");
  Serial.println("Raw_mm,Filtered_mm");
}

void loop() {

  int rawMm = tof.read();

  if (tof.timeoutOccurred()) {
    Serial.println("ToF timeout");
    return;
  }

  if (tof.ranging_data.range_status !=
      VL53L1X::RangeValid) {
    Serial.println("Invalid ToF reading");
    delay(SAMPLE_INTERVAL_MS);
    return;
  }

  // Apply exponential filter
  float filteredMm = filter.update(rawMm);

  // Print both values
  Serial.print("Raw_ToF:");
  Serial.print(rawMm);

  Serial.print(",");

  Serial.print("Filtered_ToF:");
  Serial.println(filteredMm, 2);

  delay(SAMPLE_INTERVAL_MS);
}
