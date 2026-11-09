#include "Sensors.h"
#include "Motors.h"
#include "Config.h"
#include <Arduino.h>
#include <math.h>

float distanceCM = -1.0f;
static float filteredDistanceCM = 0.0f;
static bool filterInitialized = false;
static unsigned long lastSensorTime = 0;

void setupSensors() {
  pinMode(ULTRASONIC_PIN, INPUT);
}

void updateSensors() {
  if (millis() - lastSensorTime < SENSOR_INTERVAL_MS) return;
  lastSensorTime = millis();

  int raw = analogRead(ULTRASONIC_PIN);
  // MaxSonar analog output: Vcc/512 per inch, with matching ADC reference.
  float measuredCM = raw * 1.27f;

  if (!filterInitialized) {
    filteredDistanceCM = measuredCM;
    filterInitialized = true;
  } else {
    filteredDistanceCM = 0.1f * measuredCM + 0.9f * filteredDistanceCM;
  }
  distanceCM = filteredDistanceCM;

  if (!sensorMode) return;

  int potRaw = analogRead(POT_PIN);
  Serial.print("POT_RAW:");
  Serial.println(potRaw);
  int angle = map(potRaw, 0, 1023, SERVO_MIN, SERVO_MAX);
  setServoAngle(angle);

  float bounded = constrain(distanceCM, MIN_DISTANCE_CM, MAX_DISTANCE_CM);
  float stepperAngle = (bounded - MIN_DISTANCE_CM) * 180.0f /
                       (MAX_DISTANCE_CM - MIN_DISTANCE_CM);
  long newTarget = degreesToSteps(stepperAngle);

  if (labs(newTarget - targetStep) >= 50) {
    targetStep = newTarget;
  }
}
