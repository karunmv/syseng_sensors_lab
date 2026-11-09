#include "Interface.h"
#include "Motors.h"
#include "Sensors.h"
#include "Config.h"
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

static bool lastButtonReading = HIGH;
static bool stableButtonState = HIGH;
static unsigned long lastBounceTime = 0;
static unsigned long lastStatusTime = 0;
static char command[32];
static byte commandLength = 0;

void setupInterface() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void updateButton() {
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonReading) lastBounceTime = millis();

  if (millis() - lastBounceTime >= DEBOUNCE_MS &&
      reading != stableButtonState) {
    stableButtonState = reading;
    if (stableButtonState == LOW) setSensorMode(!sensorMode);
  }
  lastButtonReading = reading;
}

static void executeCommand(char *line) {
  char *end = nullptr;
  double value = strtod(line + 1, &end);
  if (end == line + 1 || !isfinite(value)) return;
  while (*end == ' ' || *end == '\t') ++end;
  if (*end != '\0') return;

  if (line[0] == 'M') {
    if (value == 0 || value == 1) setSensorMode(value == 1);
    return;
  }
  if (sensorMode) return;

  if (line[0] == 'S') {
    if (value >= 0 && value <= 180) setServoAngle((int)round(value));
  } else if (line[0] == 'T') {
    if (value >= -360 && value <= 360) {
      targetStep += degreesToSteps(value);
    }
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (commandLength) {
        command[commandLength] = '\0';
        executeCommand(command);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(command) - 1) {
      command[commandLength++] = c;
    } else {
      commandLength = 0;
    }
  }
}

void reportStatus() {
  if (millis() - lastStatusTime < STATUS_INTERVAL_MS) return;
  lastStatusTime = millis();

  Serial.print("MODE:");
  Serial.print(sensorMode ? "SENSOR" : "GUI");
  Serial.print(",SERVO_DEG:");
  Serial.print(servoAngle);
  Serial.print(",STEPPER_TARGET_DEG:");
  Serial.print(stepsToDegrees(targetStep), 2);
  Serial.print(",STEPPER_EST_DEG:");
  Serial.print(stepsToDegrees(currentStep), 2);
  Serial.print(",ULTRASONIC_CM:");
  Serial.println(distanceCM, 2);
}
