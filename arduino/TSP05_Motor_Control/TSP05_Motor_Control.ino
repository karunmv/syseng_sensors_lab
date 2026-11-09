#include "Motors.h"
#include "Sensors.h"
#include "Interface.h"

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupSensors();
  setupInterface();
  Serial.println("READY:ULTRASONIC");
}

void loop() {
  updateButton();
  readSerial();
  updateSensors();
  updateStepper();
  reportStatus();
}
