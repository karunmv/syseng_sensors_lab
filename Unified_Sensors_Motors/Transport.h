#pragma once
#include <Arduino.h>

// One bounded output frame. Never wait for UART space in the motor-control loop.
class Transport : public Print {
public:
  using Print::write;
  size_t write(uint8_t value) override {
    if (length >= sizeof(buffer)) return 0;
    buffer[length++] = value;
    return 1;
  }
  bool busy() const { return length != 0; }
  void pump() {
    int available = Serial.availableForWrite();
    size_t count = min((size_t)available, length - sent);
    if (count) sent += Serial.write(buffer + sent, count);
    if (sent == length) { sent = 0; length = 0; }
  }
private:
  uint8_t buffer[160]; // frames are documented/tested to fit, no heap or String
  size_t length = 0;
  size_t sent = 0;
};
