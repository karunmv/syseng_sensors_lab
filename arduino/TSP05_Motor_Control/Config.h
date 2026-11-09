#pragma once
#include <Arduino.h>

constexpr byte SERVO_PIN = 3;
constexpr byte STEP_PIN = 4;
constexpr byte DIR_PIN = 5;
constexpr byte BUTTON_PIN = 7;
constexpr byte POT_PIN = A0;
constexpr byte ULTRASONIC_PIN = A1;

constexpr int SERVO_MIN = 45;
constexpr int SERVO_MAX = 135;
constexpr long STEPS_PER_REV = 1600;
constexpr unsigned long STEP_INTERVAL_US = 5000;
constexpr unsigned long DEBOUNCE_MS = 40;
constexpr unsigned long SENSOR_INTERVAL_MS = 60;
constexpr unsigned long STATUS_INTERVAL_MS = 200;
constexpr float MIN_DISTANCE_CM = 15.0f;
constexpr float MAX_DISTANCE_CM = 75.0f;
