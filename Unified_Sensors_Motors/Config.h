#pragma once
#include <Arduino.h>

// Uno pin allocation. Servo uses Timer1; DC PWM remains on Timer0 pin D6.
namespace Config {
constexpr byte ENCODER_A = 2;
constexpr byte ENCODER_B = 3;
constexpr byte STEP_PIN = 4;
constexpr byte DIR_PIN = 5;
constexpr byte DC_PWM_PIN = 6;
constexpr byte DC_IN1 = 7;
constexpr byte DC_IN2 = 8;
constexpr byte SERVO_PIN = 9;
constexpr byte BUTTON_PIN = 12; // button to GND, internal pull-up
constexpr byte POT_PIN = A0;    // retain the measured 3.3 V pot supply
constexpr byte ULTRASONIC_PIN = A1;
constexpr byte IR_PIN = A2;
// VL53L1X: SDA=A4, SCL=A5. Keep the default 5 V ADC reference.

constexpr unsigned long BAUD = 115200;
constexpr unsigned long DEBOUNCE_MS = 40;
constexpr unsigned long TELEMETRY_MS = 100;
constexpr unsigned long POT_INTERVAL_MS = 50;
constexpr float POT_M = 0.37486f;
constexpr float POT_B = 56.877f;
constexpr float POT_MIN_DEG = 80.0f;
constexpr float POT_MAX_DEG = 300.0f;
constexpr float POT_ALPHA = 0.3f;
constexpr int SERVO_MIN_US = 610;
constexpr int SERVO_MAX_US = 2500;
constexpr float SERVO_MIN_DEG = -90.0f;
constexpr float SERVO_MAX_DEG = 100.0f;

constexpr long STEPS_PER_REV = 1600; // verify EasyDriver microstep setting
constexpr unsigned long STEP_INTERVAL_US = 5000;
constexpr unsigned long ULTRASONIC_INTERVAL_MS = 60;
constexpr float ULTRASONIC_MIN_CM = 15.0f;
constexpr float ULTRASONIC_MAX_CM = 75.0f;
constexpr long STEPPER_SENSOR_DEADBAND = 50;
constexpr float STEPPER_MAX_MOVE_DEG = 360.0f;
constexpr long STEPPER_POSITION_LIMIT = 100000000L; // prevent count overflow

constexpr float DC_COUNTS_PER_REV = 12375.12f;
constexpr float DC_COUNTS_PER_DEG = DC_COUNTS_PER_REV / 360.0f;
constexpr int DC_MAX_PWM = 180;
constexpr int DC_MIN_PWM = 65;
constexpr int DC_VELOCITY_MIN_PWM = 55;
constexpr float DC_MAX_RPM = 12.0f;
constexpr float DC_MAX_MOVE_DEG = 3600.0f;
constexpr float DC_POSITION_TOLERANCE = 1.5f;
constexpr unsigned long DC_POSITION_INTERVAL_MS = 10;
constexpr unsigned long DC_VELOCITY_INTERVAL_MS = 50;
constexpr float DC_KP_POS = 3.0f;
constexpr float DC_KI_POS = 0.05f;
constexpr float DC_KD_POS = 0.2f;
constexpr float DC_KP_VEL = 4.0f;
constexpr float DC_KI_VEL = 3.0f;
constexpr float DC_VELOCITY_DEADBAND = 0.10f;

constexpr byte IR_SAMPLES = 10;
constexpr unsigned long IR_SAMPLE_INTERVAL_MS = 5;
constexpr unsigned long IR_BATCH_INTERVAL_MS = 150; // original ~50 ms sampling + 100 ms delay
constexpr unsigned long TOF_INTERVAL_MS = 100;
constexpr unsigned long TOF_POLL_MS = 5;
constexpr unsigned long TOF_STALE_MS = 500;
constexpr unsigned long I2C_TIMEOUT_US = 3000;
}
