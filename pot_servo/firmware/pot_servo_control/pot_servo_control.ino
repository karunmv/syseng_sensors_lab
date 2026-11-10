// =====================================================================
// Pot -> Servo control (sensor mode) + GUI mode over serial
// ---------------------------------------------------------------------
// Calibrated values below come from data/pot_cal.csv and the servo pulse sweep.
//
// Serial commands (115200, "Newline") - later sent by the ROS 2 node:
//   M0        -> SENSOR mode (pot drives servo)          [default]
//   M1        -> GUI mode (servo follows S commands)
//   S<deg>    -> servo target in degrees, e.g. S90 (only used in GUI mode)
//
// Output, one CSV line every 50 ms (easy for ROS 2 to parse, and the
// Arduino Serial Plotter can graph it too):
//   pot_adc,pot_deg,pot_deg_filt,servo_cmd_deg,mode
// =====================================================================
#include <Servo.h>

// ---------- Pins ----------
const int POT_PIN   = A0;
const int SERVO_PIN = 9;   // D9 (Servo lib disables PWM on 9/10 anyway; keeps D2/D3 free for the encoder)

// ---------- Pot calibration (from 2_pot_plot.py) ----------
const float M = 0.37486;   // deg per ADC count (fit 80-300 deg, pot on 3.3V)
const float B = 56.877;    // deg offset
const float POT_MIN_DEG = 80.0;   // usable linear range (outside end dead zones)
const float POT_MAX_DEG = 300.0;

// ---------- Servo calibration (from step 3) ----------
const int   SERVO_MIN_US  = 610;    // measured: -90 deg (ACW limit)
const int   SERVO_MAX_US  = 2500;   // measured: +100 deg (CW limit)
const float SERVO_MIN_DEG = -90.0;  // 0 deg = 1500 us, ~10 us/deg
const float SERVO_MAX_DEG = 100.0;

// ---------- Filter ----------
const float ALPHA = 0.3;   // EMA: 0 = very smooth/slow, 1 = no filtering

Servo servo;
int   mode = 0;            // 0 = sensor, 1 = GUI
float guiTargetDeg = 0.0;
float potFilt = -1;
unsigned long lastTick = 0;

float readPotDeg(int &adcOut) {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(POT_PIN);
  float adc = sum / 8.0;
  adcOut = (int)adc;
  return M * adc + B;
}

// Convert a real angle (deg) to a pulse width using the servo calibration
void servoWriteDeg(float deg) {
  deg = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  float us = SERVO_MIN_US + (deg - SERVO_MIN_DEG) * (SERVO_MAX_US - SERVO_MIN_US)
                            / (SERVO_MAX_DEG - SERVO_MIN_DEG);
  servo.writeMicroseconds((int)us);
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() < 2) return;
  char c = toupper(s.charAt(0));
  float v = s.substring(1).toFloat();
  if (c == 'M') mode = (v > 0.5) ? 1 : 0;
  else if (c == 'S') guiTargetDeg = constrain(v, SERVO_MIN_DEG, SERVO_MAX_DEG);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoWriteDeg(0);
  Serial.println(F("pot_adc,pot_deg,pot_deg_filt,servo_cmd_deg,mode"));
}

void loop() {
  handleSerial();

  if (millis() - lastTick < 50) return;   // 20 Hz, no blocking delay()
  lastTick = millis();

  int potAdc;
  float potDeg = readPotDeg(potAdc);
  potFilt = (potFilt < 0) ? potDeg : ALPHA * potDeg + (1 - ALPHA) * potFilt;

  float servoCmd;
  if (mode == 0) {
    // Sensor mode: map usable pot range linearly onto the servo's full range
    float p = constrain(potFilt, POT_MIN_DEG, POT_MAX_DEG);
    servoCmd = SERVO_MIN_DEG + (p - POT_MIN_DEG) * (SERVO_MAX_DEG - SERVO_MIN_DEG)
                               / (POT_MAX_DEG - POT_MIN_DEG);
  } else {
    servoCmd = guiTargetDeg;
  }
  servoWriteDeg(servoCmd);

  Serial.print(potAdc);         Serial.print(',');
  Serial.print(potDeg, 1);      Serial.print(',');
  Serial.print(potFilt, 1);     Serial.print(',');
  Serial.print(servoCmd, 1);    Serial.print(',');
  Serial.println(mode);
}
