// =====================================================================
// STEP 3 - Servo (HS-311) calibration
// ---------------------------------------------------------------------
// Why: servo.write(0..180) assumes a pulse range that doesn't match every
// servo. To command REAL degrees and hit both extreme limits safely, find
// the pulse widths (microseconds) for your servo's true end positions.
//
// Wiring: servo signal -> SERVO_PIN, servo red -> EXTERNAL 5-6 V supply,
//         servo black/brown -> supply GND AND Arduino GND (common ground).
//
// How to use (Serial Monitor 115200, line ending "Newline"):
//   - Type a pulse width in microseconds (e.g. 1500) + Enter -> servo moves.
//   - Go down in steps of ~50 us (1000, 900, 800 ...) until the horn stops
//     moving or the servo starts buzzing/straining. Back off ~20 us.
//     That is SERVO_MIN_US. Do the same upward for SERVO_MAX_US.
//   - With a protractor, write down the horn angle at MIN_US, 1500 us and
//     MAX_US. Those give SERVO_MIN_DEG / SERVO_MAX_DEG for the control sketch.
//   Never leave it buzzing against a stop - it heats up and draws big current.
// =====================================================================
#include <Servo.h>

const int SERVO_PIN = 9;   // servo signal on D9
Servo servo;

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN, 500, 2500);  // allow a wide range for testing
  servo.writeMicroseconds(1500);       // start at center
  Serial.println(F("# Servo calibration. Type pulse width in us (500-2500) + Enter."));
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() == 0) return;
    int us = constrain(s.toInt(), 500, 2500);
    servo.writeMicroseconds(us);
    Serial.print(F("pulse_us = "));
    Serial.println(us);
  }
}
