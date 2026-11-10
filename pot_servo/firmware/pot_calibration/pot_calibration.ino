// =====================================================================
// STEP 1 - Potentiometer calibration (data collection)
// ---------------------------------------------------------------------
// Wiring: pot outer pins -> 5V and GND, middle (wiper) pin -> A0
//
// How to use:
//   1. Upload, open Serial Monitor at 115200 baud, line ending = "Newline".
//   2. Type  L  + Enter to toggle a live ADC readout (use it to find the
//      end stops / dead zones and to line up the pointer).
//   3. Turn the knob to a known angle on your paper protractor, then type
//      that angle (e.g. 45) + Enter. The sketch averages 50 samples and
//      prints one CSV line:  angle_deg,adc_mean,adc_std,volts
//   4. Repeat every 15-20 deg from one end stop to the other.
//   5. Copy all CSV lines (including the header) into  pot_cal.csv.
//      Then do a second, smaller set at NEW angles -> pot_test.csv
//      (used to report error honestly).
// =====================================================================

const int POT_PIN = A0;
const int N_SAMPLES = 50;
bool live = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("# Pot calibration. Type angle in degrees + Enter. 'L' toggles live view."));
  Serial.println(F("angle_deg,adc_mean,adc_std,volts"));
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() == 0) return;

    if (s == "L" || s == "l") {
      live = !live;
      Serial.println(live ? F("# live ON") : F("# live OFF"));
      return;
    }

    float angle = s.toFloat();

    // Average N samples and compute standard deviation (noise)
    float sum = 0, sumSq = 0;
    for (int i = 0; i < N_SAMPLES; i++) {
      int v = analogRead(POT_PIN);
      sum += v;
      sumSq += (float)v * v;
      delay(2);
    }
    float mean = sum / N_SAMPLES;
    float stdev = sqrt(max(0.0, sumSq / N_SAMPLES - mean * mean));
    float volts = mean * 5.0 / 1023.0;

    Serial.print(angle, 1);  Serial.print(',');
    Serial.print(mean, 2);   Serial.print(',');
    Serial.print(stdev, 2);  Serial.print(',');
    Serial.println(volts, 3);
  }

  if (live) {
    Serial.print(F("# live ADC: "));
    Serial.println(analogRead(POT_PIN));
    delay(200);
  }
}
