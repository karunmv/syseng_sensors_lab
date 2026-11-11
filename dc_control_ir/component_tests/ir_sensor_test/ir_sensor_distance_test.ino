const int IR_PIN = A2;
const int NUM_SAMPLES = 10;

const float MIN_DISTANCE = 10.0;
const float MAX_DISTANCE = 80.0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  long adcSum = 0;

  // Average 10 sensor readings
  for (int i = 0; i < NUM_SAMPLES; i++) {
    adcSum += analogRead(IR_PIN);
    delay(5);
  }

  float rawADC = adcSum / (float)NUM_SAMPLES;
  float voltage = rawADC * (5.0 / 1023.0);

  // Convert voltage to distance
  float distanceCm = -23.4382 * voltage + 76.6889;

  // Clamp distance to the sensor's usable range
  distanceCm = constrain(distanceCm, MIN_DISTANCE, MAX_DISTANCE);

  Serial.print("Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | Distance: ");
  Serial.print(distanceCm, 2);
  Serial.println(" cm");

  delay(100);
}