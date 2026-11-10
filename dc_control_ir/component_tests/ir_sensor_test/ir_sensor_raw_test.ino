const int IR_PIN = A2;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(IR_PIN);
  float voltage = raw * 5.0 / 1023.0;

  Serial.print("Raw ADC: ");
  Serial.print(raw);

  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);

  Serial.println(" V");

  delay(50);
}