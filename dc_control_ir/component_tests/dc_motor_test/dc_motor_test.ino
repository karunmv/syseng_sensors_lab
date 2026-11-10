
const int IN1 = 7;
const int IN2 = 8;
const int EN = 6;

void stopMotor() {
  analogWrite(EN, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);

  stopMotor();
  Serial.println("DC motor test ready");
}

void loop() {
  Serial.println("Forward");

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN, 150);
  delay(1000);

  stopMotor();
  Serial.println("Stopped");
  delay(1500);

  Serial.println("Reverse");

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN, 150);
  delay(1000);

  stopMotor();
  Serial.println("Stopped");
  delay(1500);
}
