const int IR_PIN = A2;

// L298 motor driver pins
const int IN1 = 7;
const int IN2 = 8;
const int EN  = 6;

const int NUM_SAMPLES = 10;

const float MIN_DISTANCE = 10.0;
const float MAX_DISTANCE = 80.0;

// Distance thresholds
const float thresholds[] = {10, 20, 30, 40, 50};
const int NUM_THRESHOLDS = 5;

// PWM levels
const int pwmLevels[] = {220, 190, 155, 115, 80, 55};

// Distance hysteresis
const float HYSTERESIS = 2.0;

// No-detection voltage hysteresis
const float STOP_VOLTAGE = 0.30;
const float RESUME_VOLTAGE = 0.35;

int currentLevel = 5;
bool noDetection = true;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(EN, 0);
}

void loop() {
  long adcSum = 0;

  // Average 10 readings
  for (int i = 0; i < NUM_SAMPLES; i++) {
    adcSum += analogRead(IR_PIN);
    delay(5);
  }

  float rawADC = adcSum / (float)NUM_SAMPLES;
  float voltage = rawADC * (5.0 / 1023.0);

  // Check whether the sensor detects a valid target
  if (noDetection && voltage > RESUME_VOLTAGE) {
    noDetection = false;
  }
  else if (!noDetection && voltage < STOP_VOLTAGE) {
    noDetection = true;
  }

  int pwm = 0;

  if (noDetection) {
    // Stop motor when no valid detection
    analogWrite(EN, 0);
  }
  else {
    // Convert voltage to distance
    float distanceCm = -23.4382 * voltage + 76.6889;

    // Clamp distance
    distanceCm = constrain(
      distanceCm, MIN_DISTANCE, MAX_DISTANCE
    );

    // Update speed level with hysteresis
    while (currentLevel > 0 &&
           distanceCm < thresholds[currentLevel - 1] - HYSTERESIS) {
      currentLevel--;
    }

    while (currentLevel < NUM_THRESHOLDS &&
           distanceCm > thresholds[currentLevel] + HYSTERESIS) {
      currentLevel++;
    }

    pwm = pwmLevels[currentLevel];

    analogWrite(EN, pwm);

    Serial.print("Distance: ");
    Serial.print(distanceCm, 1);
    Serial.print(" cm | ");
  }

  if (noDetection) {
    Serial.print("NO DETECTION | ");
  }

  Serial.print("Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | PWM: ");
  Serial.println(pwm);

  delay(100);
}