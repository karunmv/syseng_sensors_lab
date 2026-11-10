// ======================================
// DC Motor + Encoder Test
// ======================================

// Motor driver pins
const int IN1 = 7;
const int IN2 = 8;
const int EN  = 6;

// Encoder pins
const int ENCODER_A = 2;
const int ENCODER_B = 3;

// Encoder count
volatile long encoderCount = 0;

// ======================================
// Encoder interrupt
// ======================================
void readEncoder() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ======================================
// Motor functions
// ======================================
void stopMotor() {
  analogWrite(EN, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void forwardMotor(int pwm) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN, pwm);
}

void reverseMotor(int pwm) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN, pwm);
}

// ======================================
// Print encoder count
// ======================================
void printEncoderCount() {
  noInterrupts();
  long count = encoderCount;
  interrupts();

  Serial.print("Encoder count: ");
  Serial.println(count);
}

// ======================================
// Setup
// ======================================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A),
    readEncoder,
    CHANGE
  );

  stopMotor();

  Serial.println("DC Motor + Encoder Test Started");
}

// ======================================
// Main loop
// ======================================
void loop() {

  // Reset count
  noInterrupts();
  encoderCount = 0;
  interrupts();

  // Forward
  Serial.println("\nFORWARD");
  forwardMotor(100);

  for (int i = 0; i < 20; i++) {
    delay(100);
    printEncoderCount();
  }

  stopMotor();
  Serial.println("Forward stopped");

  delay(2000);

  // Reverse
  noInterrupts();
  encoderCount = 0;
  interrupts();

  Serial.println("\nREVERSE");
  reverseMotor(100);

  for (int i = 0; i < 20; i++) {
    delay(100);
    printEncoderCount();
  }

  stopMotor();
  Serial.println("Reverse stopped");

  delay(3000);
}