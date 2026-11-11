
// Example Serial commands:
// POS,30    -> Move +30 degrees from current position
// POS,-30   -> Move -30 degrees from current position
// VEL,5     -> Run at +5 RPM
// VEL,-5    -> Run at -5 RPM
// VEL,0     -> Stop velocity mode
// STOP      -> Stop motor
//
// Serial feedback:
// STATUS,MODE,TARGET,POSITION,VELOCITY,ERROR,PWM
// =====================================================

#include <stdlib.h>
#include <string.h>
#include <math.h>

// ---------------- Motor driver pins ----------------

const int IN1 = 7;
const int IN2 = 8;
const int EN  = 6;

// ---------------- Encoder pins ----------------

const int ENCODER_A = 2;
const int ENCODER_B = 3;

volatile long encoderCount = 0;

const float COUNTS_PER_REV = 12375.12;
const float COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;

// ---------------- Operating limits ----------------

const int MAX_PWM = 180;
const int MIN_PWM = 65;

const float MAX_TARGET_RPM = 12.0;

// ---------------- Control modes ----------------

enum ControlMode {
  IDLE,
  POSITION,
  VELOCITY
};

ControlMode mode = IDLE;

// ---------------- Position PID ----------------

float Kp_pos = 3.0;
float Ki_pos = 0.05;
float Kd_pos = 0.2;

float targetAngle = 0;
float posIntegral = 0;
float previousAngle = 0;

const float POSITION_TOLERANCE = 1.5;

// ---------------- Velocity PI ----------------

float Kp_vel = 4.0;
float Ki_vel = 3.0;

const int VELOCITY_MIN_PWM = 55;
const float VELOCITY_DEADBAND = 0.10;

float targetRPM = 0;
float velIntegral = 0;

float actualRPM = 0;
long previousVelocityCount = 0;

// ---------------- Timing ----------------

unsigned long previousPositionTime = 0;
unsigned long previousVelocityTime = 0;
unsigned long lastPrintTime = 0;

const unsigned long POSITION_INTERVAL = 10;
const unsigned long VELOCITY_INTERVAL = 50;
const unsigned long PRINT_INTERVAL = 100;

// ---------------- Motor state ----------------

int currentPWM = 0;

// ---------------- Serial input ----------------

char inputBuffer[40];
byte inputIndex = 0;

// =====================================================
// ENCODER INTERRUPT
// =====================================================

void readEncoder() {

  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// =====================================================
// GET ENCODER COUNT SAFELY
// =====================================================

long getEncoderCount() {

  noInterrupts();
  long count = encoderCount;
  interrupts();

  return count;
}

// =====================================================
// GET MOTOR POSITION
// =====================================================

float getAngle() {

  return getEncoderCount() / COUNTS_PER_DEG;
}

// =====================================================
// STOP MOTOR
// =====================================================

void stopMotor() {

  analogWrite(EN, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  currentPWM = 0;
}

// =====================================================
// SET MOTOR DIRECTION AND PWM
// Positive = forward
// Negative = reverse
// =====================================================

int setMotor(int pwm) {

  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    int appliedPWM = max(pwm, MIN_PWM);

    analogWrite(EN, appliedPWM);

    currentPWM = appliedPWM;

    return appliedPWM;
  }

  else if (pwm < 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    int appliedPWM = max(-pwm, MIN_PWM);

    analogWrite(EN, appliedPWM);

    currentPWM = -appliedPWM;

    return -appliedPWM;
  }

  stopMotor();

  return 0;
}

// =====================================================
// SET MOTOR FOR VELOCITY CONTROL
// Allows PWM values starting at 55
// =====================================================

int setVelocityMotor(int pwm) {

  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    int appliedPWM = max(pwm, VELOCITY_MIN_PWM);

    analogWrite(EN, appliedPWM);

    currentPWM = appliedPWM;

    return appliedPWM;
  }

  else if (pwm < 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    int appliedPWM = max(-pwm, VELOCITY_MIN_PWM);

    analogWrite(EN, appliedPWM);

    currentPWM = -appliedPWM;

    return -appliedPWM;
  }

  stopMotor();

  return 0;
}

// =====================================================
// UPDATE VELOCITY FROM ENCODER
// =====================================================

void updateVelocity(float dt) {

  long currentCount = getEncoderCount();

  long deltaCount = currentCount - previousVelocityCount;

  actualRPM =
      (deltaCount / COUNTS_PER_REV) * (60.0 / dt);

  previousVelocityCount = currentCount;
}

// =====================================================
// RESET VELOCITY MEASUREMENT
// =====================================================

void resetVelocityMeasurement() {

  previousVelocityCount = getEncoderCount();
  actualRPM = 0;
  velIntegral = 0;

  previousVelocityTime = millis();
}

// =====================================================
// PROCESS SERIAL COMMAND
// =====================================================

void processCommand(char *command) {

  if (strncmp(command, "POS,", 4) == 0) {

    float requestedDegrees = atof(command + 4);

    // Relative movement from current position
    targetAngle = getAngle() + requestedDegrees;

    posIntegral = 0;
    previousAngle = getAngle();
    previousPositionTime = millis();

    mode = POSITION;

    Serial.println("ACK,POS");
  }

  else if (strncmp(command, "VEL,", 4) == 0) {

    targetRPM = atof(command + 4);

    targetRPM = constrain(
      targetRPM,
      -MAX_TARGET_RPM,
      MAX_TARGET_RPM
    );

    resetVelocityMeasurement();

    mode = VELOCITY;

    if (targetRPM == 0) {
      mode = IDLE;
      stopMotor();
    }

    Serial.println("ACK,VEL");
  }

  else if (strcmp(command, "STOP") == 0) {

    mode = IDLE;

    posIntegral = 0;
    velIntegral = 0;

    stopMotor();

    Serial.println("ACK,STOP");
  }

  else {

    Serial.println("ERROR,UNKNOWN_COMMAND");
  }
}

// =====================================================
// READ SERIAL WITHOUT BLOCKING CONTROL LOOP
// =====================================================

void readSerialCommands() {

  while (Serial.available() > 0) {

    char c = Serial.read();

    if (c == '\n') {

      inputBuffer[inputIndex] = '\0';

      if (inputIndex > 0) {
        processCommand(inputBuffer);
      }

      inputIndex = 0;
    }

    else if (c != '\r') {

      if (inputIndex < sizeof(inputBuffer) - 1) {
        inputBuffer[inputIndex++] = c;
      }

      else {
        inputIndex = 0;
      }
    }
  }
}

// =====================================================
// POSITION PID
// =====================================================

void runPositionPID(float dt) {

  float actualAngle = getAngle();

  float error = targetAngle - actualAngle;

  posIntegral += error * dt;

  posIntegral = constrain(
    posIntegral,
    -100.0,
    100.0
  );

  float derivative =
      (actualAngle - previousAngle) / dt;

  float output =
      Kp_pos * error
    + Ki_pos * posIntegral
    - Kd_pos * derivative;

  previousAngle = actualAngle;

  if (fabs(error) <= POSITION_TOLERANCE) {

    stopMotor();
    posIntegral = 0;
  }

  else {

    setMotor((int)output);
  }
}

// =====================================================
// VELOCITY PI CONTROLLER
// =====================================================

void runVelocityPID(float dt) {

  float direction = (targetRPM >= 0) ? 1.0 : -1.0;

  float directionalRPM = direction * actualRPM;

  float desiredRPM = fabs(targetRPM);

  float error = desiredRPM - directionalRPM;

  if (fabs(error) < VELOCITY_DEADBAND) {
    error = 0;
  }

  // Update integral
  float proposedIntegral = velIntegral + error * dt;

  proposedIntegral = constrain(
    proposedIntegral,
    -20.0,
    20.0
  );

  // PI output
  float output =
      VELOCITY_MIN_PWM
    + Kp_vel * error
    + Ki_vel * proposedIntegral;

  // Anti-windup
  if (!((output > MAX_PWM && error > 0) ||
        (output < VELOCITY_MIN_PWM && error < 0))) {

    velIntegral = proposedIntegral;
  }

  // Recalculate output
  output =
      VELOCITY_MIN_PWM
    + Kp_vel * error
    + Ki_vel * velIntegral;

  output = constrain(
    output,
    (float)VELOCITY_MIN_PWM,
    (float)MAX_PWM
  );

  int pwm = (int)output;

  // Apply direction and PWM
  setVelocityMotor((int)(direction * pwm));
}

// =====================================================
// SEND FEEDBACK TO GUI
// =====================================================

void sendStatus() {

  float actualAngle = getAngle();

  float error = 0;

  const char *modeName = "IDLE";
  float target = 0;

  if (mode == POSITION) {

    modeName = "POS";
    target = targetAngle;

    error = targetAngle - actualAngle;
  }

  else if (mode == VELOCITY) {

    modeName = "VEL";
    target = targetRPM;

    error = targetRPM - actualRPM;
  }

  Serial.print("STATUS,");
  Serial.print(modeName);

  Serial.print(",");
  Serial.print(target, 2);

  Serial.print(",");
  Serial.print(actualAngle, 2);

  Serial.print(",");
  Serial.print(actualRPM, 2);

  Serial.print(",");
  Serial.print(error, 2);

  Serial.print(",");
  Serial.println(currentPWM);
}

// =====================================================
// SETUP
// =====================================================

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

  unsigned long now = millis();

  previousPositionTime = now;
  previousVelocityTime = now;
  lastPrintTime = now;

  previousAngle = getAngle();
  resetVelocityMeasurement();

  Serial.println("READY,DC_MOTOR_CONTROLLER");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  readSerialCommands();

  unsigned long currentTime = millis();

  // Position controller
  if (mode == POSITION &&
      currentTime - previousPositionTime >= POSITION_INTERVAL) {

    float dt =
      (currentTime - previousPositionTime) / 1000.0;

    previousPositionTime = currentTime;

    runPositionPID(dt);
  }

  // Velocity controller
  if (currentTime - previousVelocityTime >= VELOCITY_INTERVAL) {

    float dt =
      (currentTime - previousVelocityTime) / 1000.0;

    previousVelocityTime = currentTime;

    updateVelocity(dt);

    if (mode == VELOCITY) {
      runVelocityPID(dt);
    }
  }

  // Feedback every 100 ms
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {

    lastPrintTime = currentTime;

    sendStatus();
  }
}