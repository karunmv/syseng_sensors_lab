#include "DcMotor.h"
#include "Config.h"
#include <math.h>
#include <util/atomic.h>

static volatile long encoderCount = 0;

static void readEncoder() {
  if (digitalRead(Config::ENCODER_A) == digitalRead(Config::ENCODER_B)) {
    ++encoderCount;
  } else {
    --encoderCount;
  }
}

void DcMotor::begin() {
  pinMode(Config::DC_IN1, OUTPUT);
  pinMode(Config::DC_IN2, OUTPUT);
  pinMode(Config::DC_PWM_PIN, OUTPUT);
  // Retain INPUT: use the kit encoder's external pull-ups as specified in the lab.
  pinMode(Config::ENCODER_A, INPUT);
  pinMode(Config::ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Config::ENCODER_A), readEncoder, CHANGE);
  stop();
  previousPositionTime = millis();
  previousAngle = angle();
  resetVelocity();
}

long DcMotor::getCount() const {
  long value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = encoderCount; }
  return value;
}

float DcMotor::angle() const {
  return getCount() / Config::DC_COUNTS_PER_DEG;
}

void DcMotor::stop() {
  analogWrite(Config::DC_PWM_PIN, 0);
  digitalWrite(Config::DC_IN1, LOW);
  digitalWrite(Config::DC_IN2, LOW);
  pwm = 0;
  mode = IDLE;
  targetRPM = 0;
  posIntegral = 0;
  velIntegral = 0;
}

void DcMotor::drive(int value, int minimum, int maximum) {
  value = constrain(value, -maximum, maximum);
  if (value == 0) {
    analogWrite(Config::DC_PWM_PIN, 0);
    digitalWrite(Config::DC_IN1, LOW);
    digitalWrite(Config::DC_IN2, LOW);
    pwm = 0;
    return;
  }
  digitalWrite(Config::DC_IN1, value > 0 ? HIGH : LOW);
  digitalWrite(Config::DC_IN2, value > 0 ? LOW : HIGH);
  int magnitude = max(abs(value), minimum);
  analogWrite(Config::DC_PWM_PIN, magnitude);
  pwm = value > 0 ? magnitude : -magnitude;
}

void DcMotor::resetVelocity() {
  previousVelocityCount = getCount();
  actualRPM = 0;
  velIntegral = 0;
  previousVelocityTime = millis();
}

void DcMotor::moveRelative(float value) {
  stop();
  previousAngle = angle();
  targetAngle = previousAngle + value;
  previousPositionTime = millis();
  mode = POSITION;
}

void DcMotor::setVelocity(float value) {
  stop();
  targetRPM = constrain(value, -Config::DC_MAX_RPM, Config::DC_MAX_RPM);
  resetVelocity();
  if (targetRPM != 0) mode = VELOCITY;
}

void DcMotor::setIrPwm(int value) {
  if (mode != IR) stop();
  mode = IR;
  // Original IR mode is open-loop, forward only, up to PWM 220 (not PID's 180).
  drive(value, 0, 255);
}

void DcMotor::positionPID(float dt) {
  float actualAngle = angle();
  float error = targetAngle - actualAngle;
  posIntegral = constrain(posIntegral + error * dt, -100.0f, 100.0f);
  float derivative = (actualAngle - previousAngle) / dt;
  float output = Config::DC_KP_POS * error + Config::DC_KI_POS * posIntegral
                 - Config::DC_KD_POS * derivative;
  previousAngle = actualAngle;
  if (fabs(error) <= Config::DC_POSITION_TOLERANCE) {
    drive(0, 0, Config::DC_MAX_PWM);
    posIntegral = 0;
  } else {
    // Clamp before the AVR's 16-bit int conversion; retain original PID tuning.
    drive((int)constrain(output, -(float)Config::DC_MAX_PWM,
                        (float)Config::DC_MAX_PWM), Config::DC_MIN_PWM, Config::DC_MAX_PWM);
  }
}

void DcMotor::velocityPI(float dt) {
  float direction = targetRPM >= 0 ? 1.0f : -1.0f;
  float error = fabs(targetRPM) - direction * actualRPM;
  if (fabs(error) < Config::DC_VELOCITY_DEADBAND) error = 0;
  float proposed = constrain(velIntegral + error * dt, -20.0f, 20.0f);
  float output = Config::DC_VELOCITY_MIN_PWM + Config::DC_KP_VEL * error
                 + Config::DC_KI_VEL * proposed;
  if (!((output > Config::DC_MAX_PWM && error > 0) ||
        (output < Config::DC_VELOCITY_MIN_PWM && error < 0))) {
    velIntegral = proposed;
  }
  output = Config::DC_VELOCITY_MIN_PWM + Config::DC_KP_VEL * error
           + Config::DC_KI_VEL * velIntegral;
  output = constrain(output, (float)Config::DC_VELOCITY_MIN_PWM, (float)Config::DC_MAX_PWM);
  drive((int)(direction * (int)output), Config::DC_VELOCITY_MIN_PWM, Config::DC_MAX_PWM);
}

void DcMotor::update() {
  unsigned long now = millis();
  if (mode == POSITION && now - previousPositionTime >= Config::DC_POSITION_INTERVAL_MS) {
    float dt = (now - previousPositionTime) / 1000.0f;
    previousPositionTime = now;
    positionPID(dt);
  }
  if (now - previousVelocityTime >= Config::DC_VELOCITY_INTERVAL_MS) {
    float dt = (now - previousVelocityTime) / 1000.0f;
    previousVelocityTime = now;
    long current = getCount();
    actualRPM = ((current - previousVelocityCount) / Config::DC_COUNTS_PER_REV) * (60.0f / dt);
    previousVelocityCount = current;
    if (mode == VELOCITY) velocityPI(dt);
  }
}

bool IrSensor::update() {
  unsigned long now = millis();
  if (!sampling) {
    if (now - batchStarted < Config::IR_BATCH_INTERVAL_MS) return false;
    batchStarted = now;
    lastSample = now - Config::IR_SAMPLE_INTERVAL_MS;
    sum = 0;
    count = 0;
    sampling = true;
  }
  if (now - lastSample < Config::IR_SAMPLE_INTERVAL_MS) return false;
  lastSample = now;
  sum += analogRead(Config::IR_PIN);
  if (++count < Config::IR_SAMPLES) return false;
  sampling = false;
  adc = sum / (float)Config::IR_SAMPLES;
  volts = adc * (5.0f / 1023.0f);
  if (!detected && volts > 0.35f) detected = true;
  else if (detected && volts < 0.30f) detected = false;
  requestedPwm = 0;
  cm = -1;
  if (detected) {
    cm = constrain(-23.4382f * volts + 76.6889f, 10.0f, 80.0f);
    const float thresholds[] = {10, 20, 30, 40, 50};
    const int levels[] = {220, 190, 155, 115, 80, 55};
    while (level > 0 && cm < thresholds[level - 1] - 2.0f) --level;
    while (level < 5 && cm > thresholds[level] + 2.0f) ++level;
    requestedPwm = levels[level];
  }
  return true;
}
