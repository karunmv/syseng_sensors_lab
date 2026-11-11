#include "Config.h"
#include "PotServo.h"
#include "UltrasonicStepper.h"
#include "DcMotor.h"
#include "TofSensor.h"
#include "Transport.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

enum SystemMode { GUI_MODE, SENSOR_MODE, STOPPED_MODE };
static SystemMode systemMode = GUI_MODE;
static PotServo pot;
static UltrasonicStepper stepper;
static DcMotor dc;
static IrSensor ir;
static TofSensor tof;
static Transport output;

static char input[48];
static byte inputLength = 0;
static bool discardInput = false;
static unsigned long lastTelemetry = 0;
static byte telemetryPart = 0;
static bool telemetryDue = false;

static const __FlashStringHelper *modeName() {
  if (systemMode == SENSOR_MODE) return F("SENSOR");
  if (systemMode == STOPPED_MODE) return F("STOPPED");
  return F("GUI");
}

static void changeMode(SystemMode next) {
  if (systemMode == next) return;
  dc.stop();
  stepper.hold();
  // Servo holds its last command; GUI takes over without a jump to an old target.
  systemMode = next;
  if (next == SENSOR_MODE) dc.setIrPwm(0); // wait for the next fresh IR batch
}

static void stopAll() {
  dc.stop();
  stepper.hold();
  systemMode = STOPPED_MODE;
}

static void updateButton() {
  static bool lastReading = HIGH;
  static bool stable = HIGH;
  static unsigned long changedAt = 0;
  bool reading = digitalRead(Config::BUTTON_PIN);
  unsigned long now = millis();
  if (reading != lastReading) changedAt = now;
  if (now - changedAt >= Config::DEBOUNCE_MS && reading != stable) {
    stable = reading;
    if (stable == LOW) changeMode(systemMode == GUI_MODE ? SENSOR_MODE : GUI_MODE);
  }
  lastReading = reading;
}

static void error(const __FlashStringHelper *reason) {
  output.print(F("ERR,"));
  output.println(reason);
}

static void acknowledge(const __FlashStringHelper *command, float applied) {
  output.print(F("ACK,"));
  output.print(command);
  output.print(',');
  output.println(applied, 3);
}

static bool parseNumber(const char *text, float &value) {
  char *end;
  value = strtod(text, &end);
  return end != text && *end == '\0' && isfinite(value);
}

static void executeCommand(char *line) {
  if (strcmp(line, "STOP") == 0 || strcmp(line, "STOP,ALL") == 0) {
    stopAll();
    output.println(F("ACK,STOP,STOPPED"));
    return;
  }
  if (strcmp(line, "MODE,GUI") == 0 || strcmp(line, "M1") == 0) {
    changeMode(GUI_MODE);
    output.println(F("ACK,MODE,GUI"));
    return;
  }
  if (strcmp(line, "MODE,SENSOR") == 0 || strcmp(line, "M0") == 0) {
    changeMode(SENSOR_MODE);
    output.println(F("ACK,MODE,SENSOR"));
    return;
  }
  if (systemMode != GUI_MODE) {
    error(systemMode == SENSOR_MODE ? F("SENSOR_MODE") : F("STOPPED"));
    return;
  }

  char *argument = strchr(line, ',');
  if (!argument) { error(F("COMMAND")); return; }
  *argument++ = '\0';
  float value;
  if (!parseNumber(argument, value)) { error(F("NUMBER")); return; }
  if (strcmp(line, "SERVO") == 0) {
    pot.setAngle(value);
    acknowledge(F("SERVO"), pot.commandDegrees);
  } else if (strcmp(line, "DC_POS") == 0 || strcmp(line, "POS") == 0) {
    if (fabs(value) > Config::DC_MAX_MOVE_DEG) { error(F("RANGE")); return; }
    dc.moveRelative(value);
    acknowledge(F("DC_POS"), value);
  } else if (strcmp(line, "DC_VEL") == 0 || strcmp(line, "VEL") == 0) {
    dc.setVelocity(value);
    acknowledge(F("DC_VEL"), dc.targetRPM);
  } else if (strcmp(line, "STEPPER") == 0) {
    if (fabs(value) > Config::STEPPER_MAX_MOVE_DEG || !stepper.moveRelative(value)) {
      error(F("RANGE"));
      return;
    }
    acknowledge(F("STEPPER"), stepper.roundedMoveDegrees);
  } else {
    error(F("COMMAND"));
  }
}

static void readSerial() {
  // One command/reply at a time; the laptop must wait for ACK/ERR before sending
  // the next command. Limit parsing work so a stream cannot starve motor updates.
  if (output.busy()) return;
  for (byte budget = 0; budget < 48 && Serial.available(); ++budget) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (discardInput) {
        discardInput = false;
        inputLength = 0;
        error(F("LINE"));
        return;
      }
      if (inputLength) {
        input[inputLength] = '\0';
        executeCommand(input);
        inputLength = 0;
        return;
      }
    } else if (!discardInput) {
      if (c < 32 || c > 126 || inputLength >= sizeof(input) - 1) {
        discardInput = true; // discard the entire line, including its suffix
        inputLength = 0;
      } else {
        input[inputLength++] = c;
      }
    }
  }
}

static void prefix(const __FlashStringHelper *kind, unsigned long now) {
  output.print(kind);
  output.print(',');
  output.print(now);
  output.print(',');
}

static void reportTelemetry() {
  unsigned long now = millis();
  if (!telemetryDue && now - lastTelemetry >= Config::TELEMETRY_MS) {
    lastTelemetry = now;
    telemetryPart = 0;
    telemetryDue = true;
  }
  if (!telemetryDue || output.busy()) return;
  // Separate small timestamped packets, instead of one long blocking println.
  switch (telemetryPart++) {
    case 0:
      prefix(F("SYS"), now);
      output.print(modeName());
      output.println(F(",1")); // protocol version
      break;
    case 1:
      prefix(F("POT"), now);
      output.print(pot.adc); output.print(',');
      output.print(pot.degrees, 2); output.print(',');
      output.print(pot.filteredDegrees, 2); output.print(',');
      output.println(pot.commandDegrees, 2);
      break;
    case 2:
      prefix(F("IR"), now);
      output.print(ir.adc, 2); output.print(',');
      output.print(ir.volts, 3); output.print(',');
      output.print(ir.cm, 2); output.print(',');
      output.print(ir.detected ? 1 : 0); output.print(',');
      output.println(ir.requestedPwm);
      break;
    case 3:
      prefix(F("ULTRA"), now);
      output.print(stepper.adc); output.print(',');
      output.print(stepper.rawCm, 2); output.print(',');
      output.println(stepper.filteredCm, 2);
      break;
    case 4:
      prefix(F("TOF"), now);
      output.print(tof.sequence); output.print(',');
      output.print(tof.sampleMs); output.print(',');
      output.print(tof.rawMm); output.print(',');
      output.print(tof.filteredMm, 2); output.print(',');
      output.print(tof.valid ? 1 : 0); output.print(',');
      output.print(tof.status); output.print(',');
      output.println(tof.online ? 1 : 0);
      break;
    case 5:
      prefix(F("DC"), now);
      if (dc.mode == DcMotor::POSITION) output.print(F("POS"));
      else if (dc.mode == DcMotor::VELOCITY) output.print(F("VEL"));
      else if (dc.mode == DcMotor::IR) output.print(F("IR"));
      else output.print(F("IDLE"));
      output.print(','); output.print(dc.angle(), 2);
      output.print(','); output.print(dc.actualRPM, 2);
      output.print(','); output.print(dc.targetAngle, 2);
      output.print(','); output.print(dc.targetRPM, 2);
      output.print(','); output.println(dc.pwm);
      break;
    case 6:
      prefix(F("STEPPER"), now);
      output.print(stepper.position); output.print(',');
      output.print(stepper.target); output.print(',');
      output.print(UltrasonicStepper::stepsToDegrees(stepper.position), 3); output.print(',');
      output.print(UltrasonicStepper::stepsToDegrees(stepper.target), 3); output.print(',');
      output.print(stepper.roundedMoveDegrees, 3); output.print(',');
      output.print(stepper.movedDegrees(), 3); output.print(',');
      output.println(stepper.position != stepper.target ? 1 : 0);
      telemetryDue = false;
      break;
  }
}

void setup() {
  Serial.begin(Config::BAUD);
  pinMode(Config::BUTTON_PIN, INPUT_PULLUP);
  dc.begin();
  stepper.begin();
  pot.begin();
  tof.begin(); // initialization may wait; motors have no pending moves yet
  output.println(F("READY,UNIFIED_SENSORS_MOTORS,1"));
}

void loop() {
  output.pump();
  updateButton();
  readSerial();
  dc.update();
  stepper.updateMotor();
  pot.update(systemMode == SENSOR_MODE);
  stepper.updateSensor(systemMode == SENSOR_MODE);
  if (ir.update() && systemMode == SENSOR_MODE) dc.setIrPwm(ir.requestedPwm);
  stepper.updateMotor();
  dc.update();
  tof.update();
  stepper.updateMotor();
  dc.update();
  reportTelemetry();
  output.pump();
}
