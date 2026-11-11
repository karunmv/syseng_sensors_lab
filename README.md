# Unified sensors and motors firmware

Open `Unified_Sensors_Motors.ino` and upload to **Arduino Uno**. This folder is
self-contained; the four teammates' original folders remain standalone references.
The ROS 2 bridge and graphical panel are the next layer; they are not implemented
by this firmware.

## Source modules

| Source | Retained behavior |
|---|---|
| `../arduino/TSP05_Motor_Control/` | Only analog ultrasonic conversion, EMA, and step/direction stepper control: 15–75 cm maps to 0–180°, 50-step target deadband, 1600 steps/rev, minimum 5000 µs between steps. |
| `../pot_servo/firmware/pot_servo_control/` | Eight-read ADC average, measured angle conversion, EMA α=0.3, 80–300° pot range mapped to −90…+100° servo range, 610–2500 µs endpoint interpolation. |
| `../dc_control_ir/position_velocity_control/` | Encoder interrupt method and 12375.12 counts/rev; position PID (3, 0.05, 0.2), ±1.5° tolerance; velocity PI (4, 3), anti-windup, deadband, ±12 RPM and original PWM limits. |
| `../dc_control_ir/ir_control/` | Ten-read average, measured distance approximation, 10–80 cm clamp, six original PWM levels, 2 cm hysteresis, 0.30/0.35 V detection hysteresis. |
| `../tof_filter/` | VL53L1X long mode, 50 ms measurement budget/continuous interval, valid samples only, normalized exponential weighting (α=0.1) over the latest ten accepted samples. No motor association. |

Integration changes: modules share one scheduler and serial interface; IR samples
are collected 5 ms apart without `delay()`; ToF readiness is polled before
`read(false)`; sensor initialization failures do not halt the motor loop; encoder
snapshots preserve interrupt state; PID output is clamped before conversion to the
Uno's 16-bit integer. Small timestamped telemetry frames drain through UART only
when space is available. There are no dynamically allocated `String` objects.
Short 10 µs direction-setup and 5 µs STEP pulses remain from the stepper code.

The original DC sketches are two distinct control paths. **GUI DC control uses
encoder feedback; IR sensor control directly applies PWM in the forward direction.**
IR mode is not an encoder velocity controller. The assignment permits this sensor
control behavior, while the GUI must demonstrate closed-loop position/velocity.
IR PWM is not constrained to the PID's limit of 180.

## Wiring and configuration

| Device | Uno connection |
|---|---|
| Encoder A / B | D2 / D3, external encoder pull-ups as in the lab instructions |
| EasyDriver STEP / DIR | D4 / D5 |
| L298 enable PWM / IN1 / IN2 | D6 / D7 / D8 |
| Servo signal | D9 |
| Pushbutton | D12 to GND; `INPUT_PULLUP`, 40 ms software debounce |
| Potentiometer wiper | A0; retain **3.3 V** potentiometer supply |
| Analog ultrasonic output | A1 |
| Sharp IR output | A2 |
| VL53L1X SDA / SCL | A4 / A5; use the actual breakout's specified supply/logic interface |

The servo moves from the old integrated sketch's D3 to the calibrated subsystem's
D9, and the button moves from D7 to D12. Servo uses Timer1, so D9/D10 cannot provide
DC PWM; D6 remains available. D0/D1 are reserved for USB serial. Use the default
5 V ADC reference; changing the reference or pot supply invalidates calibration.
Connect motor power externally with common ground, and retain the driver current
limits and encoder pull-ups required by the lab.

`Config.h` holds pins, timing, original tuning, and calibration constants. Verify
1600 step pulses per shaft revolution against the actual EasyDriver microstep
configuration, and verify encoder scale/polarity before testing PID.

## Build

Dependencies tested: Arduino AVR Boards **1.8.6** (or a compatible core with Wire
timeouts), Arduino **Servo 1.3.0**, and Pololu **VL53L1X 1.3.1**. Select the Pololu library, not a different
VL53L1X API. The integration uses its documented `dataReady()` and `read(false)`
methods: <https://github.com/pololu/vl53l1x-arduino>.

With Arduino CLI installed, from the repository root:

```sh
arduino-cli core install arduino:avr@1.8.6
arduino-cli lib install Servo@1.3.0 VL53L1X@1.3.1
arduino-cli compile --fqbn arduino:avr:uno Unified_Sensors_Motors
```

Only upload after checking the pin map and power arrangement. Desktop simulation
tests live in `tests/` and run with `bash Unified_Sensors_Motors/tests/run.sh`.
They exercise logic/protocol behavior, not motor tuning, electrical wiring, or
real sensor accuracy.

Verified Uno build: **20850/32256 bytes flash** and **1214/2048 bytes static RAM**,
leaving 834 bytes for stack/local variables. No upload or physical motor/sensor
test has been performed as part of this integration. Desktop tests cover filter
weighting, calibrated limits, step rounding/partial motion, encoder-relative
commands, IR hysteresis, ToF invalid/stale/error handling, serial validation and
framing, mode transitions, latched stop, button debounce, and UART backpressure.

## Modes and commands

115200 baud, ASCII, newline-terminated. CRLF is accepted. Commands are uppercase.
Only the laptop's serial bridge should own the port. Send one command at a time,
wait for `ACK` or `ERR`, and accept interleaved telemetry while waiting. Commands
must contain at most 47 printable characters before the newline; malformed or
overlong lines are discarded in full. Empty lines are ignored. A trailing newline
is mandatory. Do not send continuous unacknowledged command bursts.

At boot the system is in **GUI** mode: DC stopped, stepper origin defined as its
current physical position (0°), servo commanded to 0° using the original pulse
interpolation. There is no homing or absolute-position sensor on the stepper.

| Command | Effect / reply |
|---|---|
| `MODE,GUI` | Stop DC/cancel unfinished stepper movement, hold servo command, give GUI control. `ACK,MODE,GUI`. |
| `MODE,SENSOR` | Pot→servo, IR→DC PWM, ultrasonic→stepper; DC waits for a fresh IR batch. `ACK,MODE,SENSOR`. |
| `SERVO,-90` | Absolute servo angle, clamped to −90…+100°. `ACK,SERVO,<applied_deg>`. |
| `DC_POS,-30` | Relative to current encoder position; replaces the previous DC operation. ±3600° per command. `ACK,DC_POS,<requested_deg>`. |
| `DC_VEL,-5` | Signed RPM, clamped to ±12. Zero stops DC. `ACK,DC_VEL,<applied_rpm>`. |
| `STEPPER,45` | Relative to current estimated position; replaces any pending move. ±360° per command. `ACK,STEPPER,<rounded_deg>`. |
| `STOP` or `STOP,ALL` | DC output zero, stepper target=current, servo holds its last command. Enter **STOPPED**. `ACK,STOP,STOPPED`. |

Motion commands are rejected outside GUI mode. `MODE,GUI` or `MODE,SENSOR` explicitly
leaves STOPPED; a button press leaves STOPPED for GUI. Otherwise the button toggles
GUI↔SENSOR. STOP remains latched until such an explicit mode action. It is a
software motion stop, not a power-disconnect or servo torque release. Neither
serial disconnect nor lack of commands automatically stops a running motor.
Repeated requests for the already active mode do not cancel motion.

For convenience `POS,<deg>` and `VEL,<rpm>` remain aliases for the original DC
commands. `M0` means SENSOR and `M1` means GUI, matching `pot_servo` (the original
servo/stepper sketch used the reverse numbering). Prefer named MODE commands.

Numeric commands reject missing values, NaN, infinity, and trailing junk. `ERR`
reasons are `NUMBER`, `RANGE`, `COMMAND`, `LINE`, `SENSOR_MODE`, or `STOPPED`.
An absolute stepper target beyond ±100000000 steps is rejected to avoid overflow.

Example session:

```text
MODE,GUI
SERVO,-90
SERVO,100
DC_POS,30
DC_VEL,-5
DC_VEL,0
STEPPER,-30
MODE,SENSOR
STOP
```

Wait for each ACK before sending the next line, and allow a position move to finish
before issuing a replacement if the goal is to observe completion.

## Telemetry protocol v1

Boot emits `READY,UNIFIED_SENSORS_MOTORS,1`. Seven independently timestamped CSV
record types are scheduled approximately every 100 ms. Timestamps are Arduino
`millis()` values (unsigned 32-bit, wrap after about 49.7 days), not wall-clock time.
Packets are sent one at a time; they are not one atomic snapshot. Commands/replies
take priority over the next telemetry packet. Treat missing/stale packets in the
GUI explicitly. Fields always appear in the following order:

```text
SYS,ms,mode,protocol_version
POT,ms,adc_mean_truncated,pot_deg,pot_filtered_deg,servo_command_deg
IR,ms,adc_mean,volts,distance_cm,detected,requested_pwm
ULTRA,ms,adc,raw_cm,filtered_cm
TOF,ms,sample_seq,sample_ms,raw_mm,filtered_mm,valid,range_status,online
DC,ms,control_mode,position_deg,velocity_rpm,target_position_deg,target_rpm,applied_pwm
STEPPER,ms,position_steps,target_steps,position_deg,target_deg,rounded_move_deg,moved_deg,moving
```

- SYS mode is `GUI`, `SENSOR`, or `STOPPED`.
- Servo angle is a **command**, not measured shaft feedback. Preserve this label
  in the GUI. The endpoint interpolation puts a 0° command at about 1505 µs, as
  in the original code; the calibration README rounds this to 1500 µs.
- IR `detected=0` means `distance_cm=-1` and `requested_pwm=0`. Requested IR PWM
  is calculated in every mode; the DC packet reports the PWM actually applied.
- ULTRA `raw_cm` is the nominal `adc*1.27` conversion; filtered uses α=0.1.
  Neither the ultrasonic nor pot/IR ADC input can reliably distinguish a
  disconnected wire from a plausible value using this code alone.
- TOF `sample_seq` increments only for new valid filtered samples. For plots and
  filter data collection use `sample_ms`, **deduplicate sample_seq**, and require
  `valid=1`/`online=1`. `ms` is the transmission timestamp. Do not treat duplicate
  packets as new sensor samples. A nonzero range status marks an invalid reading;
  the filter is not updated. `-1` means initialization/I²C error; `-2` means no
  fresh ranging result within 500 ms (or waiting for the first result). Initial
  values are −1. After failure, retained values are historical and must not be
  presented as current measurements. A failed I²C device is disabled until reset;
  no blocking reinitialization runs during motor control. The I²C bus timeout is
  3 ms per transaction; the scheduler is cooperative rather than hard real time.
- DC mode is `IDLE`, `POS`, `VEL`, or `IR`. Target position is meaningful in POS;
  target RPM in VEL. In IR use `IR.requested_pwm` and `DC.applied_pwm`. PWM is
  signed to show direction. Position is relative to boot and remains in POS
  while holding within the original ±1.5° tolerance.
- Stepper positions are **estimates from issued pulses**, not physical feedback.
  A 1600-step revolution gives 0.225° resolution. `rounded_move_deg` is the
  achievable signed angle for the latest move; `moved_deg` is the signed pulse
  count since that move began, including partial moves and cancellation. At
  completion they match. Sensor retargeting begins a new move record. Missed
  steps/slip cannot be detected; the GUI must not label the estimate as measured.

## Assignment evidence still needed

This selection supplies four different sensors, including analog sensors, and
three distinct sensor→motor pairings for a five-person team. The existing pot
transfer plot and leave-one-out error analysis are retained in `pot_servo`.
Before the demo, collect measured error over the usable range for IR, ultrasonic,
and ToF; record/plot raw and filtered ToF side by side; test GUI control and sensor
control on all three physical motors; show the debounced button; document the
integrator's contribution/challenges and the project application. GUI/ROS work
and motor mounting/visible shaft markers remain separate deliverables.

Known original limitations retained rather than retuned:

- IR's 220-PWM band is unreachable from its initial state: distance is clamped
  to ≥10 cm but entering level zero requires `<10−2=8 cm`. The practical highest
  band is 190. The linear distance approximation still needs measured error;
  clamping does not validate accuracy or extend physical detection range.
- The ultrasonic target deadband is 50 steps = 11.25°, so small distance changes
  do not move its stepper target. Sensor-mode position is referenced to the boot
  origin, not to the position where SENSOR mode was entered.
- The original minimum velocity PWM is 55. Very low RPM commands may be physically
  unachievable at that PWM; demonstrate achievable velocities after hardware testing.
- Pot calibration sketch comments mention 5 V wiring, while its measured data,
  final controller, and transfer plot specify a 3.3 V supply. This integration
  follows the final calibrated controller and leaves those source files intact.
