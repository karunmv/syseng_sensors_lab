# ROS 2 serial bridge and rqt dashboard

ROS 2 Humble package for the protocol-v1 firmware in `../Unified_Sensors_Motors/`.
The dashboard publishes ROS commands; only `serial_bridge` opens the Uno's serial
port. The GUI and serial I/O run in separate processes. rqt owns its ROS executor;
callbacks pass data to widgets using queued Qt signals.

## Build and start

From the repository root, in a ROS 2 Humble terminal:

```sh
source /opt/ros/humble/setup.bash
colcon build --base-paths sensor_gui --symlink-install
source install/setup.bash

# Develop/test without an Uno. All data is prominently marked SIMULATED.
ros2 launch sensor_gui lab.launch.py simulate:=true
```

Use another terminal for actual hardware, after stopping the simulated launch:

```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sensor_gui lab.launch.py port:=/dev/ttyACM0
```

Upload the unified sketch first. Opening serial commonly resets an Uno; the
bridge waits two seconds and requires valid protocol-v1 SYS telemetry before
accepting commands. A persistent `/dev/serial/by-id/...` path is preferable when
available. Close Arduino Serial Monitor/Plotter before connecting. Use only one
bridge for a topic prefix; do not run simulation and hardware on `/lab` together.

The launch starts both the bridge and the rqt plugin. Closing that dashboard
shuts down this launch and the bridge. Bridge shutdown sends a best-effort STOP,
but does not wait for an acknowledgment. Use STOP ALL and confirm STOPPED before
closing when motors are running. Unloading a panel from a separately started rqt
does not stop an independently running bridge.

Dependencies are declared in `package.xml`. If they are missing on Ubuntu 22.04:

```sh
sudo apt install python3-colcon-common-extensions python3-serial python3-matplotlib \
  ros-humble-rqt ros-humble-rqt-gui-py ros-humble-python-qt-binding
```

To run components separately:

```sh
ros2 launch sensor_gui lab.launch.py simulate:=true gui:=false
# Another sourced terminal:
ros2 run rqt_gui rqt_gui --standalone sensor_gui/SensorsMotors
```

For a different topic prefix, pass `topic_prefix:=/team_lab` to the launch. A
separately opened panel accepts `--args --topic-prefix /team_lab`.

## Dashboard

- **Control and sensors:** absolute servo angle and measured endpoint buttons;
  signed DC relative degree moves and signed velocity; signed stepper degree
  moves; sensor quantities/validity and motor feedback.
- **GUI control / Sensor control:** firmware mode actions. Motion buttons require
  a live bridge, confirmed GUI mode, and no outstanding command.
- **STOP ALL:** clears queued motion and requests the firmware's latched STOPPED
  mode. If a command is awaiting ACK, STOP follows that reply. A missing ACK
  times out after 0.75 seconds and the link is closed; delivery of STOP is then
  not guaranteed. This is a software command, not a physical emergency stop.
- **Live plots and capture:** ToF raw/filtered distances in mm and DC measured/
  target RPM. The target line is populated only in VEL mode, because IR mode is
  direct PWM and POS mode has a position rather than velocity target.
- **CSV export:** retains up to 36,000 valid ToF sample pairs per panel lifetime,
  deduplicated by firmware sample identity. Rows include session and simulated
  flags. Reconnects begin a new plot segment but retain labelled capture rows.
  Select hardware rows (`simulated=False`) for lab evidence; simulated data is
  only for interface development. The visible plot uses the latest 600 pairs
  from the current session (about 60 seconds at 10 Hz).
- **Commands and events:** queued/sent/acknowledged/rejected/error outcomes. An
  acknowledgment means the command was accepted, not that a move completed.
  Observe DC position/error tolerance or stepper `moving`/`moved_deg` for completion.

Displayed servo angle is commanded, not measured. Stepper position/moved angle
are estimates from pulses, not encoder feedback. DC position/velocity use the
encoder. Sensor/table data expires after one second without fresh records; the
panel disables motion when bridge status is stale. Old plots remain historical
and are labelled disconnected/stale rather than replaced with invented zeros.

## ROS interface

Default prefix: `/lab`. Standard messages avoid a custom interface build step.

| Topic/service | Type | Purpose |
|---|---|---|
| `/lab/command` | `std_msgs/msg/String` | Uppercase firmware command, without a trailing newline. Volatile, reliable. |
| `/lab/command/result` | `std_msgs/msg/String` | JSON with `session`, `command`, `outcome`, `detail`. |
| `/lab/bridge/status` | `std_msgs/msg/String` | Transient-local JSON connection/mode/simulation/queue/error status, 4 Hz. |
| `/lab/telemetry` | `std_msgs/msg/String` | JSON record with `kind`, device `ms`, named `data`, `session`, `simulated`, ROS `received_ns`. |
| `/lab/<kind>/<field>` | `Float64`, `Bool`, or `String` | One typed scalar topic per field in `protocol.SCHEMAS`, e.g. `/lab/dc/velocity_rpm`. |
| `/lab/reconnect` | `std_srvs/srv/Trigger` | Close/reopen device, discard old commands, require fresh handshake. |

Scalar numeric fields use `Float64`, flags use `Bool`, modes use `String`.
Examples useful for other ROS tools:

```text
/lab/pot/angle_deg              /lab/pot/filtered_deg
/lab/pot/servo_command_deg
/lab/ir/distance_cm             /lab/ir/detected
/lab/ultra/raw_cm               /lab/ultra/filtered_cm
/lab/tof/raw_mm                 /lab/tof/filtered_mm
/lab/tof/valid                  /lab/tof/online
/lab/dc/position_deg            /lab/dc/velocity_rpm
/lab/dc/target_rpm              /lab/dc/applied_pwm
/lab/stepper/position_deg       /lab/stepper/moved_deg
/lab/stepper/rounded_move_deg   /lab/stepper/moving
/lab/sys/mode
```

Scalar ToF raw/filtered topics publish only new valid online pairs; invalid
readings still update validity and JSON telemetry. An invalid IR reading does
not publish a fake distance. Consumers of scalar topics must also monitor
validity and connection status: old scalar values are not zeroed on failure.
The JSON telemetry preserves invalid statuses and the firmware's sentinel values.

Command examples (no hardware should be moving unexpectedly during experiments):

```sh
ros2 topic pub --once /lab/command std_msgs/msg/String '{data: "SERVO,0"}'
ros2 topic pub --once /lab/command std_msgs/msg/String '{data: "DC_POS,-30"}'
ros2 topic pub --once /lab/command std_msgs/msg/String '{data: "DC_VEL,-5"}'
ros2 topic pub --once /lab/command std_msgs/msg/String '{data: "STEPPER,45"}'
ros2 topic pub --once /lab/command std_msgs/msg/String '{data: "STOP"}'
ros2 topic echo /lab/command/result
ros2 topic echo /lab/bridge/status
ros2 service call /lab/reconnect std_srvs/srv/Trigger '{}'
```

The bridge checks numeric bounds before transmission rather than relying on
firmware clamping. It accepts the firmware's `POS`/`VEL`, `M0`/`M1`, and `STOP,ALL`
aliases, then sends canonical commands. Commands are queued (maximum eight
waiting), with only one awaiting firmware ACK/ERR. Non-STOP queued commands expire
after one second. STOP clears waiting commands. Numeric ranges match the firmware:
servo −90…100°, DC relative ±3600°, DC velocity ±12 RPM, stepper relative ±360°.

The protocol has no transaction IDs, so loss of an ACK makes a command's outcome
uncertain. No commands are automatically retried/replayed. ACK mismatch, timeout,
serial write/read failure, or stale SYS disconnects the link and clears pending
commands. The bridge retries opening every two seconds, allowing recovery after
USB unplug/replug. Reopening can reset the Uno; servo centering and position
origin reset follow firmware startup behavior. Disconnecting USB cannot guarantee
that a previously powered motor stops. The physical stop/power arrangements
remain separate from GUI software.

## Bridge parameters

| Parameter | Default |
|---|---|
| `port` | `/dev/ttyACM0` |
| `baud` | `115200` |
| `simulate` | `false` |
| `topic_prefix` | `/lab` |
| `boot_wait_s` | `2.0` (simulator uses 0.1 s) |
| `ack_timeout_s` | `0.75` |
| `telemetry_timeout_s` | `1.5` |
| `reconnect_interval_s` | `2.0` |

Advanced parameters can be passed when running the bridge executable directly,
for example `ros2 run sensor_gui serial_bridge --ros-args -p simulate:=true`.

## Testing and troubleshooting

From the repository root:

```sh
bash sensor_gui/tests/run_checks.sh
```

This sources Humble, isolates ROS tests on localhost/domain 71, and runs Qt
offscreen. Tests cover protocol parsing, malformed/partial serial lines, bounds,
ACK serialization, stop priority, no replay after timeout, simulated ROS↔rqt
commands, capture validity/deduplication/session labels, stale-state controls,
and a pseudo-terminal using the real PySerial connection path. Physical hardware
is not required. Test screenshots are written to `/tmp/sensors-dashboard-*.png`.

If the plugin does not appear, source `install/setup.bash` in the launching
terminal and try `rqt --force-discover`. If Qt cannot open a display, run on a
desktop session or configure WSLg/X forwarding. Offscreen mode is useful for
tests but does not display an interactive window. Use the ROS system Python;
an unrelated virtual environment may not contain `rclpy`/Qt.

For a serial permission error, verify the actual device path and your machine's
serial-port access configuration. For an ACK timeout, inspect the Events tab,
SYS/bridge status, baud rate, and whether the unified sketch is running. Do not
blindly repeat a relative command with an uncertain outcome. For a ToF plot with
no samples, inspect ToF `online`, `valid`, and `range_status` first.

The simulated motor responses are illustrative, not a model for tuning PID,
validating wiring, or establishing sensor conversion error. Hardware verification
and the assignment's remaining measurements are listed in the repository README.
