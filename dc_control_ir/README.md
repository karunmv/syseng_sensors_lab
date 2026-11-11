# DC Motor Control

This folder contains the DC motor experiments and control implementation for the Sensors and Motors lab. The work was done in stages: first testing the individual components, then implementing position and velocity control using encoder feedback, and finally integrating the IR sensor with the motor.

## Folder Structure

```text
dc_control_ir/
├── component_tests/
│   ├── dc_motor_test/
│   │   └── dc_motor_test.ino
│   ├── encoder_test/
│   │   └── encoder_test.ino
│   └── ir_sensor_test/
│       └── ir_sensor_raw_test.ino
│
├── position_velocity_control/
│   └── position_control.ino
│
└── ir_dc_motor_control/
    └── ir_dc_motor_control.ino
```

## 1. Individual Component Tests

Before combining everything, we tested the motor, encoder, and IR sensor separately.

- **DC motor:** Tested forward and reverse rotation using the L298 motor driver and PWM. This verified motor direction and speed control.
- **Encoder:** Ran the motor in both directions and monitored encoder counts. This verified that the encoder was detecting rotation and that the count changed with motor direction.
- **IR sensor:** Read the raw ADC values and converted them into voltage. This helped us inspect the sensor output independently of the distance conversion and motor-control logic.

Keeping these tests separate made it easier to identify hardware and sensor issues before integrating the components.

## 2. Position and Velocity Control

The motor was then controlled using encoder feedback.

Two control modes were implemented:

- **Position control (PID):** The motor moves by a requested relative angle. The encoder provides the current position, and the PID controller uses the position error to determine the motor direction and PWM.
- **Velocity control (PI):** The motor runs at a requested RPM. Encoder counts are measured over time to estimate actual RPM, and the PI controller adjusts PWM to reduce the difference between target and measured velocity.

The commands are sent over the serial interface:

| Command | Function |
|---|---|
| `POS,30` | Move 30 degrees forward |
| `POS,-30` | Move 30 degrees backward |
| `VEL,5` | Run at +5 RPM |
| `VEL,-5` | Run at -5 RPM |
| `VEL,0` | Stop velocity mode |
| `STOP` | Stop the motor |

The controller also sends periodic status feedback over serial, including the current mode, target, measured position, velocity, error, and applied PWM. This can be used by a GUI to monitor the motor.

## 3. IR Sensor-Based Motor Control

The final part integrates the Sharp IR sensor with the DC motor. The idea is simple: **the motor runs faster when an object is closer and slows down as the object moves farther away.** If no valid object is detected, the motor stops.

### IR Sensor Calibration and the Issue We Encountered

One challenge was converting the IR sensor's voltage directly into distance. The Sharp IR sensor does not have a linear voltage-to-distance response over its entire operating range, so assuming a simple relationship without checking the actual sensor readings would give inaccurate distance estimates.

We first inspected the raw ADC readings and voltage independently. We then placed objects at known distances, recorded the corresponding sensor voltages manually, and used these measurements to obtain an approximate linear relationship for the selected operating range.

The resulting approximation is:

```cpp
distanceCm = -23.4382 * voltage + 76.6889;
```

We use this as an approximation rather than assuming it represents the sensor's exact response at every distance. The estimated distance is also clamped to the usable range of **10–80 cm**.

This gave us a practical distance estimate for the motor-control task without relying on an assumed sensor model or an automated calibration script.

### Motor Speed Control

The estimated distance is mapped to discrete PWM levels:

| Distance | PWM |
|---|---:|
| 10 cm (closest) | 220 |
| 10–20 cm | 190 |
| 20–30 cm | 155 |
| 30–40 cm | 115 |
| 40–50 cm | 80 |
| 50–80 cm (farthest) | 55 |

The controller uses distance thresholds with a 2 cm hysteresis to avoid rapid switching between PWM levels when the measured distance fluctuates near a threshold.

### No-Detection Handling

Another important part was handling cases where the sensor does not detect a valid target. Instead of treating every low voltage as a meaningful distance, we introduced separate stop and resume voltage thresholds:

- Below **0.30 V:** Stop the motor.
- Above **0.35 V:** Resume detection and motor control.

Using separate thresholds prevents the motor from repeatedly switching between stopped and running states when the voltage is close to the detection boundary.

The motor PWM is set to zero when no valid target is detected.