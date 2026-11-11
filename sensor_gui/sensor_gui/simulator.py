"""Synthetic serial device for development; never represents calibration evidence."""
import math
import random
import time
from .protocol import ProtocolError, validate_command


class SimulatorTransport:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.started = self.previous = clock()
        self.last_report = -1.0
        self.buffer = bytearray(b"READY,UNIFIED_SENSORS_MOTORS,1\r\n")
        self.mode = "GUI"
        self.servo = 0.0
        self.dc_mode = "IDLE"
        self.dc_position = self.dc_rpm = self.dc_target = self.target_rpm = 0.0
        self.step_position = self.step_target = self.step_start = 0
        self.rounded_move = 0.0
        self.sequence = 0
        self.rng = random.Random(650)
        self.tof_history = []
        self.closed = False

    def _line(self, *fields):
        self.buffer.extend((",".join(str(value) for value in fields) + "\r\n").encode("ascii"))

    def _advance(self):
        now = self.clock()
        elapsed = now - self.started
        dt = max(0.0, now - self.previous)
        self.previous = now
        pot = 190 + 105 * math.sin(elapsed * 0.25)
        ir_cm = 40 + 25 * math.sin(elapsed * 0.35)
        ultra = 45 + 28 * math.sin(elapsed * 0.2)
        if self.mode == "SENSOR":
            self.servo = -90 + (pot - 80) * 190 / 220
            self.dc_mode = "IR"
            self.target_rpm = (80 - ir_cm) * 0.12
            target = round((ultra - 15) / 60 * 800)
            if abs(target - self.step_target) >= 50:
                self.step_start = self.step_position
                self.step_target = target
                self.rounded_move = (target - self.step_position) * 0.225
        if self.dc_mode == "POS":
            error = self.dc_target - self.dc_position
            self.dc_rpm = max(-8, min(8, error * 0.2)) if abs(error) > 1.5 else 0
        elif self.dc_mode in ("VEL", "IR"):
            self.dc_rpm += (self.target_rpm - self.dc_rpm) * min(1, dt * 5)
        else:
            self.dc_rpm = 0
        self.dc_position += self.dc_rpm * 6 * dt
        delta = self.step_target - self.step_position
        self.step_position += int(math.copysign(min(abs(delta), int(dt * 200)), delta)) if delta else 0
        if now - self.last_report < 0.1:
            return
        self.last_report = now
        ms = int(elapsed * 1000) % (2**32)
        raw_tof = int(800 + 120 * math.sin(elapsed * 0.4) + self.rng.gauss(0, 18))
        self.tof_history = (self.tof_history + [raw_tof])[-10:]
        weights = [0.9 ** i for i in range(len(self.tof_history))]
        filtered = sum(value * weight for value, weight in
                       zip(reversed(self.tof_history), weights)) / sum(weights)
        self.sequence += 1
        self._line("SYS", ms, self.mode, 1)
        self._line("POT", ms, int((pot - 56.877) / 0.37486), round(pot, 2),
                   round(pot, 2), round(self.servo, 2))
        volts = (76.6889 - ir_cm) / 23.4382
        ir_pwm = 190 if ir_cm < 20 else 155 if ir_cm < 30 else 115 if ir_cm < 40 else 80 if ir_cm < 50 else 55
        self._line("IR", ms, round(volts * 1023 / 5, 2), round(volts, 3), round(ir_cm, 2), 1, ir_pwm)
        self._line("ULTRA", ms, round(ultra / 1.27), round(ultra, 2), round(ultra, 2))
        self._line("TOF", ms, self.sequence, ms, raw_tof, round(filtered, 2), 1, 0, 1)
        pwm = ir_pwm if self.dc_mode == "IR" else int(math.copysign(80, self.dc_rpm)) if self.dc_rpm else 0
        self._line("DC", ms, self.dc_mode, round(self.dc_position, 2), round(self.dc_rpm, 2),
                   round(self.dc_target, 2), round(self.target_rpm, 2), pwm)
        self._line("STEPPER", ms, self.step_position, self.step_target,
                   round(self.step_position * 0.225, 3), round(self.step_target * 0.225, 3),
                   round(self.rounded_move, 3), round((self.step_position - self.step_start) * 0.225, 3),
                   int(self.step_position != self.step_target))

    @property
    def in_waiting(self):
        self._advance()
        return len(self.buffer)

    def read(self, size):
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def write(self, data):
        self._advance()
        try:
            command = validate_command(data.decode("ascii").strip())
        except (ValueError, ProtocolError):
            self._line("ERR", "COMMAND")
            return len(data)
        name, _, argument = command.partition(",")
        if name in ("STOP", "MODE"):
            self.mode = "STOPPED" if name == "STOP" else argument
            self.dc_mode = "IDLE"
            self.dc_rpm = self.target_rpm = 0
            self.step_target = self.step_position
            self._line("ACK", name, self.mode)
        elif self.mode != "GUI":
            self._line("ERR", "SENSOR_MODE" if self.mode == "SENSOR" else "STOPPED")
        else:
            value = float(argument)
            if name == "SERVO":
                self.servo = value
            elif name == "DC_POS":
                self.dc_mode = "POS"
                self.dc_target = self.dc_position + value
                self.target_rpm = 0
            elif name == "DC_VEL":
                self.target_rpm = value
                self.dc_mode = "VEL" if value else "IDLE"
            elif name == "STEPPER":
                # Arduino lroundf rounds ties away from zero, unlike Python round.
                steps = int(math.copysign(math.floor(abs(value / 0.225) + 0.5), value))
                self.step_start = self.step_position
                self.step_target = self.step_position + steps
                value = self.rounded_move = steps * 0.225
            self._line("ACK", name, f"{value:.3f}")
        return len(data)

    def close(self):
        self.closed = True
