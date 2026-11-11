"""Protocol v1 validation, independent of ROS and Qt."""
from collections import deque
from dataclasses import dataclass
import math


class ProtocolError(ValueError):
    pass


def number(text):
    try:
        value = float(text)
    except (ValueError, OverflowError) as exc:
        raise ProtocolError("Invalid number") from exc
    if not math.isfinite(value):
        raise ProtocolError("Non-finite number")
    return value


def integer(text):
    value = int(text)
    if not -(2**31) <= value <= 2**32 - 1:
        raise ProtocolError("Integer outside firmware range")
    return value


def flag(text):
    if text not in ("0", "1"):
        raise ProtocolError("Expected 0 or 1")
    return text == "1"


# Order exactly matches Unified_Sensors_Motors.ino. Float64 scalar topics retain
# physical units in their names; JSON telemetry retains the complete record.
SCHEMAS = {
    "SYS": [("mode", str), ("protocol_version", integer)],
    "POT": [("adc", integer), ("angle_deg", number), ("filtered_deg", number),
            ("servo_command_deg", number)],
    "IR": [("adc", number), ("volts", number), ("distance_cm", number),
           ("detected", flag), ("requested_pwm", integer)],
    "ULTRA": [("adc", integer), ("raw_cm", number), ("filtered_cm", number)],
    "TOF": [("sample_seq", integer), ("sample_ms", integer), ("raw_mm", integer),
            ("filtered_mm", number), ("valid", flag), ("range_status", integer), ("online", flag)],
    "DC": [("control_mode", str), ("position_deg", number), ("velocity_rpm", number),
           ("target_position_deg", number), ("target_rpm", number), ("applied_pwm", integer)],
    "STEPPER": [("position_steps", integer), ("target_steps", integer),
                ("position_deg", number), ("target_deg", number),
                ("rounded_move_deg", number), ("moved_deg", number), ("moving", flag)],
}


def parse_line(line):
    fields = line.strip().split(",")
    kind = fields[0]
    if kind == "READY":
        if fields != ["READY", "UNIFIED_SENSORS_MOTORS", "1"]:
            raise ProtocolError("Unknown device or protocol")
        return {"kind": kind}
    if kind == "ERR":
        if len(fields) != 2 or not fields[1]:
            raise ProtocolError("Malformed ERR")
        return {"kind": kind, "reason": fields[1]}
    if kind == "ACK":
        if len(fields) != 3:
            raise ProtocolError("Malformed ACK")
        name, applied = fields[1:]
        if name in ("MODE", "STOP"):
            if applied not in ("GUI", "SENSOR", "STOPPED"):
                raise ProtocolError("Unknown mode in ACK")
        elif name in ("SERVO", "DC_POS", "DC_VEL", "STEPPER"):
            number(applied)
        else:
            raise ProtocolError("Unknown ACK")
        return {"kind": kind, "command": name, "applied": applied}
    if kind not in SCHEMAS or len(fields) != len(SCHEMAS[kind]) + 2:
        raise ProtocolError("Unknown record or wrong field count")
    try:
        ms = integer(fields[1])
        if ms < 0:
            raise ProtocolError("Negative timestamp")
        data = {key: convert(value) for (key, convert), value in
                zip(SCHEMAS[kind], fields[2:])}
    except (ValueError, OverflowError) as exc:
        raise ProtocolError(str(exc)) from exc
    if kind == "SYS" and (data["protocol_version"] != 1 or
                          data["mode"] not in ("GUI", "SENSOR", "STOPPED")):
        raise ProtocolError("Unsupported mode/protocol")
    if kind == "DC" and data["control_mode"] not in ("IDLE", "POS", "VEL", "IR"):
        raise ProtocolError("Unknown DC mode")
    if kind == "TOF" and (data["sample_ms"] < 0 or data["sample_seq"] < 0):
        raise ProtocolError("Negative sample identity")
    return {"kind": kind, "ms": ms, "data": data}


class LineDecoder:
    """Bound memory and discard whole bad lines, including their suffix."""
    def __init__(self, maximum=256):
        self.maximum = maximum
        self.buffer = bytearray()
        self.discard = False
        self.errors = 0

    def feed(self, chunk):
        lines = []
        for value in chunk:
            if value in (10, 13):
                if self.discard:
                    self.errors += 1
                elif self.buffer:
                    lines.append(self.buffer.decode("ascii"))
                self.buffer.clear()
                self.discard = False
            elif not self.discard:
                if value < 32 or value > 126 or len(self.buffer) >= self.maximum:
                    self.buffer.clear()
                    self.discard = True
                else:
                    self.buffer.append(value)
        return lines


def validate_command(text):
    """Normalize a ROS command before it can reach a physical motor."""
    if not isinstance(text, str) or len(text) > 47 or any(ord(c) < 32 or ord(c) > 126 for c in text):
        raise ProtocolError("Command must be <=47 printable ASCII characters")
    text = text.strip()
    aliases = {"STOP,ALL": "STOP", "M0": "MODE,SENSOR", "M1": "MODE,GUI"}
    text = aliases.get(text, text)
    if text in ("STOP", "MODE,GUI", "MODE,SENSOR"):
        return text
    parts = text.split(",")
    if len(parts) != 2:
        raise ProtocolError("Expected COMMAND,value")
    name, value = parts
    name = {"POS": "DC_POS", "VEL": "DC_VEL"}.get(name, name)
    limits = {"SERVO": (-90, 100), "DC_POS": (-3600, 3600),
              "DC_VEL": (-12, 12), "STEPPER": (-360, 360)}
    if name not in limits:
        raise ProtocolError("Unknown command")
    try:
        value = number(value)
    except ValueError as exc:
        raise ProtocolError("Invalid numeric argument") from exc
    if not limits[name][0] <= value <= limits[name][1]:
        raise ProtocolError(f"{name} outside {limits[name]}")
    # Round-trip precision at sub-millidegree scale; no scientific notation needed.
    return f"{name},{value:.6f}".rstrip("0").rstrip(".")


@dataclass
class Pending:
    command: str
    created: float
    sent: float = 0.0


class CommandQueue:
    """Single ACK in flight. A timeout is ambiguous and never retried."""
    def __init__(self, timeout=0.75, maximum=8):
        self.timeout = timeout
        self.maximum = maximum
        self.queue = deque()
        self.pending = None

    def enqueue(self, command, now):
        dropped = []
        if command == "STOP":
            dropped = [entry.command for entry in self.queue]
            self.queue.clear()
        elif len(self.queue) >= self.maximum:
            raise ProtocolError("Command queue full")
        self.queue.append(Pending(command, now))
        return dropped

    def next(self, now):
        if self.pending or not self.queue:
            return None
        entry = self.queue.popleft()
        if entry.command != "STOP" and now - entry.created > 1.0:
            raise ProtocolError("Queued command expired")
        entry.sent = now
        self.pending = entry
        return entry.command

    def complete(self, record):
        if not self.pending:
            return None
        if record["kind"] == "ACK":
            expected = self.pending.command.split(",")[0]
            if record["command"] != expected:
                raise ProtocolError("ACK does not match in-flight command")
            if expected == "MODE" and record["applied"] != self.pending.command.split(",")[1]:
                raise ProtocolError("MODE acknowledgment does not match requested mode")
            if expected == "STOP" and record["applied"] != "STOPPED":
                raise ProtocolError("STOP was not acknowledged as STOPPED")
        command = self.pending.command
        self.pending = None
        return command

    def expired(self, now):
        return self.pending is not None and now - self.pending.sent > self.timeout

    def clear(self):
        entries = ([self.pending] if self.pending else []) + list(self.queue)
        self.pending = None
        self.queue.clear()
        return [entry.command for entry in entries]
