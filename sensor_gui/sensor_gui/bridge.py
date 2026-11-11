"""Single-owner serial bridge; callbacks run in one ROS executor thread."""
import json
import signal
import time
import uuid
import serial
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Float64, Bool
from std_srvs.srv import Trigger
from .protocol import CommandQueue, LineDecoder, ProtocolError, SCHEMAS, parse_line, validate_command
from .simulator import SimulatorTransport


def status_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SerialBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("sensor_serial_bridge", **kwargs)
        defaults = {"port": "/dev/ttyACM0", "baud": 115200, "simulate": False,
                    "topic_prefix": "/lab", "boot_wait_s": 2.0,
                    "ack_timeout_s": 0.75, "telemetry_timeout_s": 1.5,
                    "reconnect_interval_s": 2.0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        self.prefix = "/" + str(self.settings["topic_prefix"]).strip("/")
        self.simulate = bool(self.settings["simulate"])
        self.transport = None
        self.commands = CommandQueue(float(self.settings["ack_timeout_s"]))
        self.decoder = LineDecoder()
        self.session = ""
        self.connected = False
        self.mode = "UNKNOWN"
        self.detail = "Waiting for device"
        self.last_sys = None
        self.opened = self.next_connect = self.accept_after = 0.0
        self.malformed = 0
        self.last_tof = None
        self.telemetry = self.create_publisher(String, self.prefix + "/telemetry", 100)
        self.status = self.create_publisher(String, self.prefix + "/bridge/status", status_qos())
        self.results = self.create_publisher(String, self.prefix + "/command/result", 50)
        # Commands are VOLATILE: joining/reconnecting must not replay old moves.
        self.subscription = self.create_subscription(String, self.prefix + "/command", self._command, 10)
        self.reconnect_service = self.create_service(Trigger, self.prefix + "/reconnect", self._reconnect)
        self.values = {}
        for kind, schema in SCHEMAS.items():
            for field, convert in schema:
                message_type = String if convert is str else Bool if field in (
                    "detected", "valid", "online", "moving") else Float64
                self.values[(kind, field)] = (message_type, self.create_publisher(
                    message_type, f"{self.prefix}/{kind.lower()}/{field}", 10))
        self.timer = self.create_timer(0.01, self._tick)
        self.status_timer = self.create_timer(0.25, self._publish_status)
        self._publish_status()

    @staticmethod
    def _json(publisher, data):
        publisher.publish(String(data=json.dumps(data, allow_nan=False, separators=(",", ":"))))

    def _result(self, command, outcome, detail):
        self._json(self.results, {"session": self.session, "command": command,
                                  "outcome": outcome, "detail": detail})

    def _publish_status(self):
        now = time.monotonic()
        self._json(self.status, {
            "session": self.session, "connected": self.connected,
            "simulated": self.simulate, "port": "SIMULATOR" if self.simulate else self.settings["port"],
            "mode": self.mode, "detail": self.detail,
            "malformed_lines": self.malformed + self.decoder.errors,
            "pending": self.commands.pending.command if self.commands.pending else "",
            "queued": len(self.commands.queue),
            "sys_age_s": None if self.last_sys is None else round(now - self.last_sys, 3),
        })

    def _drop_commands(self, reason):
        for command in self.commands.clear():
            self._result(command, "uncertain/cancelled", reason + "; command will not be replayed")

    def _disconnect(self, reason):
        self._drop_commands(reason)
        if self.transport is not None:
            try:
                self.transport.close()
            except (OSError, serial.SerialException):
                pass
        self.transport = None
        self.connected = False
        self.mode = "UNKNOWN"
        self.last_sys = None
        self.detail = reason
        self.next_connect = time.monotonic() + float(self.settings["reconnect_interval_s"])
        self.get_logger().warning(reason)
        self._publish_status()

    def _open(self, now):
        try:
            self.transport = SimulatorTransport() if self.simulate else serial.Serial(
                port=self.settings["port"], baudrate=int(self.settings["baud"]),
                timeout=0, write_timeout=0.1, exclusive=True)
        except (OSError, serial.SerialException, ValueError) as exc:
            self._disconnect(f"Cannot open device: {exc}")
            return
        self.decoder = LineDecoder()
        self.session = uuid.uuid4().hex[:12]
        self.last_sys = None
        self.last_tof = None
        self.opened = now
        self.accept_after = now + (0.1 if self.simulate else float(self.settings["boot_wait_s"]))
        self.detail = "Waiting for boot and protocol-v1 SYS telemetry"
        self._publish_status()

    def _reconnect(self, request, response):
        self._disconnect("Reconnect requested; old commands discarded")
        self.next_connect = 0
        response.success = True
        response.message = "Reopening device; wait for fresh SYS telemetry"
        return response

    def _command(self, message):
        try:
            command = validate_command(message.data)
            if not self.connected:
                raise ProtocolError("Device not ready")
            if command not in ("STOP", "MODE,GUI", "MODE,SENSOR") and self.mode != "GUI":
                raise ProtocolError("Motor commands require confirmed GUI mode")
            for cancelled in self.commands.enqueue(command, time.monotonic()):
                self._result(cancelled, "cancelled", "Superseded by STOP")
            self._result(command, "queued", "Waiting for serial command slot")
        except ProtocolError as exc:
            self._result(message.data, "rejected", str(exc))
        self._publish_status()

    def _record(self, record, now):
        kind = record["kind"]
        if kind == "READY":
            if self.connected:
                self._drop_commands("Device reset during session")
                self.session = uuid.uuid4().hex[:12]
                self.last_tof = None
            self.connected = False
            self.mode = "UNKNOWN"
            self.last_sys = None
            self.opened = now
            self.accept_after = max(self.accept_after, now + 0.1)
            self.detail = "Device booted; waiting for fresh SYS"
            self._publish_status()
            return
        if kind in ("ACK", "ERR"):
            try:
                command = self.commands.complete(record)
            except ProtocolError as exc:
                self._disconnect(str(exc))
                return
            if command:
                self._result(command, "acknowledged" if kind == "ACK" else "error",
                             record.get("applied", record.get("reason", "")))
            return
        if kind == "SYS":
            self.last_sys = now
            self.mode = record["data"]["mode"]
            if now >= self.accept_after:
                self.connected = True
                self.detail = "SIMULATED data — not hardware measurements" if self.simulate else "Device ready"
        envelope = dict(record, session=self.session, simulated=self.simulate,
                        received_ns=self.get_clock().now().nanoseconds)
        self._json(self.telemetry, envelope)
        data = record["data"]
        new_tof = kind != "TOF" or (data["valid"] and data["online"] and
                                   (data["sample_seq"], data["sample_ms"]) != self.last_tof)
        if kind == "TOF" and new_tof:
            self.last_tof = (data["sample_seq"], data["sample_ms"])
        for field, value in data.items():
            if (kind == "TOF" and field in ("raw_mm", "filtered_mm") and not new_tof) or (
                    kind == "IR" and field == "distance_cm" and not data["detected"]):
                continue
            message_type, publisher = self.values[(kind, field)]
            publisher.publish(message_type(data=value if message_type in (String, Bool) else float(value)))

    def _tick(self):
        now = time.monotonic()
        if self.transport is None:
            if now >= self.next_connect:
                self._open(now)
            return
        try:
            chunk = self.transport.read(min(self.transport.in_waiting, 8192))
            for line in self.decoder.feed(chunk):
                try:
                    self._record(parse_line(line), now)
                except (ProtocolError, ValueError):
                    self.malformed += 1
                if self.transport is None:
                    return
            age = now - (self.last_sys if self.last_sys is not None else self.opened)
            allowed = float(self.settings["telemetry_timeout_s"]) + (
                max(0, self.accept_after - self.opened) if self.last_sys is None else 0)
            if age > allowed:
                self._disconnect("SYS telemetry timed out; motor state is unknown")
                return
            if self.commands.expired(now):
                self._disconnect("ACK timed out; last command outcome is unknown")
                return
            if self.connected:
                command = self.commands.next(now)
                if command is not None:
                    data = (command + "\n").encode("ascii")
                    if self.transport.write(data) != len(data):
                        raise ProtocolError("Partial serial write; command outcome unknown")
                    self._result(command, "sent", "Waiting for firmware ACK/ERR")
        except (OSError, serial.SerialException, ProtocolError) as exc:
            self._disconnect(str(exc))

    def destroy_node(self):
        self.timer.cancel()
        self.status_timer.cancel()
        if self.transport is not None:
            # Best effort only; shutdown/disconnection cannot guarantee a physical stop.
            try:
                self.transport.write(b"STOP\n")
                self.transport.close()
            except (OSError, serial.SerialException):
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch may deliver another SIGINT while ROS entities are being destroyed.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
