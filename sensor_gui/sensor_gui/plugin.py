"""rqt owns the ROS executor; only queued signals touch Qt widgets."""
import argparse
import json
from python_qt_binding.QtCore import QObject, Signal, Qt
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .bridge import status_qos
from .dashboard import Dashboard


class GuiSignals(QObject):
    status = Signal(dict)
    telemetry = Signal(dict)
    result = Signal(dict)


class SensorsMotorsPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("SensorsMotors")
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument("--topic-prefix", default="/lab")
        args, _ = parser.parse_known_args(context.argv())
        prefix = "/" + args.topic_prefix.strip("/")
        self.node = context.node
        self.widget = Dashboard()
        self.signals = GuiSignals()
        self.signals.status.connect(self.widget.receive_status, Qt.QueuedConnection)
        self.signals.telemetry.connect(self.widget.receive_telemetry, Qt.QueuedConnection)
        self.signals.result.connect(self.widget.receive_result, Qt.QueuedConnection)
        self.publisher = self.node.create_publisher(String, prefix + "/command", 10)
        self.client = self.node.create_client(Trigger, prefix + "/reconnect")
        self.subscriptions = [
            self.node.create_subscription(String, prefix + "/bridge/status", self._status, status_qos()),
            self.node.create_subscription(String, prefix + "/telemetry", self._telemetry, 100),
            self.node.create_subscription(String, prefix + "/command/result", self._result, 50),
        ]
        self.widget.command_requested.connect(self._send)
        self.widget.reconnect_requested.connect(self._reconnect)
        if context.serial_number() > 1:
            self.widget.setWindowTitle(f"{self.widget.windowTitle()} ({context.serial_number()})")
        context.add_widget(self.widget)

    def _decode(self, message, signal):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                signal.emit(value)
        except ValueError:
            self.node.get_logger().warning("Invalid dashboard JSON")

    def _status(self, message):
        self._decode(message, self.signals.status)

    def _telemetry(self, message):
        self._decode(message, self.signals.telemetry)

    def _result(self, message):
        self._decode(message, self.signals.result)

    def _send(self, command):
        self.publisher.publish(String(data=command))

    def _reconnect(self):
        if self.client.service_is_ready():
            future = self.client.call_async(Trigger.Request())
            future.add_done_callback(self._reconnected)
        else:
            self.widget.receive_result({"outcome": "error", "command": "RECONNECT", "detail": "Bridge service is unavailable"})

    def _reconnected(self, future):
        try:
            response = future.result()
            self.signals.result.emit({"outcome": "info", "command": "RECONNECT", "detail": response.message})
        except Exception as exc:
            self.signals.result.emit({"outcome": "error", "command": "RECONNECT", "detail": str(exc)})

    def shutdown_plugin(self):
        self.widget.shutdown()
        # Unloading a panel must not destroy rqt's shared ROS node/executor.
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.node.destroy_publisher(self.publisher)
        self.node.destroy_client(self.client)
