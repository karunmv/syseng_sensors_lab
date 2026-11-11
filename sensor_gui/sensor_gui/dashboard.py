"""Qt widgets only. ROS callbacks enter via queued Qt signals in plugin.py."""
from collections import deque
import csv
import time
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QDoubleSpinBox, QGroupBox, QTabWidget, QTableWidget, QTableWidgetItem,
    QHeaderView, QPlainTextEdit, QFileDialog,
)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from .protocol import SCHEMAS, validate_command


class Dashboard(QWidget):
    command_requested = Signal(str)
    reconnect_requested = Signal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TSP05 · Sensors and Motors")
        self.resize(1120, 850)
        self.session = None
        self.bridge = {}
        self.last_status = 0.0
        self.records = {}
        self.seen = {}
        self.motion_buttons = []
        self.last_sample = None
        self.tof_samples = deque(maxlen=36000)  # bounded, up to one hour at 10 Hz
        self.dc_samples = deque(maxlen=600)
        self.plot_dirty = False
        self.plot_state = None
        self.local_command = None

        root = QVBoxLayout(self)
        self.banner = QLabel("Waiting for ROS bridge status…")
        self.banner.setWordWrap(True)
        root.addWidget(self.banner)
        toolbar = QHBoxLayout()
        self.gui_button = QPushButton("GUI control")
        self.sensor_button = QPushButton("Sensor control")
        self.stop_button = QPushButton("STOP ALL")
        self.stop_button.setStyleSheet("background: #b42318; color: white; font-weight: bold; padding: 8px")
        self.reconnect_button = QPushButton("Reconnect device")
        self.gui_button.clicked.connect(lambda: self.send("MODE,GUI"))
        self.sensor_button.clicked.connect(lambda: self.send("MODE,SENSOR"))
        self.stop_button.clicked.connect(lambda: self.send("STOP"))
        self.reconnect_button.clicked.connect(self.reconnect_requested.emit)
        for button in (self.gui_button, self.sensor_button, self.stop_button, self.reconnect_button):
            toolbar.addWidget(button)
        root.addLayout(toolbar)
        self.command_status = QLabel("No commands sent")
        self.command_status.setWordWrap(True)
        root.addWidget(self.command_status)

        tabs = QTabWidget()
        root.addWidget(tabs, 1)
        controls = QWidget()
        layout = QVBoxLayout(controls)
        servo_box, servo_layout = self._group("RC servo · calibrated commanded angle")
        self.servo_input = self._input(-90, 100, 0, 1)
        servo_layout.addWidget(QLabel("Target (deg)"), 0, 0)
        servo_layout.addWidget(self.servo_input, 0, 1)
        self._motion(servo_layout, "Apply angle", lambda: self.send(f"SERVO,{self.servo_input.value()}"), 0, 2)
        self._motion(servo_layout, "−90° limit", lambda: self.send("SERVO,-90"), 0, 3)
        self._motion(servo_layout, "+100° limit", lambda: self.send("SERVO,100"), 0, 4)
        self.servo_feedback = QLabel("Command: — (no shaft feedback sensor)")
        servo_layout.addWidget(self.servo_feedback, 1, 0, 1, 5)
        layout.addWidget(servo_box)

        dc_box, dc_layout = self._group("DC motor · encoder feedback")
        self.dc_position_input = self._input(-3600, 3600, 30, 1)
        self.dc_velocity_input = self._input(-12, 12, 5, 2)
        dc_layout.addWidget(QLabel("Relative move (deg)"), 0, 0)
        dc_layout.addWidget(self.dc_position_input, 0, 1)
        self._motion(dc_layout, "Move DC", lambda: self.send(f"DC_POS,{self.dc_position_input.value()}"), 0, 2)
        dc_layout.addWidget(QLabel("Velocity (RPM, signed)"), 1, 0)
        dc_layout.addWidget(self.dc_velocity_input, 1, 1)
        self._motion(dc_layout, "Set velocity", lambda: self.send(f"DC_VEL,{self.dc_velocity_input.value()}"), 1, 2)
        self._motion(dc_layout, "Stop DC", lambda: self.send("DC_VEL,0"), 1, 3)
        self.dc_feedback = QLabel("Waiting for encoder feedback…")
        self.dc_feedback.setWordWrap(True)
        dc_layout.addWidget(self.dc_feedback, 2, 0, 1, 5)
        layout.addWidget(dc_box)

        step_box, step_layout = self._group("Stepper · estimated position from issued pulses")
        self.stepper_input = self._input(-360, 360, 45, 3)
        step_layout.addWidget(QLabel("Relative move (deg)"), 0, 0)
        step_layout.addWidget(self.stepper_input, 0, 1)
        self._motion(step_layout, "Move stepper", lambda: self.send(f"STEPPER,{self.stepper_input.value()}"), 0, 2)
        self.stepper_feedback = QLabel("Waiting for stepper state…")
        self.stepper_feedback.setWordWrap(True)
        step_layout.addWidget(self.stepper_feedback, 1, 0, 1, 5)
        layout.addWidget(step_box)

        self.sensor_table = QTableWidget(4, 4)
        self.sensor_table.setHorizontalHeaderLabels(["Sensor", "Physical quantity", "Raw / filtered", "Status"])
        self.sensor_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.sensor_table.verticalHeader().hide()
        self.sensor_table.setEditTriggers(QTableWidget.NoEditTriggers)
        for row, name in enumerate(("Potentiometer", "Sharp IR", "Ultrasonic", "ToF VL53L1X")):
            for column in range(4):
                self.sensor_table.setItem(row, column, QTableWidgetItem(name if column == 0 else "—"))
        layout.addWidget(self.sensor_table)
        tabs.addTab(controls, "Control and sensors")

        plots = QWidget()
        plot_layout = QVBoxLayout(plots)
        self.figure = Figure(figsize=(9, 6), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.tof_axis, self.dc_axis = self.figure.subplots(2, 1)
        self.raw_line, = self.tof_axis.plot([], [], label="Raw ToF", color="#b54708", alpha=0.7)
        self.filtered_line, = self.tof_axis.plot([], [], label="Filtered ToF", color="#175cd3", linewidth=2)
        self.tof_axis.set(ylabel="Distance (mm)", xlabel="Device sample time (s)", title="ToF · raw vs filtered")
        self.tof_axis.legend(loc="upper right")
        self.tof_axis.grid(alpha=0.25)
        self.rpm_line, = self.dc_axis.plot([], [], label="Measured RPM", color="#067647")
        self.target_line, = self.dc_axis.plot([], [], label="Target RPM (velocity mode)", color="#6941c6")
        self.dc_axis.set(ylabel="Velocity (RPM)", xlabel="Device time (s)", title="DC motor velocity")
        self.dc_axis.legend(loc="upper right")
        self.dc_axis.grid(alpha=0.25)
        plot_layout.addWidget(self.canvas)
        export = QHBoxLayout()
        self.export_button = QPushButton("Export valid ToF pairs to CSV…")
        self.export_button.clicked.connect(self.export_csv_dialog)
        export.addWidget(self.export_button)
        self.capture_label = QLabel("0 valid ToF pairs captured")
        export.addWidget(self.capture_label)
        plot_layout.addLayout(export)
        plot_layout.addWidget(QLabel("SIMULATED data is labelled in plots and CSV. It is not lab measurement evidence."))
        tabs.addTab(plots, "Live plots and capture")

        self.events = QPlainTextEdit()
        self.events.setReadOnly(True)
        self.events.document().setMaximumBlockCount(250)
        tabs.addTab(self.events, "Commands and events")
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(200)
        self.refresh()

    @staticmethod
    def _group(title):
        box = QGroupBox(title)
        return box, QGridLayout(box)

    @staticmethod
    def _input(low, high, value, decimals):
        widget = QDoubleSpinBox()
        widget.setRange(low, high)
        widget.setDecimals(decimals)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        return widget

    def _motion(self, layout, label, callback, row, column):
        button = QPushButton(label)
        button.clicked.connect(callback)
        layout.addWidget(button, row, column)
        self.motion_buttons.append(button)
        return button

    def send(self, text):
        text = validate_command(text)
        self.local_command = text
        self.command_status.setText(f"Requested: {text}")
        # Disable immediately, before the bridge status round-trip can arrive.
        for button in self.motion_buttons + [self.gui_button, self.sensor_button]:
            button.setEnabled(False)
        self.command_requested.emit(text)

    def receive_status(self, status):
        self.last_status = time.monotonic()
        session = status.get("session")
        if session != self.session:
            self.session = session
            self.records.clear()
            self.seen.clear()
            self.last_sample = None
            self.dc_samples.clear()
            # Keep captured pairs from earlier sessions for export; each is labelled.
            self.plot_dirty = True
        self.bridge = status
        if not status.get("connected"):
            self.local_command = None
        self.refresh()

    def receive_result(self, result):
        if result.get("command") == self.local_command and result.get("outcome") in (
                "acknowledged", "error", "rejected", "cancelled", "uncertain/cancelled"):
            self.local_command = None
        text = f"{result['outcome']}: {result['command']} — {result['detail']}"
        self.command_status.setText(text)
        self.events.appendPlainText(text)

    def receive_telemetry(self, record):
        if record.get("session") != self.session:
            return
        kind, data = record["kind"], record["data"]
        if kind not in SCHEMAS:
            return
        self.records[kind] = data
        self.seen[kind] = time.monotonic()
        if kind == "TOF" and data["valid"] and data["online"]:
            identity = (self.session, data["sample_seq"], data["sample_ms"])
            if identity != self.last_sample:
                self.last_sample = identity
                self.tof_samples.append(dict(data, session=self.session, simulated=record["simulated"]))
                self.plot_dirty = True
        elif kind == "DC":
            target = data["target_rpm"] if data["control_mode"] == "VEL" else float("nan")
            self.dc_samples.append((record["ms"] / 1000, data["velocity_rpm"], target))
            self.plot_dirty = True

    def refresh(self):
        now = time.monotonic()
        online = bool(self.bridge.get("connected")) and now - self.last_status < 1.0
        busy = bool(self.local_command or self.bridge.get("pending") or self.bridge.get("queued"))
        mode = self.bridge.get("mode", "UNKNOWN")
        simulated = self.bridge.get("simulated", False)
        source = "SIMULATED" if simulated else "HARDWARE"
        tof_state = self.records.get("TOF", {})
        tof_fresh = (online and now - self.seen.get("TOF", 0) < 1.0 and
                     tof_state.get("valid", False) and tof_state.get("online", False))
        dc_fresh = online and now - self.seen.get("DC", 0) < 1.0
        if (source, tof_fresh, dc_fresh) != self.plot_state:
            self.plot_state = (source, tof_fresh, dc_fresh)
            self.plot_dirty = True
        if not online:
            detail = self.bridge.get("detail", "No ROS bridge status")
            if self.last_status and now - self.last_status >= 1:
                detail = "ROS bridge status is stale"
            self.banner.setText(f"{source} · DISCONNECTED / NOT READY · {detail}")
            self.banner.setStyleSheet("background: #fef0c7; color: #7a2e0e; padding: 9px")
        else:
            self.banner.setText(f"{source} · {self.bridge.get('port', '')} · Mode: {mode} · "
                                f"{self.bridge.get('detail', '')} · Bad lines: {self.bridge.get('malformed_lines', 0)}")
            self.banner.setStyleSheet("background: #d1e9ff; color: #1849a9; padding: 9px" if simulated else
                                     "background: #dcfae6; color: #085d3a; padding: 9px")
        for button in self.motion_buttons:
            button.setEnabled(online and mode == "GUI" and not busy)
        self.gui_button.setEnabled(online and not busy)
        self.sensor_button.setEnabled(online and not busy)
        self.stop_button.setEnabled(online) # STOP can clear queued motion while another ACK is pending
        def fresh(kind):
            return online and now - self.seen.get(kind, 0) < 1.0

        for row, kind in enumerate(("POT", "IR", "ULTRA", "TOF")):
            data = self.records.get(kind)
            valid = fresh(kind)
            values = ["—", "—", "STALE / waiting"]
            if data and valid:
                if kind == "POT":
                    values = [f"{data['filtered_deg']:.2f} deg", f"{data['angle_deg']:.2f} / {data['filtered_deg']:.2f} deg; ADC {data['adc']}", "Valid"]
                elif kind == "IR":
                    values = [f"{data['distance_cm']:.2f} cm" if data["detected"] else "No detection",
                              f"{data['volts']:.3f} V; ADC {data['adc']:.1f}", "Detected" if data["detected"] else "Motor PWM request 0"]
                elif kind == "ULTRA":
                    values = [f"{data['filtered_cm']:.2f} cm", f"{data['raw_cm']:.2f} / {data['filtered_cm']:.2f} cm", "Nominal conversion"]
                else:
                    valid_tof = data["valid"] and data["online"]
                    values = [f"{data['filtered_mm']:.2f} mm" if valid_tof else "Invalid / offline",
                              f"{data['raw_mm']} / {data['filtered_mm']:.2f} mm" if valid_tof else "—",
                              f"Status {data['range_status']}; sample {data['sample_seq']}"]
            for column, text in enumerate(values, 1):
                self.sensor_table.item(row, column).setText(text)
        pot = self.records.get("POT", {})
        self.servo_feedback.setText(f"Command: {pot['servo_command_deg']:.2f} deg (not measured)" if fresh("POT") else "Servo command: STALE / waiting")
        dc = self.records.get("DC", {})
        target = (f"Target position {dc.get('target_position_deg', 0):.2f} deg" if dc.get("control_mode") == "POS" else
                  f"Target velocity {dc.get('target_rpm', 0):.2f} RPM" if dc.get("control_mode") == "VEL" else
                  "IR uses direct PWM" if dc.get("control_mode") == "IR" else "Idle")
        self.dc_feedback.setText(
            f"{dc['control_mode']} · Position {dc['position_deg']:.2f} deg · Velocity {dc['velocity_rpm']:.2f} RPM · "
            f"PWM {dc['applied_pwm']} · {target}"
            if fresh("DC") else "Encoder feedback: STALE / waiting")
        step = self.records.get("STEPPER", {})
        self.stepper_feedback.setText(
            f"Estimate {step['position_deg']:.3f} deg → target {step['target_deg']:.3f} deg · "
            f"Move {step['moved_deg']:.3f} / {step['rounded_move_deg']:.3f} deg · "
            + ("MOVING" if step["moving"] else "Holding") if fresh("STEPPER") else "Stepper state: STALE / waiting")
        self.capture_label.setText(f"{len(self.tof_samples)} valid ToF pairs captured (max 36,000)")
        self.export_button.setEnabled(bool(self.tof_samples))
        if self.plot_dirty:
            pairs = [sample for sample in self.tof_samples if sample["session"] == self.session][-600:]
            times = [sample["sample_ms"] / 1000 for sample in pairs]
            self.raw_line.set_data(times, [sample["raw_mm"] for sample in pairs])
            self.filtered_line.set_data(times, [sample["filtered_mm"] for sample in pairs])
            self.rpm_line.set_data([entry[0] for entry in self.dc_samples], [entry[1] for entry in self.dc_samples])
            self.target_line.set_data([entry[0] for entry in self.dc_samples], [entry[2] for entry in self.dc_samples])
            self.tof_axis.set_title(f"{source} · ToF raw vs filtered" + (" · INVALID / STALE" if not tof_fresh else ""))
            self.dc_axis.set_title(f"{source} · DC motor velocity" + (" · STALE / disconnected" if not dc_fresh else ""))
            for axis in (self.tof_axis, self.dc_axis):
                axis.relim()
                axis.autoscale_view()
            self.canvas.draw_idle()
            self.plot_dirty = False

    def export_csv(self, path):
        fields = ["session", "simulated", "sample_seq", "sample_ms", "raw_mm", "filtered_mm", "valid", "range_status", "online"]
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.tof_samples)

    def export_csv_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export valid ToF pairs", "tof_pairs.csv", "CSV files (*.csv)")
        if path:
            try:
                self.export_csv(path)
                self.events.appendPlainText(f"Exported {len(self.tof_samples)} labelled pairs to {path}")
            except OSError as exc:
                self.command_status.setText(f"Export failed: {exc}")

    def shutdown(self):
        self.refresh_timer.stop()
