"""Brutalist PySide6 control panel for the dishwasher robot gripper."""

from __future__ import annotations

import time

import serial.tools.list_ports
from PySide6.QtCore import QSettings, QTimer, Qt
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QMainWindow,
    QPushButton, QScrollArea, QSpinBox, QTabWidget, QVBoxLayout, QWidget,
)

from .config import (
    APP_TITLE, CALIBRATION_STEP_US, COMMAND_INTERVAL_MS, CONTINUOUS_SERVO_CHANNELS,
    DEFAULT_ACCELERATION_DEG_S2, DEFAULT_ANGLE, DEFAULT_CALIBRATION_US,
    DEFAULT_CONTINUOUS_CALIBRATION_US, DEFAULT_MAX_SPEED_DEG_S,
    MAX_ACCELERATION_DEG_S2, MAX_MAX_SPEED_DEG_S, MAX_ANGLE,
    MIN_ACCELERATION_DEG_S2, MIN_MAX_SPEED_DEG_S, MIN_ANGLE,
    PULSE_MAX_ALLOWED_US, PULSE_MIN_ALLOWED_US, SERVO_CHANNELS,
)
from .calibration import ServoCalibration
from .protocol import parse_line, set_angle, set_calibration, set_motion_profile
from .serial_worker import SerialWorker
from .servo_card import ServoCard


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1220, 880)
        self.setMinimumSize(900, 680)
        self.worker: SerialWorker | None = None
        self.arduino_ready = False
        self.pending: dict[int, int] = {}
        self.cards: dict[int, ServoCard] = {}
        self._last_error = ""
        self._last_pong = 0.0
        self._reported_servo_count: int | None = None
        self.settings = QSettings("MDU-C2", "Gripper Control")
        self.cal_inputs: dict[int, tuple[QSpinBox, QSpinBox, QSpinBox]] = {}
        self.motion_enabled_boxes: dict[int, QCheckBox] = {}
        self.motion_speed_spins: dict[int, QSpinBox] = {}
        self.motion_acceleration_spins: dict[int, QSpinBox] = {}
        self.motion_notes: dict[int, QLabel] = {}
        self.cal_test_buttons: dict[int, tuple[QPushButton, QPushButton, QPushButton]] = {}
        self.cal_notes: dict[int, QLabel] = {}

        self._build_ui()
        self.refresh_ports()

        self.send_timer = QTimer(self)
        self.send_timer.setInterval(COMMAND_INTERVAL_MS)
        self.send_timer.timeout.connect(self._flush_pending)
        self.send_timer.start()

        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.setInterval(1000)
        self.heartbeat_timer.timeout.connect(self._heartbeat)
        self.heartbeat_timer.start()

    def _build_ui(self) -> None:
        scroll = QScrollArea()
        scroll.setObjectName("AppScroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setCentralWidget(scroll)

        page = QWidget()
        page.setObjectName("Page")
        scroll.setWidget(page)
        content = QVBoxLayout(page)
        content.setContentsMargins(28, 22, 28, 26)
        content.setSpacing(16)

        masthead = QHBoxLayout()
        heading = QVBoxLayout()
        heading.setSpacing(2)
        eyebrow = QLabel("MDU  /  DISHWASHER ROBOT  /  HT26")
        eyebrow.setObjectName("Kicker")
        heading.addWidget(eyebrow)
        title = QLabel("GRIPPER CONTROL")
        title.setObjectName("Hero")
        heading.addWidget(title)
        subtitle = QLabel("MANUAL SERVO CONTROL  ·  ARDUINO UNO R4 WIFI + PCA9685")
        subtitle.setObjectName("BodyMuted")
        heading.addWidget(subtitle)
        masthead.addLayout(heading)
        masthead.addStretch(1)
        self.status = QLabel("■ OFFLINE")
        self.status.setObjectName("Off")
        masthead.addWidget(self.status, alignment=Qt.AlignmentFlag.AlignTop)
        content.addLayout(masthead)

        connection = QFrame()
        connection.setObjectName("Panel")
        connection_layout = QVBoxLayout(connection)
        connection_layout.setContentsMargins(18, 14, 18, 14)
        connection_layout.setSpacing(10)
        label = QLabel("01  /  CONNECTION & SAFETY")
        label.setObjectName("Kicker")
        connection_layout.addWidget(label)
        row = QHBoxLayout()
        row.setSpacing(9)
        self.ports = QComboBox()
        self.ports.setMinimumWidth(280)
        row.addWidget(self.ports, 1)
        self.refresh_button = QPushButton("REFRESH PORTS")
        self.refresh_button.clicked.connect(self.refresh_ports)
        row.addWidget(self.refresh_button)
        self.connect_button = QPushButton("CONNECT")
        self.connect_button.setObjectName("Primary")
        self.connect_button.setMinimumWidth(130)
        self.connect_button.clicked.connect(self._toggle_connection)
        row.addWidget(self.connect_button)
        global_center = QPushButton("CENTER / STOP ALL  ↺")
        global_center.setToolTip(
            "Always available: moves all positional servos to 90° and stops all continuous servos"
        )
        global_center.clicked.connect(self._center_all)
        row.addWidget(global_center)
        connection_layout.addLayout(row)
        self.connection_note = QLabel("SELECT A PORT AND CONNECT.")
        self.connection_note.setObjectName("Description")
        connection_layout.addWidget(self.connection_note)
        content.addWidget(connection)

        self.tabs = QTabWidget()
        self.tabs.setObjectName("MainTabs")
        content.addWidget(self.tabs, 1)

        positional_channels = [
            ch for ch in SERVO_CHANNELS if ch not in CONTINUOUS_SERVO_CHANNELS
        ]
        continuous_channels = [
            ch for ch in SERVO_CHANNELS if ch in CONTINUOUS_SERVO_CHANNELS
        ]

        # Tab 01: Servos
        control_page = QWidget()
        control_layout = QVBoxLayout(control_page)
        control_layout.setContentsMargins(6, 16, 6, 12)
        control_layout.setSpacing(14)
        section_row = QHBoxLayout()
        section_title = QLabel("02  /  MANUAL SERVO CONTROL")
        section_title.setObjectName("Kicker")
        section_row.addWidget(section_title)
        section_row.addStretch(1)
        center = QPushButton("CENTER ALL  ↺")
        center.setToolTip("Moves all position servos to 90° and stops continuous servos")
        center.clicked.connect(self._center_all)
        section_row.addWidget(center)
        control_layout.addLayout(section_row)

        control_layout.addWidget(
            self._build_section_banner(
                "POSITIONAL SERVOS  ·  0°–180° JOINT CONTROL",
                f"{len(positional_channels)} channels · slider, fine nudge, and quick jumps",
                continuous=False,
            )
        )
        pos_grid = QGridLayout()
        pos_grid.setHorizontalSpacing(14)
        pos_grid.setVerticalSpacing(14)
        pos_grid.setColumnStretch(0, 1)
        pos_grid.setColumnStretch(1, 1)
        for index, channel in enumerate(positional_channels):
            card = ServoCard(channel, SERVO_CHANNELS[channel], continuous=False)
            card.changed.connect(self._servo_changed)
            self.cards[channel] = card
            pos_grid.addWidget(card, index // 2, index % 2)
        control_layout.addLayout(pos_grid)

        control_layout.addSpacing(6)
        control_layout.addWidget(
            self._build_section_banner(
                "CONTINUOUS SERVOS  ·  FINGER DRIVES (HOLD TO RUN)",
                f"{len(continuous_channels)} channels · hold Reverse/Forward to rotate, release or press Stop",
                continuous=True,
            )
        )
        cont_grid = QGridLayout()
        cont_grid.setHorizontalSpacing(14)
        cont_grid.setVerticalSpacing(14)
        cont_grid.setColumnStretch(0, 1)
        cont_grid.setColumnStretch(1, 1)
        for index, channel in enumerate(continuous_channels):
            card = ServoCard(channel, SERVO_CHANNELS[channel], continuous=True)
            card.changed.connect(self._servo_changed)
            self.cards[channel] = card
            cont_grid.addWidget(card, index // 2, index % 2)
        control_layout.addLayout(cont_grid)
        control_layout.addStretch(1)
        self.tabs.addTab(control_page, "01  SERVOS")

        # Tab 02: Calibration
        calibration_page = QWidget()
        calibration_layout = QVBoxLayout(calibration_page)
        calibration_layout.setContentsMargins(6, 16, 6, 12)
        calibration_layout.setSpacing(14)
        cal_intro = QFrame()
        cal_intro.setObjectName("Panel")
        intro_layout = QVBoxLayout(cal_intro)
        intro_layout.setContentsMargins(18, 14, 18, 14)
        intro_layout.setSpacing(6)
        intro_title = QLabel("03  /  SERVO CALIBRATION & MOTION PROFILES")
        intro_title.setObjectName("Kicker")
        intro_layout.addWidget(intro_title)
        intro_text = QLabel(
            "Adjust the pulse width and motion profile for each channel. Position servos use 0° / 90° / 180°. "
            "Continuous servos use reverse / stop / forward. Save at stop and test one "
            "point at a time without mechanical load."
        )
        intro_text.setObjectName("Description")
        intro_text.setWordWrap(True)
        intro_layout.addWidget(intro_text)
        calibration_layout.addWidget(cal_intro)

        calibration_layout.addWidget(
            self._build_section_banner(
                "POSITIONAL SERVOS  ·  CALIBRATION & MOTION",
                f"{len(positional_channels)} channels · 0° / 90° / 180° pulse mapping",
                continuous=False,
            )
        )
        pos_cal_grid = QGridLayout()
        pos_cal_grid.setHorizontalSpacing(14)
        pos_cal_grid.setVerticalSpacing(14)
        pos_cal_grid.setColumnStretch(0, 1)
        pos_cal_grid.setColumnStretch(1, 1)
        for index, channel in enumerate(positional_channels):
            pos_cal_grid.addWidget(self._build_calibration_card(channel), index // 2, index % 2)
        calibration_layout.addLayout(pos_cal_grid)

        calibration_layout.addSpacing(6)
        calibration_layout.addWidget(
            self._build_section_banner(
                "CONTINUOUS SERVOS  ·  FINGER DRIVES CALIBRATION & MOTION",
                f"{len(continuous_channels)} channels · reverse / stop / forward pulse mapping",
                continuous=True,
            )
        )
        cont_cal_grid = QGridLayout()
        cont_cal_grid.setHorizontalSpacing(14)
        cont_cal_grid.setVerticalSpacing(14)
        cont_cal_grid.setColumnStretch(0, 1)
        cont_cal_grid.setColumnStretch(1, 1)
        for index, channel in enumerate(continuous_channels):
            cont_cal_grid.addWidget(self._build_calibration_card(channel), index // 2, index % 2)
        calibration_layout.addLayout(cont_cal_grid)
        calibration_layout.addStretch(1)
        self.tabs.addTab(calibration_page, "02  CALIBRATION")

        footer = QLabel("SERIAL 115200 BAUD  /  ANGLE IS COMMANDED, NOT MEASURED")
        footer.setObjectName("SmallCaps")
        content.addWidget(footer)

    @staticmethod
    def _build_section_banner(title: str, subtitle: str, continuous: bool = False) -> QFrame:
        banner = QFrame()
        banner.setObjectName("SectionBanner")
        banner.setProperty("continuous", continuous)
        layout = QHBoxLayout(banner)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(12)
        title_label = QLabel(title)
        title_label.setObjectName("Kicker")
        layout.addWidget(title_label)
        subtitle_label = QLabel(subtitle)
        subtitle_label.setObjectName("Description")
        layout.addWidget(subtitle_label)
        layout.addStretch(1)
        badge = QLabel("360° CONTINUOUS" if continuous else "180° POSITIONAL")
        badge.setObjectName("TypeBadgeCont" if continuous else "TypeBadgePos")
        layout.addWidget(badge)
        return banner

    def _build_calibration_card(self, channel: int) -> QFrame:
        continuous = channel in CONTINUOUS_SERVO_CHANNELS
        panel = QFrame()
        panel.setObjectName("CalibrationCard")
        panel.setProperty("continuous", continuous)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        header = QHBoxLayout()
        header.setSpacing(8)
        number = QLabel(f"CH {channel:02d}")
        number.setObjectName("ChannelTag")
        header.addWidget(number)
        name = QLabel(SERVO_CHANNELS[channel].upper())
        name.setObjectName("ServoName")
        header.addWidget(name)
        header.addStretch(1)
        type_badge = QLabel("360° CONTINUOUS" if continuous else "180° POSITIONAL")
        type_badge.setObjectName("TypeBadgeCont" if continuous else "TypeBadgePos")
        header.addWidget(type_badge)
        layout.addLayout(header)

        motion_panel = QFrame()
        motion_panel.setObjectName("MotionPanel")
        motion_layout = QVBoxLayout(motion_panel)
        motion_layout.setContentsMargins(10, 8, 10, 8)
        motion_layout.setSpacing(7)
        motion_header = QHBoxLayout()
        motion_title = QLabel("MOTION PROFILE · THIS SERVO")
        motion_title.setObjectName("Kicker")
        motion_header.addWidget(motion_title)
        motion_header.addStretch(1)
        motion_enabled = QCheckBox("Enabled")
        motion_enabled.setToolTip(
            "When disabled, commands move directly to the requested target without ramping."
        )
        motion_enabled.setChecked(self.settings.value(
            f"motion/{channel}/enabled", True, type=bool
        ))
        motion_header.addWidget(motion_enabled)
        motion_layout.addLayout(motion_header)

        motion_fields = QHBoxLayout()
        motion_fields.setSpacing(8)
        speed_field = QVBoxLayout()
        speed_label = QLabel("Pulse rate" if continuous else "Max speed")
        speed_label.setObjectName("Description")
        speed_field.addWidget(speed_label)
        speed_box = QSpinBox()
        speed_box.setRange(MIN_MAX_SPEED_DEG_S, MAX_MAX_SPEED_DEG_S)
        speed_box.setSuffix(" cmd°/s" if continuous else " °/s")
        old_speed = self.settings.value(
            "motion/max_speed_deg_s", DEFAULT_MAX_SPEED_DEG_S, type=int
        )
        speed_box.setValue(self.settings.value(
            f"motion/{channel}/max_speed_deg_s", old_speed, type=int
        ))
        speed_box.setEnabled(motion_enabled.isChecked())
        speed_field.addWidget(speed_box)
        motion_fields.addLayout(speed_field)

        acceleration_field = QVBoxLayout()
        acceleration_label = QLabel("Pulse accel." if continuous else "Acceleration")
        acceleration_label.setObjectName("Description")
        acceleration_field.addWidget(acceleration_label)
        acceleration_box = QSpinBox()
        acceleration_box.setRange(
            MIN_ACCELERATION_DEG_S2, MAX_ACCELERATION_DEG_S2
        )
        acceleration_box.setSuffix(" cmd°/s²" if continuous else " °/s²")
        old_acceleration = self.settings.value(
            "motion/acceleration_deg_s2", DEFAULT_ACCELERATION_DEG_S2, type=int
        )
        acceleration_box.setValue(self.settings.value(
            f"motion/{channel}/acceleration_deg_s2", old_acceleration, type=int
        ))
        acceleration_box.setEnabled(motion_enabled.isChecked())
        acceleration_field.addWidget(acceleration_box)
        motion_fields.addLayout(acceleration_field)
        motion_fields.addStretch(1)
        motion_layout.addLayout(motion_fields)

        apply_profile = QPushButton("APPLY THIS SERVO PROFILE")
        apply_profile.setObjectName("Quick")
        apply_profile.clicked.connect(
            lambda checked=False, ch=channel: self._apply_motion_profile(ch)
        )
        motion_layout.addWidget(apply_profile)
        motion_note = QLabel(
            "Disabled: target commands are sent directly, without a ramp."
            if not motion_enabled.isChecked()
            else (
                "This ramps the pulse command, not rotation RPM. Releasing the direction button stops immediately."
                if continuous
                else "Ramps the commanded angle and eases into the target."
            )
        )
        motion_note.setObjectName("Description")
        motion_note.setWordWrap(True)
        motion_layout.addWidget(motion_note)
        layout.addWidget(motion_panel)
        self.motion_enabled_boxes[channel] = motion_enabled
        self.motion_speed_spins[channel] = speed_box
        self.motion_acceleration_spins[channel] = acceleration_box
        self.motion_notes[channel] = motion_note
        motion_enabled.toggled.connect(
            lambda enabled, ch=channel: self._motion_enabled_changed(ch, enabled)
        )

        defaults = DEFAULT_CONTINUOUS_CALIBRATION_US if continuous else DEFAULT_CALIBRATION_US
        old_min = self.settings.value(f"servo/{channel}/min_us", defaults[0], type=int)
        old_max = self.settings.value(f"servo/{channel}/max_us", defaults[2], type=int)
        old_center = self.settings.value(f"servo/{channel}/center_us", (old_min + old_max) // 2, type=int)
        values = (old_min, old_center, old_max)
        migration_key = f"servo/{channel}/continuous_defaults_v3"
        if continuous and not self.settings.value(migration_key, False, type=bool):
            if values == DEFAULT_CALIBRATION_US:
                values = defaults
                for key, value in zip(("min_us", "center_us", "max_us"), values):
                    self.settings.setValue(f"servo/{channel}/{key}", value)
            self.settings.setValue(migration_key, True)
            self.settings.sync()

        pulse_panel = QFrame()
        pulse_panel.setObjectName("SubPanel")
        pulse_layout = QVBoxLayout(pulse_panel)
        pulse_layout.setContentsMargins(10, 8, 10, 8)
        pulse_layout.setSpacing(8)
        pulse_kicker = QLabel("PULSE WIDTH MAPPING (µs)")
        pulse_kicker.setObjectName("Kicker")
        pulse_layout.addWidget(pulse_kicker)

        row = QHBoxLayout()
        row.setSpacing(8)
        groups: list[QSpinBox] = []
        point_names = (
            ("REVERSE · MIN", "STOP · CENTER", "FORWARD · MAX")
            if continuous else ("0° · MIN", "90° · CENTER", "180° · MAX")
        )
        for value, point_name in zip(values, point_names):
            group = QVBoxLayout()
            group.setSpacing(4)
            point_label = QLabel(point_name)
            point_label.setObjectName("SmallCaps")
            group.addWidget(point_label)
            box = QSpinBox()
            box.setRange(PULSE_MIN_ALLOWED_US, PULSE_MAX_ALLOWED_US)
            box.setSingleStep(CALIBRATION_STEP_US)
            box.setSuffix(" µs")
            box.setValue(max(PULSE_MIN_ALLOWED_US, min(PULSE_MAX_ALLOWED_US, value)))
            box.setToolTip(
                f"Allowed software range: {PULSE_MIN_ALLOWED_US}–{PULSE_MAX_ALLOWED_US} µs"
            )
            group.addWidget(box)
            row.addLayout(group)
            groups.append(box)
        pulse_layout.addLayout(row)
        self.cal_inputs[channel] = (groups[0], groups[1], groups[2])

        tests = QHBoxLayout()
        tests.setSpacing(6)
        test_buttons: list[QPushButton] = []
        for angle, text in zip((MIN_ANGLE, DEFAULT_ANGLE, MAX_ANGLE), point_names):
            button = QPushButton(f"TEST {text.split(' · ')[0]}")
            button.setObjectName("Quick")
            button.clicked.connect(
                lambda checked=False, ch=channel, value=angle: self._test_calibration_point(ch, value)
            )
            tests.addWidget(button)
            test_buttons.append(button)
        self.cal_test_buttons[channel] = (test_buttons[0], test_buttons[1], test_buttons[2])
        pulse_layout.addLayout(tests)
        layout.addWidget(pulse_panel)

        actions = QHBoxLayout()
        actions.setSpacing(8)
        reset_btn = QPushButton("RESET DEFAULTS")
        reset_btn.setObjectName("Quick")
        reset_btn.setToolTip("Restore factory default µs values into the fields above")
        reset_btn.clicked.connect(
            lambda checked=False, ch=channel: self._reset_calibration_defaults(ch)
        )
        actions.addWidget(reset_btn)
        cancel = QPushButton("CANCEL / 90°")
        cancel.setObjectName("Quick")
        cancel.clicked.connect(lambda checked=False, ch=channel: self._cancel_calibration(ch))
        actions.addWidget(cancel)
        save = QPushButton("SAVE / APPLY")
        save.setObjectName("Primary")
        save.clicked.connect(lambda checked=False, ch=channel: self._apply_calibration(ch))
        actions.addWidget(save)
        layout.addLayout(actions)

        note = QLabel("START VALUES ARE A GUIDE. STOP IF THE SERVO BUZZES OR MEETS RESISTANCE.")
        note.setObjectName("Description")
        note.setWordWrap(True)
        self.cal_notes[channel] = note
        layout.addWidget(note)
        return panel

    def _reset_calibration_defaults(self, channel: int) -> None:
        defaults = (
            DEFAULT_CONTINUOUS_CALIBRATION_US
            if channel in CONTINUOUS_SERVO_CHANNELS
            else DEFAULT_CALIBRATION_US
        )
        for box, value in zip(self.cal_inputs[channel], defaults):
            box.setValue(value)
        self.cal_notes[channel].setText(
            f"DEFAULTS LOADED ({defaults[0]} / {defaults[1]} / {defaults[2]} µs). "
            "CLICK SAVE / APPLY TO PERSIST."
        )

    def refresh_ports(self) -> None:
        previous = self.ports.currentData()
        self.ports.clear()
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            self.ports.addItem(f"{port.device}  ·  {port.description}", port.device)
        if not ports:
            self.ports.addItem("NO SERIAL PORT FOUND", None)
        if previous:
            index = self.ports.findData(previous)
            if index >= 0:
                self.ports.setCurrentIndex(index)
        else:
            for index in range(self.ports.count()):
                path = self.ports.itemData(index) or ""
                if "usbmodem" in path or "usbserial" in path or "ttyACM" in path:
                    self.ports.setCurrentIndex(index)
                    break
        self.connect_button.setEnabled(self.ports.currentData() is not None and self.worker is None)

    def _toggle_connection(self) -> None:
        if self.worker is not None:
            self.connection_note.setText("DISCONNECTING …")
            self.connect_button.setEnabled(False)
            self.worker.stop(self._continuous_stop_commands())
            return
        port = self.ports.currentData()
        if not port:
            return
        self.arduino_ready = False
        self._last_error = ""
        self._reported_servo_count = None
        self._set_status("Waiting", "■ CONNECTING")
        self.connection_note.setText("OPENING PORT AND WAITING FOR ARDUINO …")
        self.connect_button.setEnabled(False)
        self.ports.setEnabled(False)
        self.refresh_button.setEnabled(False)
        self.worker = SerialWorker(port, self)
        self.worker.port_opened.connect(self._port_opened)
        self.worker.ready.connect(self._arduino_is_ready)
        self.worker.received.connect(self._received)
        self.worker.failed.connect(self._failed)
        self.worker.finished.connect(self._worker_finished)
        self.worker.start()

    def _port_opened(self, port: str) -> None:
        self.connection_note.setText(f"PORT OPEN: {port}  /  WAITING FOR READY …")
        self.connect_button.setText("DISCONNECT")
        self.connect_button.setObjectName("Disconnect")
        self._repolish(self.connect_button)
        self.connect_button.setEnabled(True)

    def _arduino_is_ready(self) -> None:
        if not self.worker or not self.worker.isRunning():
            return
        self.arduino_ready = True
        self._last_pong = time.monotonic()
        self._set_status("Good", "■ CONNECTED")
        if (
            self._reported_servo_count is not None
            and self._reported_servo_count != len(SERVO_CHANNELS)
        ):
            self.connection_note.setText(
                f"WARNING: ARDUINO REPORTS {self._reported_servo_count} SERVOS, "
                f"APP EXPECTS {len(SERVO_CHANNELS)}. RE-UPLOAD FIRMWARE."
            )
        else:
            self.connection_note.setText("ARDUINO READY. SERVO CONTROLS ARE ACTIVE.")
        for channel in self.cards:
            low, center, high = self._calibration_values(channel)
            self.worker.send(set_calibration(channel, low, center, high))
            self.worker.send(set_motion_profile(
                channel,
                self.motion_enabled_boxes[channel].isChecked(),
                self.motion_speed_spins[channel].value(),
                self.motion_acceleration_spins[channel].value(),
            ))
            target_angle = (
                DEFAULT_ANGLE
                if channel in CONTINUOUS_SERVO_CHANNELS
                else self.cards[channel].angle()
            )
            self.cards[channel].set_angle(target_angle)
            self.pending[channel] = target_angle
        self._flush_pending()

    def _apply_motion_profile(self, channel: int) -> None:
        enabled = self.motion_enabled_boxes[channel].isChecked()
        speed = self.motion_speed_spins[channel].value()
        acceleration = self.motion_acceleration_spins[channel].value()
        self.settings.setValue(f"motion/{channel}/enabled", enabled)
        self.settings.setValue(f"motion/{channel}/max_speed_deg_s", speed)
        self.settings.setValue(f"motion/{channel}/acceleration_deg_s2", acceleration)
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.motion_notes[channel].setText("Could not save this motion profile.")
            return
        if self.arduino_ready and self.worker:
            self.worker.send(set_motion_profile(channel, enabled, speed, acceleration))
            self.motion_notes[channel].setText(self._motion_profile_message(
                channel, enabled, speed, acceleration, active=True
            ))
        else:
            self.motion_notes[channel].setText(self._motion_profile_message(
                channel, enabled, speed, acceleration, active=False
            ))

    def _motion_enabled_changed(self, channel: int, enabled: bool) -> None:
        self.motion_speed_spins[channel].setEnabled(enabled)
        self.motion_acceleration_spins[channel].setEnabled(enabled)
        self._apply_motion_profile(channel)

    @staticmethod
    def _motion_profile_message(
        channel: int, enabled: bool, speed: int, acceleration: int, active: bool
    ) -> str:
        status = "Active" if active else "Saved for the next connection"
        if not enabled:
            return f"{status}: motion ramp disabled; targets are sent directly."
        if channel in CONTINUOUS_SERVO_CHANNELS:
            unit = "cmd°/s pulse rate"
            acceleration_unit = "cmd°/s² pulse acceleration"
        else:
            unit = "°/s max speed"
            acceleration_unit = "°/s² acceleration"
        return (
            f"{status}: {speed} {unit}, {acceleration} {acceleration_unit}."
        )

    def _heartbeat(self) -> None:
        if not self.worker or not self.arduino_ready:
            return
        self.worker.send("PING\n")
        if time.monotonic() - self._last_pong > 4.0:
            self._set_status("Bad", "■ NO RESPONSE")
            self.connection_note.setText(
                "ARDUINO IS NOT RESPONDING TO PING. DISCONNECT AND RECONNECT."
            )
            for card in self.cards.values():
                card.set_offline()

    def _apply_calibration(self, channel: int) -> None:
        low, center, high = self._calibration_values(channel)
        error = ServoCalibration(low, center, high).validate()
        if error:
            self.cal_notes[channel].setText(f"ERROR: {error}")
            return
        if (
            self.arduino_ready
            and channel in CONTINUOUS_SERVO_CHANNELS
            and self.cards[channel].angle() != DEFAULT_ANGLE
        ):
            self.cal_notes[channel].setText("STOP THE CONTINUOUS SERVO BEFORE CALIBRATING.")
            return
        for key, value in zip(("min_us", "center_us", "max_us"), (low, center, high)):
            self.settings.setValue(f"servo/{channel}/{key}", value)
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.cal_notes[channel].setText("COULD NOT SAVE THE CALIBRATION.")
            return
        if self.arduino_ready and self.worker:
            target_angle = (
                DEFAULT_ANGLE
                if channel in CONTINUOUS_SERVO_CHANNELS
                else self.cards[channel].angle()
            )
            self.worker.send(set_calibration(channel, low, center, high))
            self.worker.send(set_angle(channel, target_angle))
            self.cal_notes[channel].setText(f"ACTIVE: {low} / {center} / {high} µs.")
        else:
            self.cal_notes[channel].setText(
                f"SAVED: {low} / {center} / {high} µs. SENT NEXT TIME ARDUINO CONNECTS."
            )

    def _calibration_values(self, channel: int) -> tuple[int, int, int]:
        boxes = self.cal_inputs[channel]
        return boxes[0].value(), boxes[1].value(), boxes[2].value()

    def _test_calibration_point(self, channel: int, angle: int) -> None:
        if not self.arduino_ready or not self.worker:
            self.cal_notes[channel].setText("CONNECT THE ARDUINO BEFORE TESTING.")
            return
        low, center, high = self._calibration_values(channel)
        error = ServoCalibration(low, center, high).validate()
        if error:
            self.cal_notes[channel].setText(f"ERROR: {error}")
            return
        self.worker.send(set_calibration(channel, low, center, high))
        self.cards[channel].set_angle(angle)
        self.pending[channel] = angle
        self.cal_notes[channel].setText(
            f"TESTING {self._servo_command_label(channel, angle)}. ACTIVE MAP: {low} / {center} / {high} µs."
        )

    def _cancel_calibration(self, channel: int) -> None:
        self.cards[channel].set_angle(DEFAULT_ANGLE)
        if self.arduino_ready:
            self.pending[channel] = DEFAULT_ANGLE
        self.cal_notes[channel].setText("TEST CANCELLED. CHANNEL COMMANDED TO 90° / STOP.")

    def _servo_changed(self, channel: int, angle: int) -> None:
        self.pending[channel] = angle

    def _flush_pending(self) -> None:
        if not self.arduino_ready or not self.worker:
            return
        queued = self.pending
        self.pending = {}
        for channel, angle in queued.items():
            self.worker.send(set_angle(channel, angle))

    def _received(self, message: str) -> None:
        if message == "PONG":
            self._last_pong = time.monotonic()
            if self.worker and self.arduino_ready and self.status.objectName() == "Bad":
                self.arduino_ready = True
                self._set_status("Good", "■ CONNECTED")
                self.connection_note.setText("ARDUINO IS RESPONDING AGAIN.")
            return
        kind, values = parse_line(message)
        if kind == "ACK" and len(values) == 2:
            try:
                channel, angle = map(int, values)
            except ValueError:
                return
            if channel in self.cards:
                self.cards[channel].set_ack(angle)
        elif kind == "ACKCAL" and len(values) == 4:
            try:
                channel, low, center, high = map(int, values)
            except ValueError:
                return
            if channel in self.cal_notes:
                self.cal_notes[channel].setText(
                    f"ARDUINO CONFIRMED: {low} / {center} / {high} µs."
                )
        elif kind == "ACKMOTION" and len(values) == 4:
            try:
                channel, enabled, speed, acceleration = map(int, values)
            except ValueError:
                return
            if channel in self.motion_notes:
                self.motion_notes[channel].setText(self._motion_profile_message(
                    channel, bool(enabled), speed, acceleration, active=True
                ))
        elif kind == "READY" and values:
            try:
                reported_count = int(values[0])
            except ValueError:
                return
            self._reported_servo_count = reported_count
            if reported_count != len(SERVO_CHANNELS):
                self.connection_note.setText(
                    f"WARNING: ARDUINO REPORTS {reported_count} SERVOS, "
                    f"APP EXPECTS {len(SERVO_CHANNELS)}. RE-UPLOAD FIRMWARE."
                )
        elif kind == "ERR":
            self.connection_note.setText(f"ARDUINO: {','.join(values)}")

    def _failed(self, reason: str) -> None:
        self._last_error = reason
        self.connection_note.setText(f"CONNECTION ERROR: {reason}")
        self._set_status("Bad", "■ ERROR")

    def _worker_finished(self) -> None:
        self.arduino_ready = False
        self.pending.clear()
        for card in self.cards.values():
            card.reset_continuous_state()
            card.set_offline()
        self._set_status("Bad" if self._last_error else "Off",
                         "■ ERROR" if self._last_error else "■ OFFLINE")
        if not self._last_error:
            self.connection_note.setText("DISCONNECTED. SERVO TARGETS ARE SAVED IN THE APP.")
        self.connect_button.setText("CONNECT")
        self.connect_button.setObjectName("Primary")
        self._repolish(self.connect_button)
        self.ports.setEnabled(True)
        self.refresh_button.setEnabled(True)
        if self.worker:
            self.worker.deleteLater()
            self.worker = None
        self.refresh_ports()

    def _center_all(self) -> None:
        for card in self.cards.values():
            card.set_angle(DEFAULT_ANGLE)
        if self.arduino_ready and self.worker:
            self.pending.clear()
            self.worker.send("CENTER\n")

    @staticmethod
    def _servo_command_label(channel: int, angle: int) -> str:
        if channel in CONTINUOUS_SERVO_CHANNELS:
            return {
                MIN_ANGLE: "REVERSE",
                DEFAULT_ANGLE: "STOP",
                MAX_ANGLE: "FORWARD",
            }.get(angle, f"{angle}°")
        return f"{angle}°"

    def _continuous_stop_commands(self) -> tuple[str, ...]:
        if not self.arduino_ready:
            return ()
        return tuple(set_angle(channel, DEFAULT_ANGLE) for channel in CONTINUOUS_SERVO_CHANNELS)

    def _set_status(self, style: str, text: str) -> None:
        self.status.setObjectName(style)
        self.status.setText(text)
        self._repolish(self.status)

    @staticmethod
    def _repolish(widget: QWidget) -> None:
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def closeEvent(self, event) -> None:
        if self.worker is not None:
            self.worker.stop(self._continuous_stop_commands())
            self.worker.wait(2000)
        super().closeEvent(event)
