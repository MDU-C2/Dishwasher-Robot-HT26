"""PySide6 desktop controller for the dishwasher robot gripper."""

from __future__ import annotations

from collections import deque
import time

import serial.tools.list_ports
from PySide6.QtCore import QPointF, QSettings, QTimer, Qt
from PySide6.QtGui import QColor, QKeySequence, QPainter, QPainterPath, QPen, QShortcut
from PySide6.QtWidgets import (
    QAbstractSpinBox, QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow, QProgressBar, QPushButton,
    QScrollArea, QSlider, QSpinBox, QTabWidget, QVBoxLayout, QWidget,
)

from .config import (
    APP_TITLE, CALIBRATION_STEP_US, COMMAND_INTERVAL_MS, CONTINUOUS_SERVO_CHANNELS,
    DEFAULT_ACCELERATION_DEG_S2, DEFAULT_ANGLE, DEFAULT_CALIBRATION_US,
    DEFAULT_CONTINUOUS_CALIBRATION_US, DEFAULT_FORCE_HOLD_DEADBAND,
    DEFAULT_FSR_CONTACT_THRESHOLD, DEFAULT_FSR_GRIP_THRESHOLD,
    DEFAULT_FSR_MAX_THRESHOLD, DEFAULT_GRIP_TIMEOUT_MS, DEFAULT_LIDAR_DWELL_MS,
    DEFAULT_LIDAR_SLIP_VELOCITY_MM_S, DEFAULT_LIDAR_TRIGGER_MM,
    DEFAULT_MAX_SPEED_DEG_S, DEFAULT_OPTICAL_MOTION_THRESHOLD,
    DEFAULT_OPTICAL_SQUAL_THRESHOLD, DEFAULT_RELEASE_DURATION_MS,
    DEFAULT_SLIP_BOOST_STEP, FSR_PINS, FSR_STATE_LABELS,
    GRIP_CONTROLLER_LABELS, LIDAR_I2C_BUS, LIDAR_STATE_LABELS,
    MAX_ACCELERATION_DEG_S2, MAX_ANGLE, MAX_FORCE_HOLD_DEADBAND,
    MAX_FSR_THRESHOLD, MAX_GRIP_TIMEOUT_MS, MAX_LIDAR_DWELL_MS,
    MAX_LIDAR_SLIP_VELOCITY_MM_S, MAX_LIDAR_TRIGGER_MM, MAX_MAX_SPEED_DEG_S,
    MAX_OPTICAL_MOTION_THRESHOLD, MAX_OPTICAL_SQUAL_THRESHOLD,
    MAX_RELEASE_DURATION_MS, MAX_SLIP_BOOST_STEP, MIN_ACCELERATION_DEG_S2,
    MIN_ANGLE, MIN_FORCE_HOLD_DEADBAND, MIN_FSR_THRESHOLD, MIN_GRIP_TIMEOUT_MS,
    MIN_LIDAR_DWELL_MS, MIN_LIDAR_SLIP_VELOCITY_MM_S, MIN_LIDAR_TRIGGER_MM,
    MIN_MAX_SPEED_DEG_S, MIN_OPTICAL_MOTION_THRESHOLD,
    MIN_OPTICAL_SQUAL_THRESHOLD, MIN_RELEASE_DURATION_MS, MIN_SLIP_BOOST_STEP,
    OPTICAL_SPI_PINS, OPTICAL_STATE_LABELS, PULSE_MAX_ALLOWED_US,
    PULSE_MIN_ALLOWED_US, SERVO_CHANNELS, SLIP_STATE_LABELS,
)
from .calibration import ServoCalibration
from .protocol import (
    grip_command, parse_line, set_angle, set_auto_modes, set_calibration,
    set_fsr_calibration, set_fsr_guard, set_lidar_and_loop_config,
    set_motion_profile, set_optical_calibration, set_optical_led, tare_fsr,
    zero_optical,
)
from .serial_worker import SerialWorker
from .servo_card import ServoCard
from .theme import TOKENS


class DistanceHistoryWidget(QWidget):
    """Compact rolling LiDAR distance plot (~12 s at 10 Hz)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.samples: deque[int] = deque(maxlen=120)
        self.setFixedHeight(56)
        self.setToolTip(
            "Recent VL53L4CD distance measurements (approximately 12 seconds of history)."
        )

    def add_sample(self, distance_mm: int) -> None:
        self.samples.append(distance_mm)
        self.update()

    def clear(self) -> None:
        self.samples.clear()
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(TOKENS["paper_3"]))
        painter.setPen(QPen(QColor(TOKENS["rule"]), 1))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))

        area = self.rect().adjusted(40, 6, -8, -14)
        if area.width() <= 1 or area.height() <= 1:
            painter.end()
            return

        for i in range(3):
            y = area.top() + (area.height() * i / 2.0)
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))

        font = painter.font()
        font.setPointSize(9)
        painter.setFont(font)

        if len(self.samples) < 2:
            painter.setPen(QColor(TOKENS["muted"]))
            painter.drawText(
                self.rect(), Qt.AlignmentFlag.AlignCenter, "Waiting for distance samples"
            )
            painter.end()
            return

        low = min(self.samples)
        high = max(self.samples)
        if high - low < 50:
            low = max(0, low - 25)
            high = low + 50
        else:
            margin = max(10, int((high - low) * 0.08))
            low = max(0, low - margin)
            high += margin
        span = max(1, high - low)

        painter.setPen(QColor(TOKENS["muted"]))
        painter.drawText(4, area.top() + 8, f"{high}")
        painter.drawText(4, area.bottom() + 2, f"{low} mm")

        path = QPainterPath()
        count = len(self.samples)
        for index, value in enumerate(self.samples):
            x = area.left() + (area.width() * index / max(1, count - 1))
            y = area.bottom() - ((value - low) / span) * area.height()
            point = QPointF(x, y)
            if index == 0:
                path.moveTo(point)
            else:
                path.lineTo(point)
        painter.setPen(QPen(QColor(TOKENS["cool"]), 1.8))
        painter.drawPath(path)

        painter.setPen(QColor(TOKENS["muted"]))
        painter.drawText(
            self.rect().adjusted(40, 0, -6, -2),
            Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignRight,
            "12s history -> now",
        )
        painter.end()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1280, 900)
        self.setMinimumSize(940, 680)
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
        self.fsr_contact_spins: dict[int, QSpinBox] = {}
        self.fsr_grip_spins: dict[int, QSpinBox] = {}
        self.fsr_max_spins: dict[int, QSpinBox] = {}
        self.fsr_invert_boxes: dict[int, QCheckBox] = {}
        self.fsr_cal_notes: dict[int, QLabel] = {}
        self._grip_sync_active = False
        self._opt_state = 3
        self._lidar_state = 3
        self._ctrl_state = 0
        self._slip_force_boost = 0
        self._slip_state = 0
        self._slip_confidence = 0
        self.simple_sliders: dict[int, QSlider] = {}
        self.simple_angle_labels: dict[int, QLabel] = {}
        self.simple_fsr_bars: dict[int, QProgressBar] = {}
        self.simple_fsr_labels: dict[int, QLabel] = {}
        self._cal_arm_widgets: list[QWidget] = []
        self._cal_finger_widgets: list[QWidget] = []
        self._cal_sensor_widgets: list[QWidget] = []
        self._cal_filter_buttons: dict[str, QPushButton] = {}
        self._shortcuts: list[QShortcut] = []

        self._build_ui()
        self._setup_shortcuts()
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
        page = QWidget()
        page.setObjectName("Page")
        self.setCentralWidget(page)
        content = QVBoxLayout(page)
        content.setContentsMargins(20, 14, 20, 14)
        content.setSpacing(10)

        toolbar = QFrame()
        toolbar.setObjectName("TopToolbar")
        toolbar_layout = QVBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(16, 10, 16, 10)
        toolbar_layout.setSpacing(4)

        toolbar_row = QHBoxLayout()
        toolbar_row.setSpacing(10)
        title = QLabel("Gripper Control")
        title.setObjectName("Hero")
        toolbar_row.addWidget(title)

        self.status = QLabel("Offline")
        self.status.setObjectName("Off")
        toolbar_row.addWidget(self.status)
        toolbar_row.addStretch(1)

        self.ports = QComboBox()
        self.ports.setMinimumWidth(240)
        toolbar_row.addWidget(self.ports)
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)
        toolbar_row.addWidget(self.refresh_button)
        self.connect_button = QPushButton("Connect")
        self.connect_button.setObjectName("ConnectAction")
        self.connect_button.setMinimumWidth(104)
        self.connect_button.clicked.connect(self._toggle_connection)
        toolbar_row.addWidget(self.connect_button)
        global_center = QPushButton("Stop / Center [Esc]")
        global_center.setObjectName("Quick")
        global_center.setToolTip(
            "Stop all continuous finger servos and return positional servos to 90°"
        )
        global_center.clicked.connect(self._center_all)
        toolbar_row.addWidget(global_center)
        toolbar_layout.addLayout(toolbar_row)

        self.connection_note = QLabel("Select a port and click Connect.")
        self.connection_note.setObjectName("Description")
        toolbar_layout.addWidget(self.connection_note)
        content.addWidget(toolbar)

        self.tabs = QTabWidget()
        self.tabs.setObjectName("MainTabs")
        content.addWidget(self.tabs, 1)

        positional_channels = [
            ch for ch in SERVO_CHANNELS if ch not in CONTINUOUS_SERVO_CHANNELS
        ]
        continuous_channels = [
            ch for ch in SERVO_CHANNELS if ch in CONTINUOUS_SERVO_CHANNELS
        ]

        # Tab: Advanced Servos (build cards before Simple Control binds to them)
        control_page = QWidget()
        control_layout = QVBoxLayout(control_page)
        control_layout.setContentsMargins(2, 10, 2, 10)
        control_layout.setSpacing(12)
        section_row = QHBoxLayout()
        section_title = QLabel("Manual Servo Control")
        section_title.setObjectName("SectionTitle")
        section_row.addWidget(section_title)
        section_row.addStretch(1)
        center = QPushButton("Center All (90°)")
        center.setObjectName("Quick")
        center.setToolTip("Moves all positional servos to 90° and stops continuous servos")
        center.clicked.connect(self._center_all)
        section_row.addWidget(center)
        control_layout.addLayout(section_row)

        control_layout.addWidget(
            self._build_section_banner(
                "Positional Servos (0°–180°)",
                f"{len(positional_channels)} channels, slider, fine nudge, and angle presets",
                continuous=False,
            )
        )
        pos_grid = QGridLayout()
        pos_grid.setHorizontalSpacing(12)
        pos_grid.setVerticalSpacing(12)
        pos_grid.setColumnStretch(0, 1)
        pos_grid.setColumnStretch(1, 1)
        for index, channel in enumerate(positional_channels):
            card = ServoCard(channel, SERVO_CHANNELS[channel], continuous=False)
            card.changed.connect(self._servo_changed)
            card.changed.connect(self._sync_simple_slider)
            self.cards[channel] = card
            pos_grid.addWidget(card, index // 2, index % 2)
        control_layout.addLayout(pos_grid)

        control_layout.addSpacing(4)
        control_layout.addWidget(
            self._build_section_banner(
                "Continuous Finger Drives (360°)",
                f"{len(continuous_channels)} channels, hold Reverse or Forward to drive",
                continuous=True,
            )
        )
        control_layout.addWidget(self._build_grip_master_panel())
        cont_grid = QGridLayout()
        cont_grid.setHorizontalSpacing(12)
        cont_grid.setVerticalSpacing(12)
        cont_grid.setColumnStretch(0, 1)
        cont_grid.setColumnStretch(1, 1)
        for index, channel in enumerate(continuous_channels):
            card = ServoCard(channel, SERVO_CHANNELS[channel], continuous=True)
            card.changed.connect(self._servo_changed)
            self.cards[channel] = card
            cont_grid.addWidget(card, index // 2, index % 2)
        control_layout.addLayout(cont_grid)
        control_layout.addStretch(1)

        # Tab: Calibration (built before Simple Control reads saved thresholds)
        calibration_page = QWidget()
        calibration_layout = QVBoxLayout(calibration_page)
        calibration_layout.setContentsMargins(2, 10, 2, 10)
        calibration_layout.setSpacing(12)

        cal_intro = QFrame()
        cal_intro.setObjectName("Panel")
        intro_layout = QVBoxLayout(cal_intro)
        intro_layout.setContentsMargins(16, 12, 16, 12)
        intro_layout.setSpacing(8)

        cal_header_row = QHBoxLayout()
        cal_header_row.setSpacing(8)
        intro_title = QLabel("Calibration & Sensor Tuning")
        intro_title.setObjectName("SectionTitle")
        cal_header_row.addWidget(intro_title)
        cal_header_row.addStretch(1)

        filter_specs = (
            ("all", "All"),
            ("arm", "Arm Servos (0–4)"),
            ("fingers", "Fingers && FSR (5–7)"),
            ("sensors", "LiDAR && Optical"),
        )
        for key, label_text in filter_specs:
            f_btn = QPushButton(label_text)
            f_btn.setObjectName("FilterTab")
            f_btn.setProperty("active", key == "all")
            f_btn.clicked.connect(lambda checked=False, cat=key: self._set_calibration_filter(cat))
            cal_header_row.addWidget(f_btn)
            self._cal_filter_buttons[key] = f_btn
        intro_layout.addLayout(cal_header_row)

        intro_text = QLabel(
            "Adjust pulse limits (µs), speed ramps, FSR force thresholds, optical slip thresholds, "
            "and closed-loop tuning. Save settings and test individual points without load."
        )
        intro_text.setObjectName("Description")
        intro_text.setWordWrap(True)
        intro_layout.addWidget(intro_text)
        calibration_layout.addWidget(cal_intro)

        arm_cal_section = QWidget()
        arm_cal_layout = QVBoxLayout(arm_cal_section)
        arm_cal_layout.setContentsMargins(0, 0, 0, 0)
        arm_cal_layout.setSpacing(12)
        arm_cal_layout.addWidget(
            self._build_section_banner(
                "Positional Servos Calibration",
                f"{len(positional_channels)} channels, 0° / 90° / 180° pulse mapping",
                continuous=False,
            )
        )
        pos_cal_grid = QGridLayout()
        pos_cal_grid.setHorizontalSpacing(12)
        pos_cal_grid.setVerticalSpacing(12)
        pos_cal_grid.setColumnStretch(0, 1)
        pos_cal_grid.setColumnStretch(1, 1)
        for index, channel in enumerate(positional_channels):
            pos_cal_grid.addWidget(self._build_calibration_card(channel), index // 2, index % 2)
        arm_cal_layout.addLayout(pos_cal_grid)
        calibration_layout.addWidget(arm_cal_section)
        self._cal_arm_widgets.append(arm_cal_section)

        finger_cal_section = QWidget()
        finger_cal_layout = QVBoxLayout(finger_cal_section)
        finger_cal_layout.setContentsMargins(0, 0, 0, 0)
        finger_cal_layout.setSpacing(12)
        finger_cal_layout.addWidget(
            self._build_section_banner(
                "Finger Drives & FSR Calibration",
                f"{len(continuous_channels)} channels, reverse / stop / forward and force limits",
                continuous=True,
            )
        )
        cont_cal_grid = QGridLayout()
        cont_cal_grid.setHorizontalSpacing(12)
        cont_cal_grid.setVerticalSpacing(12)
        cont_cal_grid.setColumnStretch(0, 1)
        cont_cal_grid.setColumnStretch(1, 1)
        for index, channel in enumerate(continuous_channels):
            cont_cal_grid.addWidget(self._build_calibration_card(channel), index // 2, index % 2)
        finger_cal_layout.addLayout(cont_cal_grid)
        calibration_layout.addWidget(finger_cal_section)
        self._cal_finger_widgets.append(finger_cal_section)

        sensor_cal_section = QWidget()
        sensor_cal_layout = QVBoxLayout(sensor_cal_section)
        sensor_cal_layout.setContentsMargins(0, 0, 0, 0)
        sensor_cal_layout.setSpacing(12)
        sensor_cal_layout.addWidget(
            self._build_section_banner(
                "Optical Slip Sensor (PAA5100JE)",
                f"SPI {OPTICAL_SPI_PINS}, movement detection thresholds",
                continuous=True,
                badge_text="SPI optical",
            )
        )
        sensor_cal_layout.addWidget(self._build_optical_calibration_card())
        sensor_cal_layout.addWidget(
            self._build_section_banner(
                "Palm LiDAR & Closed-Loop Regulator (VL53L4CD)",
                f"{LIDAR_I2C_BUS}, proximity trigger, dwell, release and slip boost tuning",
                continuous=True,
                badge_text="I2C-1 regulator",
            )
        )
        sensor_cal_layout.addWidget(self._build_lidar_loop_calibration_card())
        calibration_layout.addWidget(sensor_cal_section)
        self._cal_sensor_widgets.append(sensor_cal_section)
        calibration_layout.addStretch(1)

        # Tab 1: Simple Control
        simple_page = self._build_simple_control_page(positional_channels, continuous_channels)

        self.tabs.addTab(self._wrap_scroll(simple_page), "Simple Control")
        self.tabs.addTab(self._wrap_scroll(control_page), "Advanced Servos")
        self.tabs.addTab(self._wrap_scroll(calibration_page), "Calibration")

        footer = QLabel(
            "115200 baud   |   Space / Esc: Stop fingers   |   G: Grip   |   R: Release   |   T: Tare FSRs   |   1–3: Recall pose"
        )
        footer.setObjectName("Tick")
        content.addWidget(footer)

    @staticmethod
    def _wrap_scroll(inner: QWidget) -> QScrollArea:
        inner.setObjectName("Page")
        scroll = QScrollArea()
        scroll.setObjectName("AppScroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(inner)
        return scroll

    @staticmethod
    def _build_section_banner(
        title: str,
        subtitle: str,
        continuous: bool = False,
        badge_text: str | None = None,
    ) -> QFrame:
        banner = QFrame()
        banner.setObjectName("SectionBanner")
        banner.setProperty("continuous", continuous)
        layout = QHBoxLayout(banner)
        layout.setContentsMargins(2, 6, 2, 8)
        layout.setSpacing(12)
        title_label = QLabel(title)
        title_label.setObjectName("SectionTitle")
        layout.addWidget(title_label)
        subtitle_label = QLabel(subtitle)
        subtitle_label.setObjectName("Description")
        layout.addWidget(subtitle_label)
        layout.addStretch(1)
        badge = QLabel(badge_text or ("360° continuous" if continuous else "180° positional"))
        badge.setObjectName("TypeBadgeCont" if continuous else "TypeBadgePos")
        layout.addWidget(badge)
        return banner

    def _setup_shortcuts(self) -> None:
        bindings: list[tuple[Qt.Key, object, bool]] = [
            (Qt.Key.Key_Escape, self._stop_all_fingers, False),
            (Qt.Key.Key_Space, self._stop_all_fingers, True),
            (Qt.Key.Key_G, self._trigger_auto_grip, True),
            (Qt.Key.Key_R, self._trigger_timed_release, True),
            (Qt.Key.Key_T, self._tare_fsrs, True),
            (Qt.Key.Key_1, lambda: self._apply_arm_pose(1), True),
            (Qt.Key.Key_2, lambda: self._apply_arm_pose(2), True),
            (Qt.Key.Key_3, lambda: self._apply_arm_pose(3), True),
        ]
        for key, callback, check_focus in bindings:
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
            if check_focus:
                shortcut.activated.connect(
                    lambda fn=callback: self._run_if_no_text_focus(fn)
                )
            else:
                shortcut.activated.connect(callback)
            self._shortcuts.append(shortcut)

    def _run_if_no_text_focus(self, callback) -> None:
        focused = self.focusWidget()
        if isinstance(focused, (QAbstractSpinBox, QLineEdit)):
            return
        callback()

    def _set_calibration_filter(self, category: str) -> None:
        show_all = category == "all"
        for widget in self._cal_arm_widgets:
            widget.setVisible(show_all or category == "arm")
        for widget in self._cal_finger_widgets:
            widget.setVisible(show_all or category == "fingers")
        for widget in self._cal_sensor_widgets:
            widget.setVisible(show_all or category == "sensors")
        for key, button in self._cal_filter_buttons.items():
            button.setProperty("active", key == category)
            self._repolish(button)

    def _nudge_simple_servo(self, channel: int, delta: int) -> None:
        if channel in self.cards:
            self.cards[channel].set_angle(self.cards[channel].angle() + delta)

    def _build_grip_master_panel(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("GripMasterPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(10)

        top = QHBoxLayout()
        top.setSpacing(10)
        title = QLabel("Master 3-Finger Grip & FSR Control")
        title.setObjectName("SectionTitle")
        top.addWidget(title)
        top.addStretch(1)

        self.fsr_guard_checkbox = QCheckBox("FSR Auto-Stop Guard")
        self.fsr_guard_checkbox.setToolTip(
            "Automatically stops each finger in closing direction when its FSR hits the grip limit"
        )
        self.fsr_guard_checkbox.setChecked(
            self.settings.value("fsr/guard_enabled", True, type=bool)
        )
        self.fsr_guard_checkbox.toggled.connect(self._toggle_fsr_guard)
        top.addWidget(self.fsr_guard_checkbox)

        tare_btn = QPushButton("Tare FSRs (Unloaded)")
        tare_btn.setObjectName("Quick")
        tare_btn.setToolTip("Set current unloaded FSR readings as zero baseline on all 3 fingers")
        tare_btn.clicked.connect(self._tare_fsrs)
        top.addWidget(tare_btn)
        layout.addLayout(top)

        actions = QHBoxLayout()
        actions.setSpacing(8)
        auto_grip_btn = QPushButton("Auto-Grip to FSR Limit")
        auto_grip_btn.setObjectName("Primary")
        auto_grip_btn.setToolTip(
            "Closes all 3 fingers simultaneously, slows on contact, and stops each finger at its FSR limit"
        )
        auto_grip_btn.clicked.connect(self._trigger_auto_grip)
        actions.addWidget(auto_grip_btn, 2)

        hold_open_btn = QPushButton("Hold Open All")
        hold_open_btn.setObjectName("HoldReverse")
        hold_open_btn.setToolTip("Hold to open all 3 fingers simultaneously; release to stop")
        hold_open_btn.pressed.connect(lambda: self._grip_hold_action("OPEN_HOLD"))
        hold_open_btn.released.connect(lambda: self._grip_hold_action("STOP"))
        actions.addWidget(hold_open_btn, 2)

        hold_close_btn = QPushButton("Hold Close All")
        hold_close_btn.setObjectName("HoldForward")
        hold_close_btn.setToolTip(
            "Hold to close all 3 fingers simultaneously (FSR auto-stop stops each finger at its limit)"
        )
        hold_close_btn.pressed.connect(lambda: self._grip_hold_action("CLOSE_HOLD"))
        hold_close_btn.released.connect(lambda: self._grip_hold_action("STOP"))
        actions.addWidget(hold_close_btn, 2)

        stop_btn = QPushButton("Stop Fingers")
        stop_btn.setObjectName("StopButton")
        stop_btn.setToolTip("Immediately stop all 3 continuous finger servos")
        stop_btn.clicked.connect(self._stop_all_fingers)
        actions.addWidget(stop_btn, 1)
        layout.addLayout(actions)

        self.grip_master_status = QLabel(
            "Connect Arduino to control all 3 fingers together with live FSR pressure limits."
        )
        self.grip_master_status.setObjectName("Description")
        self.grip_master_status.setWordWrap(True)
        layout.addWidget(self.grip_master_status)

        opt_panel = QFrame()
        opt_panel.setObjectName("SubPanel")
        opt_layout = QVBoxLayout(opt_panel)
        opt_layout.setContentsMargins(10, 8, 10, 8)
        opt_layout.setSpacing(6)

        opt_header = QHBoxLayout()
        opt_header.setSpacing(8)
        opt_title = QLabel("PAA5100JE Optical Flow Monitor")
        opt_title.setObjectName("Kicker")
        opt_header.addWidget(opt_title)
        opt_header.addStretch(1)

        self.opt_led_checkbox = QCheckBox("Sensor LED")
        self.opt_led_checkbox.setToolTip(
            "Toggle the white illumination LEDs on the PAA5100JE breakout"
        )
        self.opt_led_checkbox.setChecked(
            self.settings.value("optical/led_enabled", True, type=bool)
        )
        self.opt_led_checkbox.toggled.connect(self._toggle_optical_led)
        opt_header.addWidget(self.opt_led_checkbox)

        zero_opt_btn = QPushButton("Zero X/Y")
        zero_opt_btn.setObjectName("Quick")
        zero_opt_btn.setToolTip("Reset accumulated Total X and Total Y movement counters to zero")
        zero_opt_btn.clicked.connect(self._zero_optical)
        opt_header.addWidget(zero_opt_btn)

        self.opt_state_badge = QLabel(OPTICAL_STATE_LABELS[3])
        self.opt_state_badge.setObjectName("OptState3")
        opt_header.addWidget(self.opt_state_badge)
        opt_layout.addLayout(opt_header)

        saved_motion_thr = max(
            MIN_OPTICAL_MOTION_THRESHOLD,
            min(
                MAX_OPTICAL_MOTION_THRESHOLD,
                self.settings.value(
                    "optical/motion_threshold_v2", DEFAULT_OPTICAL_MOTION_THRESHOLD, type=int
                ),
            ),
        )
        saved_squal_thr = max(
            MIN_OPTICAL_SQUAL_THRESHOLD,
            min(
                MAX_OPTICAL_SQUAL_THRESHOLD,
                self.settings.value(
                    "optical/squal_threshold_v2", DEFAULT_OPTICAL_SQUAL_THRESHOLD, type=int
                ),
            ),
        )

        self.opt_mag_bar = QProgressBar()
        self.opt_mag_bar.setRange(0, saved_motion_thr)
        self.opt_mag_bar.setValue(0)
        self.opt_mag_bar.setTextVisible(False)
        self.opt_mag_bar.setProperty("optState", "3")
        opt_layout.addWidget(self.opt_mag_bar)

        self.opt_value_label = QLabel(
            self._format_optical_readout(0, 0, 0, 0, 0, saved_motion_thr, saved_squal_thr)
        )
        self.opt_value_label.setObjectName("Description")
        self.opt_value_label.setWordWrap(True)
        opt_layout.addWidget(self.opt_value_label)

        adv_slip_head = QHBoxLayout()
        adv_slip_head.setSpacing(8)
        adv_slip_title = QLabel("Slip Fusion (Opt + LiDAR + FSR)")
        adv_slip_title.setObjectName("Kicker")
        adv_slip_head.addWidget(adv_slip_title)
        adv_slip_head.addStretch(1)
        self.adv_slip_badge = QLabel(SLIP_STATE_LABELS[0])
        self.adv_slip_badge.setObjectName("SlipState0")
        adv_slip_head.addWidget(self.adv_slip_badge)
        opt_layout.addLayout(adv_slip_head)

        self.adv_slip_bar = QProgressBar()
        self.adv_slip_bar.setRange(0, 100)
        self.adv_slip_bar.setValue(0)
        self.adv_slip_bar.setTextVisible(False)
        self.adv_slip_bar.setProperty("slipState", "0")
        opt_layout.addWidget(self.adv_slip_bar)

        self.adv_slip_label = QLabel("Evidence: 0% (Opt: still, LiDAR: still)")
        self.adv_slip_label.setObjectName("Description")
        opt_layout.addWidget(self.adv_slip_label)

        layout.addWidget(opt_panel)
        return panel

    def _build_optical_calibration_card(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("CalibrationCard")
        panel.setProperty("continuous", True)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        header = QHBoxLayout()
        header.setSpacing(8)
        tag = QLabel("SPI")
        tag.setObjectName("ChannelTag")
        header.addWidget(tag)
        name = QLabel("PAA5100JE Optical Flow Sensor")
        name.setObjectName("ServoName")
        header.addWidget(name)
        header.addStretch(1)
        badge = QLabel("Optical Slip")
        badge.setObjectName("TypeBadgeCont")
        header.addWidget(badge)
        layout.addLayout(header)

        sub = QFrame()
        sub.setObjectName("SubPanel")
        sub_layout = QVBoxLayout(sub)
        sub_layout.setContentsMargins(10, 8, 10, 8)
        sub_layout.setSpacing(8)
        kicker = QLabel("Movement Activation Thresholds (Triggers Closed-Loop Slip Boost)")
        kicker.setObjectName("Kicker")
        sub_layout.addWidget(kicker)

        fields = QHBoxLayout()
        fields.setSpacing(10)

        motion_col = QVBoxLayout()
        motion_col.setSpacing(4)
        motion_lbl = QLabel("Movement Threshold (Mag)")
        motion_lbl.setObjectName("SmallCaps")
        motion_col.addWidget(motion_lbl)
        self.opt_motion_spin = QSpinBox()
        self.opt_motion_spin.setRange(MIN_OPTICAL_MOTION_THRESHOLD, MAX_OPTICAL_MOTION_THRESHOLD)
        self.opt_motion_spin.setSingleStep(1)
        saved_motion = self.settings.value(
            "optical/motion_threshold_v2", DEFAULT_OPTICAL_MOTION_THRESHOLD, type=int
        )
        self.opt_motion_spin.setValue(
            max(MIN_OPTICAL_MOTION_THRESHOLD, min(MAX_OPTICAL_MOTION_THRESHOLD, saved_motion))
        )
        self.opt_motion_spin.setToolTip(
            "Minimum optical motion magnitude sqrt(dX^2 + dY^2) required to flag MOVEMENT DETECTED"
        )
        motion_col.addWidget(self.opt_motion_spin)
        fields.addLayout(motion_col)

        squal_col = QVBoxLayout()
        squal_col.setSpacing(4)
        squal_lbl = QLabel("Min Surface Quality (SQUAL)")
        squal_lbl.setObjectName("SmallCaps")
        squal_col.addWidget(squal_lbl)
        self.opt_squal_spin = QSpinBox()
        self.opt_squal_spin.setRange(MIN_OPTICAL_SQUAL_THRESHOLD, MAX_OPTICAL_SQUAL_THRESHOLD)
        self.opt_squal_spin.setSingleStep(5)
        saved_squal = self.settings.value(
            "optical/squal_threshold_v2", DEFAULT_OPTICAL_SQUAL_THRESHOLD, type=int
        )
        self.opt_squal_spin.setValue(
            max(MIN_OPTICAL_SQUAL_THRESHOLD, min(MAX_OPTICAL_SQUAL_THRESHOLD, saved_squal))
        )
        self.opt_squal_spin.setToolTip(
            "Minimum SQUAL (0-255) required so empty-air noise does not trigger MOVEMENT DETECTED"
        )
        squal_col.addWidget(self.opt_squal_spin)
        fields.addLayout(squal_col)
        fields.addStretch(1)
        sub_layout.addLayout(fields)
        layout.addWidget(sub)

        actions = QHBoxLayout()
        actions.setSpacing(8)
        reset_btn = QPushButton("Reset Optical Defaults")
        reset_btn.setObjectName("Quick")
        reset_btn.clicked.connect(self._reset_optical_defaults)
        actions.addWidget(reset_btn)
        apply_btn = QPushButton("Save / Apply Optical")
        apply_btn.setObjectName("Primary")
        apply_btn.clicked.connect(self._apply_optical_calibration)
        actions.addWidget(apply_btn)
        layout.addLayout(actions)

        self.opt_cal_note = QLabel(
            f"SPI pins: {OPTICAL_SPI_PINS}. Connect Arduino and move a surface 15–35 mm below the lens to tune."
        )
        self.opt_cal_note.setObjectName("Description")
        self.opt_cal_note.setWordWrap(True)
        layout.addWidget(self.opt_cal_note)
        return panel

    def _build_lidar_loop_calibration_card(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("CalibrationCard")
        panel.setProperty("continuous", True)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        header = QHBoxLayout()
        header.setSpacing(8)
        tag = QLabel("I2C-1")
        tag.setObjectName("ChannelTag")
        header.addWidget(tag)
        name = QLabel("Palm LiDAR & Closed-Loop Regulator")
        name.setObjectName("ServoName")
        header.addWidget(name)
        header.addStretch(1)
        badge = QLabel("VL53L4CD + Loop")
        badge.setObjectName("TypeBadgeCont")
        header.addWidget(badge)
        layout.addLayout(header)

        sub = QFrame()
        sub.setObjectName("SubPanel")
        sub_layout = QVBoxLayout(sub)
        sub_layout.setContentsMargins(10, 8, 10, 8)
        sub_layout.setSpacing(8)
        kicker = QLabel("Proximity Auto-Grip, Timed Release & Slip Force Regulator")
        kicker.setObjectName("Kicker")
        sub_layout.addWidget(kicker)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)

        def _make_spin(
            key: str,
            default_val: int,
            low_val: int,
            high_val: int,
            step: int,
            suffix: str,
            tooltip: str,
        ) -> QSpinBox:
            spin = QSpinBox()
            spin.setRange(low_val, high_val)
            spin.setSingleStep(step)
            spin.setSuffix(suffix)
            saved = self.settings.value(f"loop/{key}", default_val, type=int)
            spin.setValue(max(low_val, min(high_val, saved)))
            spin.setToolTip(tooltip)
            return spin

        self.loop_trigger_spin = _make_spin(
            "trigger_mm", DEFAULT_LIDAR_TRIGGER_MM, MIN_LIDAR_TRIGGER_MM, MAX_LIDAR_TRIGGER_MM,
            5, " mm", "Distance under which Palm LiDAR starts dwell timer for Auto-Grip",
        )
        self.loop_dwell_spin = _make_spin(
            "dwell_ms", DEFAULT_LIDAR_DWELL_MS, MIN_LIDAR_DWELL_MS, MAX_LIDAR_DWELL_MS,
            50, " ms", "How long an object must stay inside trigger distance before closing",
        )
        self.loop_slip_vel_spin = _make_spin(
            "slip_vel_mm_s", DEFAULT_LIDAR_SLIP_VELOCITY_MM_S,
            MIN_LIDAR_SLIP_VELOCITY_MM_S, MAX_LIDAR_SLIP_VELOCITY_MM_S,
            5, " mm/s", "Minimum downward/away LiDAR speed while holding to flag vertical slip",
        )
        self.loop_release_spin = _make_spin(
            "release_ms", DEFAULT_RELEASE_DURATION_MS, MIN_RELEASE_DURATION_MS, MAX_RELEASE_DURATION_MS,
            100, " ms", "Duration for the timed Release Object open button",
        )
        self.loop_boost_spin = _make_spin(
            "boost_step", DEFAULT_SLIP_BOOST_STEP, MIN_SLIP_BOOST_STEP, MAX_SLIP_BOOST_STEP,
            25, " ADC", "Force setpoint increase added per detected slip event (capped by Max Safety Thr)",
        )
        self.loop_deadband_spin = _make_spin(
            "hold_deadband", DEFAULT_FORCE_HOLD_DEADBAND, MIN_FORCE_HOLD_DEADBAND, MAX_FORCE_HOLD_DEADBAND,
            10, " ADC", "Force drop hysteresis before a finger re-tightens during Closed-Loop Hold",
        )

        self.loop_grip_timeout_spin = QDoubleSpinBox()
        self.loop_grip_timeout_spin.setRange(
            MIN_GRIP_TIMEOUT_MS / 1000.0, MAX_GRIP_TIMEOUT_MS / 1000.0
        )
        self.loop_grip_timeout_spin.setDecimals(1)
        self.loop_grip_timeout_spin.setSingleStep(0.5)
        self.loop_grip_timeout_spin.setSuffix(" s")
        saved_timeout_ms = self.settings.value(
            "loop/grip_timeout_ms", DEFAULT_GRIP_TIMEOUT_MS, type=int
        )
        clamped_timeout_ms = max(MIN_GRIP_TIMEOUT_MS, min(MAX_GRIP_TIMEOUT_MS, saved_timeout_ms))
        self.loop_grip_timeout_spin.setValue(clamped_timeout_ms / 1000.0)
        self.loop_grip_timeout_spin.setToolTip(
            "Maximum closing time in seconds before non-contacting fingers stop"
        )
        self.loop_grip_timeout_spin.valueChanged.connect(self._cal_grip_timeout_changed)

        specs = (
            ("Auto-Grip Distance", self.loop_trigger_spin),
            ("Proximity Dwell", self.loop_dwell_spin),
            ("LiDAR Slip Velocity", self.loop_slip_vel_spin),
            ("Release Open Time", self.loop_release_spin),
            ("Slip Force Boost", self.loop_boost_spin),
            ("Force Hold Deadband", self.loop_deadband_spin),
            ("Max Grip Time", self.loop_grip_timeout_spin),
        )
        for idx, (lbl_text, spin_box) in enumerate(specs):
            col = QVBoxLayout()
            col.setSpacing(4)
            lbl = QLabel(lbl_text)
            lbl.setObjectName("SmallCaps")
            col.addWidget(lbl)
            col.addWidget(spin_box)
            grid.addLayout(col, idx // 3, idx % 3)
        sub_layout.addLayout(grid)
        layout.addWidget(sub)

        actions = QHBoxLayout()
        actions.setSpacing(8)
        reset_btn = QPushButton("Reset Loop Defaults")
        reset_btn.setObjectName("Quick")
        reset_btn.clicked.connect(self._reset_loop_defaults)
        actions.addWidget(reset_btn)
        apply_btn = QPushButton("Save / Apply Loop")
        apply_btn.setObjectName("Primary")
        apply_btn.clicked.connect(self._apply_loop_calibration)
        actions.addWidget(apply_btn)
        layout.addLayout(actions)

        self.lidar_cal_note = QLabel(
            f"{LIDAR_I2C_BUS}: Connect Arduino to monitor live VL53L4CD distance and closed-loop status."
        )
        self.lidar_cal_note.setObjectName("Description")
        self.lidar_cal_note.setWordWrap(True)
        layout.addWidget(self.lidar_cal_note)
        return panel

    def _build_simple_control_page(
        self, positional_channels: list[int], continuous_channels: list[int]
    ) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(2, 8, 2, 8)
        layout.setSpacing(10)

        # 1. Smart Gripper Panel
        smart_panel = QFrame()
        smart_panel.setObjectName("GripMasterPanel")
        smart_layout = QVBoxLayout(smart_panel)
        smart_layout.setContentsMargins(16, 12, 16, 12)
        smart_layout.setSpacing(10)

        top = QHBoxLayout()
        top.setSpacing(10)
        title = QLabel("Smart Gripper")
        title.setObjectName("SectionTitle")
        top.addWidget(title)
        subtitle = QLabel("Auto-proximity & closed-loop slip control")
        subtitle.setObjectName("Description")
        top.addWidget(subtitle)
        top.addStretch(1)
        tare_btn = QPushButton("Tare FSRs [T]")
        tare_btn.setObjectName("Quick")
        tare_btn.setToolTip("Set current unloaded FSR readings as zero baseline")
        tare_btn.clicked.connect(self._tare_fsrs)
        top.addWidget(tare_btn)
        smart_layout.addLayout(top)

        self.simple_ctrl_banner = QLabel(
            f"{GRIP_CONTROLLER_LABELS[0]}   |   Force Boost: +0 ADC"
        )
        self.simple_ctrl_banner.setObjectName("CtrlStateBanner")
        self.simple_ctrl_banner.setProperty("ctrlState", "0")
        smart_layout.addWidget(self.simple_ctrl_banner)

        toggles = QHBoxLayout()
        toggles.setSpacing(20)
        self.auto_lidar_checkbox = QCheckBox("Auto-Grip on Palm LiDAR Distance")
        self.auto_lidar_checkbox.setToolTip(
            "When armed, closes automatically after an object stays within trigger distance for dwell time"
        )
        self.auto_lidar_checkbox.setChecked(
            self.settings.value("loop/auto_lidar", False, type=bool)
        )
        self.auto_lidar_checkbox.toggled.connect(self._toggle_auto_modes)
        toggles.addWidget(self.auto_lidar_checkbox)

        self.closed_loop_checkbox = QCheckBox(
            "Closed-Loop Force Hold && Slip Control (FSR + Optical + LiDAR)"
        )
        self.closed_loop_checkbox.setToolTip(
            "Maintains grip force if FSR pressure drops and boosts force if Optical or LiDAR detects slip"
        )
        self.closed_loop_checkbox.setChecked(
            self.settings.value("loop/closed_loop_hold", True, type=bool)
        )
        self.closed_loop_checkbox.toggled.connect(self._toggle_auto_modes)
        toggles.addWidget(self.closed_loop_checkbox)
        toggles.addStretch(1)

        timeout_lbl = QLabel("Max Grip Time")
        timeout_lbl.setObjectName("SmallCaps")
        toggles.addWidget(timeout_lbl)
        self.simple_grip_timeout_spin = QDoubleSpinBox()
        self.simple_grip_timeout_spin.setRange(
            MIN_GRIP_TIMEOUT_MS / 1000.0, MAX_GRIP_TIMEOUT_MS / 1000.0
        )
        self.simple_grip_timeout_spin.setDecimals(1)
        self.simple_grip_timeout_spin.setSingleStep(0.5)
        self.simple_grip_timeout_spin.setSuffix(" s")
        self.simple_grip_timeout_spin.setFixedWidth(84)
        self.simple_grip_timeout_spin.setValue(self.loop_grip_timeout_spin.value())
        self.simple_grip_timeout_spin.setToolTip(
            "Maximum closing time in seconds before non-contacting fingers stop"
        )
        self.simple_grip_timeout_spin.valueChanged.connect(self._simple_grip_timeout_changed)
        toggles.addWidget(self.simple_grip_timeout_spin)

        smart_layout.addLayout(toggles)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        grip_btn = QPushButton("Grip Object [G]")
        grip_btn.setObjectName("Primary")
        grip_btn.setProperty("hero", True)
        grip_btn.setFixedHeight(40)
        grip_btn.clicked.connect(self._trigger_auto_grip)
        btn_row.addWidget(grip_btn, 2)

        release_btn = QPushButton("Release Object [R]")
        release_btn.setObjectName("HoldReverse")
        release_btn.setProperty("hero", True)
        release_btn.setFixedHeight(40)
        release_btn.setToolTip("Opens all 3 fingers for the configured release duration")
        release_btn.clicked.connect(self._trigger_timed_release)
        btn_row.addWidget(release_btn, 2)

        hold_open_btn = QPushButton("Hold to Open")
        hold_open_btn.setObjectName("HoldReverse")
        hold_open_btn.setProperty("hero", True)
        hold_open_btn.setFixedHeight(40)
        hold_open_btn.setToolTip("Hold to manually open all 3 fingers; release to stop")
        hold_open_btn.pressed.connect(lambda: self._grip_hold_action("OPEN_HOLD"))
        hold_open_btn.released.connect(lambda: self._grip_hold_action("STOP"))
        btn_row.addWidget(hold_open_btn, 1)

        stop_btn = QPushButton("Stop Fingers [Space]")
        stop_btn.setObjectName("StopButton")
        stop_btn.setProperty("hero", True)
        stop_btn.setFixedHeight(40)
        stop_btn.clicked.connect(self._stop_all_fingers)
        btn_row.addWidget(stop_btn, 1)
        smart_layout.addLayout(btn_row)
        layout.addWidget(smart_panel)

        # 2. 3-Column Live Sensor Strip
        sensor_row = QHBoxLayout()
        sensor_row.setSpacing(10)

        # Card 1: Palm LiDAR
        lidar_card = QFrame()
        lidar_card.setObjectName("Panel")
        lidar_layout = QVBoxLayout(lidar_card)
        lidar_layout.setContentsMargins(14, 10, 14, 10)
        lidar_layout.setSpacing(5)
        l_head = QHBoxLayout()
        l_title = QLabel("Palm LiDAR (VL53L4CD)")
        l_title.setObjectName("Kicker")
        l_head.addWidget(l_title)
        l_head.addStretch(1)
        self.simple_lidar_badge = QLabel(LIDAR_STATE_LABELS[3])
        self.simple_lidar_badge.setObjectName("LidarState3")
        l_head.addWidget(self.simple_lidar_badge)
        lidar_layout.addLayout(l_head)
        self.simple_lidar_bar = QProgressBar()
        self.simple_lidar_bar.setRange(0, 300)
        self.simple_lidar_bar.setValue(0)
        self.simple_lidar_bar.setTextVisible(False)
        self.simple_lidar_bar.setProperty("lidarState", "3")
        lidar_layout.addWidget(self.simple_lidar_bar)
        self.simple_lidar_label = QLabel("Distance: -- mm, Vel: 0 mm/s")
        self.simple_lidar_label.setObjectName("Description")
        self.simple_lidar_label.setWordWrap(True)
        lidar_layout.addWidget(self.simple_lidar_label)
        self.simple_lidar_plot = DistanceHistoryWidget()
        lidar_layout.addWidget(self.simple_lidar_plot)
        sensor_row.addWidget(lidar_card, 1)

        # Card 2: Slip Fusion + Optical Flow
        opt_card = QFrame()
        opt_card.setObjectName("Panel")
        opt_layout = QVBoxLayout(opt_card)
        opt_layout.setContentsMargins(14, 10, 14, 10)
        opt_layout.setSpacing(4)

        s_head = QHBoxLayout()
        s_title = QLabel("Slip Fusion (Opt + LiDAR)")
        s_title.setObjectName("Kicker")
        s_head.addWidget(s_title)
        s_head.addStretch(1)
        self.simple_slip_badge = QLabel(SLIP_STATE_LABELS[0])
        self.simple_slip_badge.setObjectName("SlipState0")
        s_head.addWidget(self.simple_slip_badge)
        opt_layout.addLayout(s_head)

        self.simple_slip_bar = QProgressBar()
        self.simple_slip_bar.setRange(0, 100)
        self.simple_slip_bar.setValue(0)
        self.simple_slip_bar.setTextVisible(False)
        self.simple_slip_bar.setProperty("slipState", "0")
        opt_layout.addWidget(self.simple_slip_bar)

        self.simple_slip_label = QLabel("Evidence: 0% (Opt: still, LiDAR: still)")
        self.simple_slip_label.setObjectName("Description")
        self.simple_slip_label.setWordWrap(True)
        opt_layout.addWidget(self.simple_slip_label)

        o_head = QHBoxLayout()
        o_title = QLabel("Optical (PAA5100JE)")
        o_title.setObjectName("Kicker")
        o_head.addWidget(o_title)
        o_head.addStretch(1)
        self.simple_opt_badge = QLabel(OPTICAL_STATE_LABELS[3])
        self.simple_opt_badge.setObjectName("OptState3")
        o_head.addWidget(self.simple_opt_badge)
        s_zero_btn = QPushButton("Zero")
        s_zero_btn.setObjectName("Quick")
        s_zero_btn.clicked.connect(self._zero_optical)
        o_head.addWidget(s_zero_btn)
        opt_layout.addLayout(o_head)

        self.simple_opt_bar = QProgressBar()
        self.simple_opt_bar.setRange(0, max(1, self.opt_motion_spin.value()))
        self.simple_opt_bar.setValue(0)
        self.simple_opt_bar.setTextVisible(False)
        self.simple_opt_bar.setProperty("optState", "3")
        opt_layout.addWidget(self.simple_opt_bar)

        self.simple_opt_label = QLabel("Mag: 0 (X: 0, Y: 0), Total: 0, 0")
        self.simple_opt_label.setObjectName("Description")
        self.simple_opt_label.setWordWrap(True)
        opt_layout.addWidget(self.simple_opt_label)
        opt_layout.addStretch(1)
        sensor_row.addWidget(opt_card, 1)

        # Card 3: 3-Finger Force Loop
        fsr_card = QFrame()
        fsr_card.setObjectName("Panel")
        fsr_layout = QVBoxLayout(fsr_card)
        fsr_layout.setContentsMargins(14, 10, 14, 10)
        fsr_layout.setSpacing(6)
        f_head = QHBoxLayout()
        f_title = QLabel("Finger Force (FSR A0–A2)")
        f_title.setObjectName("Kicker")
        f_head.addWidget(f_title)
        f_head.addStretch(1)
        self.simple_boost_label = QLabel("Boost: +0")
        self.simple_boost_label.setObjectName("SmallCaps")
        f_head.addWidget(self.simple_boost_label)
        fsr_layout.addLayout(f_head)

        for ch in continuous_channels:
            row = QHBoxLayout()
            row.setSpacing(8)
            lbl = QLabel(SERVO_CHANNELS[ch])
            lbl.setObjectName("Description")
            lbl.setFixedWidth(62)
            row.addWidget(lbl)
            bar = QProgressBar()
            grip_limit = self.fsr_grip_spins[ch].value() if ch in self.fsr_grip_spins else DEFAULT_FSR_GRIP_THRESHOLD
            bar.setRange(0, max(1, grip_limit))
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setProperty("fsrState", "0")
            row.addWidget(bar, 1)
            val_lbl = QLabel(f"0 / {grip_limit}")
            val_lbl.setObjectName("SmallCaps")
            val_lbl.setFixedWidth(84)
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            row.addWidget(val_lbl)
            self.simple_fsr_bars[ch] = bar
            self.simple_fsr_labels[ch] = val_lbl
            fsr_layout.addLayout(row)
        fsr_layout.addStretch(1)
        sensor_row.addWidget(fsr_card, 1)
        layout.addLayout(sensor_row)

        # 3. Simple Arm & Wrist Control + Saved Poses
        arm_panel = QFrame()
        arm_panel.setObjectName("Panel")
        arm_layout = QVBoxLayout(arm_panel)
        arm_layout.setContentsMargins(16, 12, 16, 12)
        arm_layout.setSpacing(8)

        arm_head = QHBoxLayout()
        arm_head.setSpacing(8)
        arm_title = QLabel("Arm & Wrist Position (Ch 0–4)")
        arm_title.setObjectName("SectionTitle")
        arm_head.addWidget(arm_title)
        arm_head.addStretch(1)

        home_btn = QPushButton("Home 90°")
        home_btn.setObjectName("PoseButton")
        home_btn.clicked.connect(self._center_all)
        arm_head.addWidget(home_btn)

        for pose_idx in (1, 2, 3):
            p_btn = QPushButton(f"Pose {pose_idx} [{pose_idx}]")
            p_btn.setObjectName("PoseButton")
            p_btn.setToolTip(f"Load saved Pose {pose_idx} (Shortcut: {pose_idx})")
            p_btn.clicked.connect(lambda checked=False, idx=pose_idx: self._apply_arm_pose(idx))
            arm_head.addWidget(p_btn)
            s_btn = QPushButton("Save")
            s_btn.setObjectName("Quick")
            s_btn.setToolTip(f"Save current channels 0–4 angles into Pose {pose_idx}")
            s_btn.clicked.connect(lambda checked=False, idx=pose_idx: self._save_arm_pose(idx))
            arm_head.addWidget(s_btn)
        arm_layout.addLayout(arm_head)

        self.pose_note = QLabel("Adjust sliders below or press 1–3 to recall saved poses.")
        self.pose_note.setObjectName("Description")
        arm_layout.addWidget(self.pose_note)

        for ch in positional_channels:
            s_row = QHBoxLayout()
            s_row.setSpacing(8)
            name_lbl = QLabel(SERVO_CHANNELS[ch])
            name_lbl.setObjectName("Kicker")
            name_lbl.setFixedWidth(74)
            s_row.addWidget(name_lbl)

            minus_btn = QPushButton("-5°")
            minus_btn.setObjectName("Nudge")
            minus_btn.setFixedWidth(40)
            minus_btn.clicked.connect(
                lambda checked=False, channel=ch: self._nudge_simple_servo(channel, -5)
            )
            s_row.addWidget(minus_btn)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(MIN_ANGLE, MAX_ANGLE)
            slider.setSingleStep(1)
            slider.setPageStep(10)
            slider.setValue(self.cards[ch].angle() if ch in self.cards else DEFAULT_ANGLE)
            slider.valueChanged.connect(
                lambda val, channel=ch: self._simple_slider_changed(channel, val)
            )
            s_row.addWidget(slider, 1)

            plus_btn = QPushButton("+5°")
            plus_btn.setObjectName("Nudge")
            plus_btn.setFixedWidth(40)
            plus_btn.clicked.connect(
                lambda checked=False, channel=ch: self._nudge_simple_servo(channel, 5)
            )
            s_row.addWidget(plus_btn)

            ang_lbl = QLabel(f"{slider.value():03d}°")
            ang_lbl.setObjectName("Angle")
            ang_lbl.setFixedWidth(46)
            ang_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            s_row.addWidget(ang_lbl)

            self.simple_sliders[ch] = slider
            self.simple_angle_labels[ch] = ang_lbl
            arm_layout.addLayout(s_row)

        layout.addWidget(arm_panel)
        layout.addStretch(1)
        return page

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
        number = QLabel(f"Ch {channel}, Pin {channel}")
        number.setObjectName("ChannelTag")
        header.addWidget(number)
        name = QLabel(SERVO_CHANNELS[channel])
        name.setObjectName("ServoName")
        header.addWidget(name)
        header.addStretch(1)
        type_badge = QLabel("360° continuous" if continuous else "180° positional")
        type_badge.setObjectName("TypeBadgeCont" if continuous else "TypeBadgePos")
        header.addWidget(type_badge)
        layout.addLayout(header)

        motion_panel = QFrame()
        motion_panel.setObjectName("MotionPanel")
        motion_layout = QVBoxLayout(motion_panel)
        motion_layout.setContentsMargins(10, 8, 10, 8)
        motion_layout.setSpacing(7)
        motion_header = QHBoxLayout()
        motion_title = QLabel("Motion Profile")
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

        apply_profile = QPushButton("Apply Motion Profile")
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
        pulse_kicker = QLabel("Pulse Width Mapping (µs)")
        pulse_kicker.setObjectName("Kicker")
        pulse_layout.addWidget(pulse_kicker)

        row = QHBoxLayout()
        row.setSpacing(8)
        groups: list[QSpinBox] = []
        point_names = (
            ("Reverse (Min)", "Stop (Center)", "Forward (Max)")
            if continuous else ("0° (Min)", "90° (Center)", "180° (Max)")
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
            button = QPushButton(f"Test {text.split(' ')[0]}")
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
        reset_btn = QPushButton("Reset Defaults")
        reset_btn.setObjectName("Quick")
        reset_btn.setToolTip("Restore factory default µs values into the fields above")
        reset_btn.clicked.connect(
            lambda checked=False, ch=channel: self._reset_calibration_defaults(ch)
        )
        actions.addWidget(reset_btn)
        cancel = QPushButton("Cancel / 90°")
        cancel.setObjectName("Quick")
        cancel.clicked.connect(lambda checked=False, ch=channel: self._cancel_calibration(ch))
        actions.addWidget(cancel)
        save = QPushButton("Save / Apply")
        save.setObjectName("Primary")
        save.clicked.connect(lambda checked=False, ch=channel: self._apply_calibration(ch))
        actions.addWidget(save)
        layout.addLayout(actions)

        note = QLabel("Start values are a guide. Stop immediately if the servo buzzes or meets resistance.")
        note.setObjectName("Description")
        note.setWordWrap(True)
        self.cal_notes[channel] = note
        layout.addWidget(note)

        if continuous:
            layout.addWidget(self._build_fsr_calibration_panel(channel))

        return panel

    def _build_fsr_calibration_panel(self, channel: int) -> QFrame:
        fsr_panel = QFrame()
        fsr_panel.setObjectName("SubPanel")
        fsr_layout = QVBoxLayout(fsr_panel)
        fsr_layout.setContentsMargins(10, 8, 10, 8)
        fsr_layout.setSpacing(8)

        header = QHBoxLayout()
        header.setSpacing(8)
        pin_name = FSR_PINS.get(channel, "Analog")
        title = QLabel(f"FSR Sensor ({pin_name}) Force Limits")
        title.setObjectName("Kicker")
        header.addWidget(title)
        header.addStretch(1)

        invert_box = QCheckBox("Invert Close (Reverse = Close)")
        invert_box.setToolTip(
            "Check this if Reverse (0°) rather than Forward (180°) tightens this finger's tendon."
        )
        invert_box.setChecked(
            self.settings.value(f"fsr/{channel}/invert_close", False, type=bool)
        )
        header.addWidget(invert_box)
        fsr_layout.addLayout(header)

        fields = QHBoxLayout()
        fields.setSpacing(8)
        specs = (
            ("Contact Thr", "contact", DEFAULT_FSR_CONTACT_THRESHOLD),
            ("Grip Stop Thr", "grip", DEFAULT_FSR_GRIP_THRESHOLD),
            ("Max Safety Thr", "max", DEFAULT_FSR_MAX_THRESHOLD),
        )
        boxes: list[QSpinBox] = []
        for label_text, key, default_val in specs:
            col = QVBoxLayout()
            col.setSpacing(4)
            lbl = QLabel(label_text)
            lbl.setObjectName("SmallCaps")
            col.addWidget(lbl)
            spin = QSpinBox()
            spin.setRange(MIN_FSR_THRESHOLD, MAX_FSR_THRESHOLD)
            spin.setSingleStep(25)
            saved_val = self.settings.value(f"fsr/{channel}/{key}", default_val, type=int)
            spin.setValue(max(MIN_FSR_THRESHOLD, min(MAX_FSR_THRESHOLD, saved_val)))
            col.addWidget(spin)
            fields.addLayout(col)
            boxes.append(spin)
        fsr_layout.addLayout(fields)

        self.fsr_contact_spins[channel] = boxes[0]
        self.fsr_grip_spins[channel] = boxes[1]
        self.fsr_max_spins[channel] = boxes[2]
        self.fsr_invert_boxes[channel] = invert_box

        apply_btn = QPushButton("Apply FSR Settings")
        apply_btn.setObjectName("Quick")
        apply_btn.clicked.connect(
            lambda checked=False, ch=channel: self._apply_fsr_calibration(ch)
        )
        fsr_layout.addWidget(apply_btn)

        fsr_note = QLabel(f"Live {pin_name}: waiting for Arduino connection …")
        fsr_note.setObjectName("Description")
        fsr_note.setWordWrap(True)
        self.fsr_cal_notes[channel] = fsr_note
        fsr_layout.addWidget(fsr_note)

        if channel in self.cards:
            self.cards[channel].update_fsr(0, 0, 0, boxes[1].value())

        return fsr_panel

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
            self.ports.addItem("No serial port found", None)
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
            self.connection_note.setText("Disconnecting…")
            self.connect_button.setEnabled(False)
            self.worker.stop(self._continuous_stop_commands())
            return
        port = self.ports.currentData()
        if not port:
            return
        self.arduino_ready = False
        self._last_error = ""
        self._reported_servo_count = None
        self._set_status("Waiting", "Connecting")
        self.connection_note.setText("Opening port and waiting for Arduino…")
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
        self.connection_note.setText(f"Port open: {port}, waiting for READY…")
        self.connect_button.setText("Disconnect")
        self.connect_button.setObjectName("Disconnect")
        self._repolish(self.connect_button)
        self.connect_button.setEnabled(True)

    def _arduino_is_ready(self) -> None:
        if not self.worker or not self.worker.isRunning():
            return
        self.arduino_ready = True
        self._last_pong = time.monotonic()
        self._set_status("Good", "Connected")
        if (
            self._reported_servo_count is not None
            and self._reported_servo_count != len(SERVO_CHANNELS)
        ):
            self.connection_note.setText(
                f"Warning: Arduino reports {self._reported_servo_count} servos, "
                f"app expects {len(SERVO_CHANNELS)}. Re-upload firmware."
            )
        else:
            self.connection_note.setText("Arduino ready. Servo and sensor telemetry active.")
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
        self.worker.send(set_fsr_guard(self.fsr_guard_checkbox.isChecked()))
        for finger_index, channel in enumerate(CONTINUOUS_SERVO_CHANNELS):
            contact, grip, max_thr, invert = self._fsr_values(channel)
            self.worker.send(
                set_fsr_calibration(finger_index, contact, grip, max_thr, invert)
            )
            self.cards[channel].update_fsr(0, 0, 0, grip)
        self.worker.send(
            set_optical_calibration(
                self.opt_motion_spin.value(), self.opt_squal_spin.value()
            )
        )
        self.worker.send(set_optical_led(self.opt_led_checkbox.isChecked()))
        self.worker.send(
            set_lidar_and_loop_config(
                self.loop_trigger_spin.value(),
                self.loop_dwell_spin.value(),
                self.loop_slip_vel_spin.value(),
                self.loop_release_spin.value(),
                self.loop_boost_spin.value(),
                self.loop_deadband_spin.value(),
                int(round(self.loop_grip_timeout_spin.value() * 1000)),
            )
        )
        self.worker.send(
            set_auto_modes(
                self.auto_lidar_checkbox.isChecked(),
                self.closed_loop_checkbox.isChecked(),
            )
        )
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

    def _fsr_values(self, channel: int) -> tuple[int, int, int, bool]:
        return (
            self.fsr_contact_spins[channel].value(),
            self.fsr_grip_spins[channel].value(),
            self.fsr_max_spins[channel].value(),
            self.fsr_invert_boxes[channel].isChecked(),
        )

    def _apply_fsr_calibration(self, channel: int) -> None:
        contact, grip, max_thr, invert = self._fsr_values(channel)
        if not (MIN_FSR_THRESHOLD <= contact < grip < max_thr <= MAX_FSR_THRESHOLD):
            self.fsr_cal_notes[channel].setText(
                f"ERROR: Must satisfy {MIN_FSR_THRESHOLD} <= CONTACT < GRIP STOP < MAX <= {MAX_FSR_THRESHOLD}."
            )
            return
        self.settings.setValue(f"fsr/{channel}/contact", contact)
        self.settings.setValue(f"fsr/{channel}/grip", grip)
        self.settings.setValue(f"fsr/{channel}/max", max_thr)
        self.settings.setValue(f"fsr/{channel}/invert_close", invert)
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.fsr_cal_notes[channel].setText("COULD NOT SAVE FSR SETTINGS.")
            return
        if channel in self.cards:
            self.cards[channel].update_fsr(0, 0, 0, grip)
        finger_index = CONTINUOUS_SERVO_CHANNELS.index(channel)
        if self.arduino_ready and self.worker:
            self.worker.send(set_fsr_calibration(finger_index, contact, grip, max_thr, invert))
            self.fsr_cal_notes[channel].setText(
                f"ACTIVE: Contact {contact} / Stop {grip} / Max {max_thr}"
                + (" (Inverted close)" if invert else "")
            )
        else:
            self.fsr_cal_notes[channel].setText(
                f"SAVED: Contact {contact} / Stop {grip} / Max {max_thr}. Sent on next connect."
            )

    def _trigger_auto_grip(self) -> None:
        if not self.arduino_ready or not self.worker:
            self.grip_master_status.setText("CONNECT THE ARDUINO BEFORE STARTING AUTO-GRIP.")
            self._set_ctrl_banner(0, 0, "CONNECT ARDUINO FIRST")
            return
        self._grip_sync_active = True
        for ch in CONTINUOUS_SERVO_CHANNELS:
            self.pending.pop(ch, None)
        self.worker.send(grip_command("START"))
        self.grip_master_status.setText(
            "AUTO-GRIP ACTIVE: Closing all 3 fingers until each FSR reaches its stop limit …"
        )
        self._set_ctrl_banner(2, self._slip_force_boost)

    def _trigger_timed_release(self) -> None:
        if not self.arduino_ready or not self.worker:
            self.grip_master_status.setText("CONNECT THE ARDUINO BEFORE RELEASING.")
            self._set_ctrl_banner(0, 0, "CONNECT ARDUINO FIRST")
            return
        self._grip_sync_active = True
        for ch in CONTINUOUS_SERVO_CHANNELS:
            self.pending.pop(ch, None)
        self.worker.send(grip_command("RELEASE"))
        self.grip_master_status.setText(
            f"RELEASING OBJECT: Opening all 3 fingers for {self.loop_release_spin.value()} ms …"
        )
        self._set_ctrl_banner(4, 0)

    def _toggle_auto_modes(self) -> None:
        auto_lidar = self.auto_lidar_checkbox.isChecked()
        closed_loop = self.closed_loop_checkbox.isChecked()
        self.settings.setValue("loop/auto_lidar", auto_lidar)
        self.settings.setValue("loop/closed_loop_hold", closed_loop)
        self.settings.sync()
        if self.arduino_ready and self.worker:
            self.worker.send(set_auto_modes(auto_lidar, closed_loop))

    def _cal_grip_timeout_changed(self, seconds: float) -> None:
        self.simple_grip_timeout_spin.blockSignals(True)
        self.simple_grip_timeout_spin.setValue(seconds)
        self.simple_grip_timeout_spin.blockSignals(False)

    def _simple_grip_timeout_changed(self, seconds: float) -> None:
        self.loop_grip_timeout_spin.blockSignals(True)
        self.loop_grip_timeout_spin.setValue(seconds)
        self.loop_grip_timeout_spin.blockSignals(False)
        self._apply_loop_calibration()

    def _reset_loop_defaults(self) -> None:
        self.loop_trigger_spin.setValue(DEFAULT_LIDAR_TRIGGER_MM)
        self.loop_dwell_spin.setValue(DEFAULT_LIDAR_DWELL_MS)
        self.loop_slip_vel_spin.setValue(DEFAULT_LIDAR_SLIP_VELOCITY_MM_S)
        self.loop_release_spin.setValue(DEFAULT_RELEASE_DURATION_MS)
        self.loop_boost_spin.setValue(DEFAULT_SLIP_BOOST_STEP)
        self.loop_deadband_spin.setValue(DEFAULT_FORCE_HOLD_DEADBAND)
        self.loop_grip_timeout_spin.setValue(DEFAULT_GRIP_TIMEOUT_MS / 1000.0)
        self.lidar_cal_note.setText(
            "LOOP DEFAULTS LOADED. CLICK SAVE / APPLY LOOP TO PERSIST."
        )

    def _apply_loop_calibration(self) -> None:
        trig = self.loop_trigger_spin.value()
        dwell = self.loop_dwell_spin.value()
        slip_vel = self.loop_slip_vel_spin.value()
        rel_ms = self.loop_release_spin.value()
        boost = self.loop_boost_spin.value()
        deadband = self.loop_deadband_spin.value()
        grip_timeout_ms = int(round(self.loop_grip_timeout_spin.value() * 1000))
        for key, val in (
            ("trigger_mm", trig),
            ("dwell_ms", dwell),
            ("slip_vel_mm_s", slip_vel),
            ("release_ms", rel_ms),
            ("boost_step", boost),
            ("hold_deadband", deadband),
            ("grip_timeout_ms", grip_timeout_ms),
        ):
            self.settings.setValue(f"loop/{key}", val)
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.lidar_cal_note.setText("COULD NOT SAVE LOOP SETTINGS.")
            return
        if self.arduino_ready and self.worker:
            self.worker.send(
                set_lidar_and_loop_config(
                    trig, dwell, slip_vel, rel_ms, boost, deadband, grip_timeout_ms
                )
            )
            self.lidar_cal_note.setText(
                f"ACTIVE: Trigger ≤ {trig} mm ({dwell} ms), Slip ≥ {slip_vel} mm/s, "
                f"Release {rel_ms} ms, Boost +{boost}, Deadband {deadband}, "
                f"Timeout {grip_timeout_ms / 1000.0:.1f} s."
            )
        else:
            self.lidar_cal_note.setText(
                f"SAVED: Trigger ≤ {trig} mm ({dwell} ms), Slip ≥ {slip_vel} mm/s, "
                f"Timeout {grip_timeout_ms / 1000.0:.1f} s. Sent on connect."
            )

    def _sync_simple_slider(self, channel: int, angle: int) -> None:
        slider = self.simple_sliders.get(channel)
        label = self.simple_angle_labels.get(channel)
        if slider is None or label is None:
            return
        slider.blockSignals(True)
        slider.setValue(angle)
        slider.blockSignals(False)
        label.setText(f"{angle:03d}°")

    def _simple_slider_changed(self, channel: int, angle: int) -> None:
        if channel in self.simple_angle_labels:
            self.simple_angle_labels[channel].setText(f"{angle:03d}°")
        if channel in self.cards:
            self.cards[channel].set_angle(angle)

    def _save_arm_pose(self, pose_idx: int) -> None:
        positional = [ch for ch in SERVO_CHANNELS if ch not in CONTINUOUS_SERVO_CHANNELS]
        angles = [str(self.cards[ch].angle()) for ch in positional if ch in self.cards]
        self.settings.setValue(f"poses/pose_{pose_idx}", ",".join(angles))
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.pose_note.setText(f"Could not save Pose {pose_idx}.")
            return
        self.pose_note.setText(f"Saved Pose {pose_idx}: {', '.join(a + '°' for a in angles)}")

    def _apply_arm_pose(self, pose_idx: int) -> None:
        raw = self.settings.value(f"poses/pose_{pose_idx}", "", type=str)
        if not raw:
            self.pose_note.setText(
                f"Pose {pose_idx} is empty. Adjust sliders and click SAVE {pose_idx}."
            )
            return
        positional = [ch for ch in SERVO_CHANNELS if ch not in CONTINUOUS_SERVO_CHANNELS]
        try:
            angles = [int(part) for part in raw.split(",")]
        except ValueError:
            self.pose_note.setText(f"Pose {pose_idx} is corrupted; save it again.")
            return
        if len(angles) != len(positional):
            self.pose_note.setText(f"Pose {pose_idx} mismatch; save it again.")
            return
        for ch, angle in zip(positional, angles):
            if ch in self.cards:
                self.cards[ch].set_angle(angle)
        self.pose_note.setText(
            f"Loaded Pose {pose_idx}: {', '.join(f'{a}°' for a in angles)}"
        )

    def _grip_hold_action(self, action: str) -> None:
        if not self.arduino_ready or not self.worker:
            if action != "STOP":
                self.grip_master_status.setText("CONNECT THE ARDUINO BEFORE DRIVING FINGERS.")
            return
        self._grip_sync_active = action != "STOP"
        for ch in CONTINUOUS_SERVO_CHANNELS:
            self.pending.pop(ch, None)
            if action == "STOP":
                self.cards[ch].sync_continuous_state(DEFAULT_ANGLE)
        self.worker.send(grip_command(action))
        if action == "CLOSE_HOLD":
            self.grip_master_status.setText(
                "HOLDING CLOSE ALL 3 FINGERS (FSR auto-stop guard active)."
            )
        elif action == "OPEN_HOLD":
            self.grip_master_status.setText("HOLDING OPEN ALL 3 FINGERS.")
        else:
            self.grip_master_status.setText("ALL 3 FINGERS STOPPED.")

    def _stop_all_fingers(self) -> None:
        self._grip_sync_active = False
        self._set_ctrl_banner(0, 0)
        for ch in CONTINUOUS_SERVO_CHANNELS:
            self.pending.pop(ch, None)
            self.cards[ch].sync_continuous_state(DEFAULT_ANGLE)
        if self.arduino_ready and self.worker:
            self.worker.send(grip_command("STOP"))
        self.grip_master_status.setText("ALL 3 FINGERS STOPPED.")

    def _toggle_fsr_guard(self, enabled: bool) -> None:
        self.settings.setValue("fsr/guard_enabled", enabled)
        self.settings.sync()
        if self.arduino_ready and self.worker:
            self.worker.send(set_fsr_guard(enabled))
        self.grip_master_status.setText(
            "FSR Auto-Stop Guard ENABLED: fingers stop automatically at their grip limit."
            if enabled
            else "WARNING: FSR Auto-Stop Guard DISABLED (manual pressure monitoring only)."
        )

    def _tare_fsrs(self) -> None:
        if not self.arduino_ready or not self.worker:
            self.grip_master_status.setText("CONNECT THE ARDUINO TO ZERO / TARE FSR SENSORS.")
            return
        self.worker.send(tare_fsr())
        self.grip_master_status.setText("ZERO / TARE SENT (KEEP ALL FINGERS UNLOADED) …")

    def _reset_optical_defaults(self) -> None:
        self.opt_motion_spin.setValue(DEFAULT_OPTICAL_MOTION_THRESHOLD)
        self.opt_squal_spin.setValue(DEFAULT_OPTICAL_SQUAL_THRESHOLD)
        self.opt_cal_note.setText(
            f"DEFAULTS LOADED (Mag ≥ {DEFAULT_OPTICAL_MOTION_THRESHOLD}, SQUAL ≥ {DEFAULT_OPTICAL_SQUAL_THRESHOLD}). "
            "CLICK SAVE / APPLY OPTICAL TO PERSIST."
        )

    def _apply_optical_calibration(self) -> None:
        motion_thr = self.opt_motion_spin.value()
        squal_thr = self.opt_squal_spin.value()
        self.settings.setValue("optical/motion_threshold_v2", motion_thr)
        self.settings.setValue("optical/squal_threshold_v2", squal_thr)
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.opt_cal_note.setText("COULD NOT SAVE OPTICAL SETTINGS.")
            return
        self.opt_mag_bar.setRange(0, max(1, motion_thr))
        if self.arduino_ready and self.worker:
            self.worker.send(set_optical_calibration(motion_thr, squal_thr))
            self.opt_cal_note.setText(
                f"ACTIVE: Trigger when Mag ≥ {motion_thr}"
                + (f" & SQUAL ≥ {squal_thr}." if squal_thr > 0 else ".")
            )
        else:
            self.opt_cal_note.setText(
                f"SAVED: Mag ≥ {motion_thr}"
                + (f" & SQUAL ≥ {squal_thr}." if squal_thr > 0 else ".")
                + " Sent on next connect."
            )

    def _toggle_optical_led(self, enabled: bool) -> None:
        self.settings.setValue("optical/led_enabled", enabled)
        self.settings.sync()
        if self.arduino_ready and self.worker:
            self.worker.send(set_optical_led(enabled))

    def _zero_optical(self) -> None:
        if not self.arduino_ready or not self.worker:
            self.opt_cal_note.setText("CONNECT THE ARDUINO TO ZERO TOTAL X / Y.")
            return
        self.worker.send(zero_optical())

    @staticmethod
    def _format_optical_readout(
        dx: int,
        dy: int,
        mag: int,
        total_x: int,
        total_y: int,
        motion_thr: int,
        squal_thr: int,
    ) -> str:
        trigger = f"Mag ≥ {motion_thr}" + (f", SQUAL ≥ {squal_thr}" if squal_thr > 0 else "")
        return (
            f"X: {dx}  |  Y: {dy}  |  Mag: {mag}  |  Total: {total_x}, {total_y}  |  "
            f"Trigger: {trigger}"
        )

    def _update_optical_display(
        self, dx: int, dy: int, mag: int, total_x: int, total_y: int, state: int
    ) -> None:
        motion_thr = max(1, self.opt_motion_spin.value())
        squal_thr = self.opt_squal_spin.value()
        clamped_state = state if state in OPTICAL_STATE_LABELS else 3
        self.opt_mag_bar.setRange(0, motion_thr)
        self.opt_mag_bar.setValue(max(0, min(mag, motion_thr)))
        if clamped_state != self._opt_state:
            self._opt_state = clamped_state
            self.opt_state_badge.setObjectName(f"OptState{clamped_state}")
            self.opt_state_badge.setText(OPTICAL_STATE_LABELS[clamped_state])
            self._repolish(self.opt_state_badge)
            self.opt_mag_bar.setProperty("optState", str(clamped_state))
            self._repolish(self.opt_mag_bar)
        self.opt_value_label.setText(
            self._format_optical_readout(dx, dy, mag, total_x, total_y, motion_thr, squal_thr)
        )
        self.simple_opt_bar.setRange(0, motion_thr)
        self.simple_opt_bar.setValue(max(0, min(mag, motion_thr)))
        if self.simple_opt_badge.objectName() != f"OptState{clamped_state}":
            self.simple_opt_badge.setObjectName(f"OptState{clamped_state}")
            self.simple_opt_badge.setText(OPTICAL_STATE_LABELS[clamped_state])
            self._repolish(self.simple_opt_badge)
            self.simple_opt_bar.setProperty("optState", str(clamped_state))
            self._repolish(self.simple_opt_bar)
        self.simple_opt_label.setText(
            f"Mag: {mag} (X: {dx}, Y: {dy}), Total: {total_x}, {total_y}"
        )
        if clamped_state == 3:
            self.opt_cal_note.setText(
                f"Sensor offline: check PAA5100JE SPI wiring ({OPTICAL_SPI_PINS}) and power."
            )
        else:
            self.opt_cal_note.setText(
                f"Live PAA5100JE ({OPTICAL_STATE_LABELS[clamped_state]}): "
                f"X {dx}, Y {dy}, Mag {mag}, Total {total_x}, {total_y}"
            )

    def _set_ctrl_banner(
        self, ctrl_state: int, slip_boost: int, extra_note: str | None = None
    ) -> None:
        clamped = ctrl_state if ctrl_state in GRIP_CONTROLLER_LABELS else 0
        self._slip_force_boost = max(0, int(slip_boost))
        if clamped != self._ctrl_state:
            self._ctrl_state = clamped
            self.simple_ctrl_banner.setProperty("ctrlState", str(clamped))
            self._repolish(self.simple_ctrl_banner)
        text = f"{GRIP_CONTROLLER_LABELS[clamped]}   |   Force Boost: +{self._slip_force_boost} ADC"
        if extra_note:
            text = f"{text}   |   {extra_note}"
        self.simple_ctrl_banner.setText(text)
        self.simple_boost_label.setText(f"Boost: +{self._slip_force_boost}")

    def _update_slip_display(
        self, confidence: int, state: int, optical_active: bool, lidar_active: bool
    ) -> None:
        clamped_conf = max(0, min(100, int(confidence)))
        clamped_state = state if state in SLIP_STATE_LABELS else 0
        self._slip_confidence = clamped_conf
        self._slip_state = clamped_state

        opt_txt = "moving" if optical_active else "still"
        lidar_txt = "moving" if lidar_active else "still"
        label_text = SLIP_STATE_LABELS[clamped_state]
        summary_text = f"Evidence: {clamped_conf}% (Opt: {opt_txt}, LiDAR: {lidar_txt})"

        for badge, bar, lbl in (
            (self.simple_slip_badge, self.simple_slip_bar, self.simple_slip_label),
            (self.adv_slip_badge, self.adv_slip_bar, self.adv_slip_label),
        ):
            bar.setValue(clamped_conf)
            if badge.objectName() != f"SlipState{clamped_state}":
                badge.setObjectName(f"SlipState{clamped_state}")
                badge.setText(label_text)
                self._repolish(badge)
                bar.setProperty("slipState", str(clamped_state))
                self._repolish(bar)
            lbl.setText(summary_text)

    def _update_lidar_and_loop_display(
        self, dist_mm: int, vel_mm_s: int, lidar_state: int, ctrl_state: int, slip_boost: int
    ) -> None:
        clamped_lidar = lidar_state if lidar_state in LIDAR_STATE_LABELS else 3
        self.simple_lidar_bar.setRange(0, 300)
        if clamped_lidar == 3 or dist_mm <= 0:
            self.simple_lidar_bar.setValue(0)
            dist_text = "--"
        else:
            self.simple_lidar_bar.setValue(max(0, 300 - min(dist_mm, 300)))
            dist_text = str(dist_mm)
            self.simple_lidar_plot.add_sample(dist_mm)
        if clamped_lidar != self._lidar_state:
            self._lidar_state = clamped_lidar
            self.simple_lidar_badge.setObjectName(f"LidarState{clamped_lidar}")
            self.simple_lidar_badge.setText(LIDAR_STATE_LABELS[clamped_lidar])
            self._repolish(self.simple_lidar_badge)
            self.simple_lidar_bar.setProperty("lidarState", str(clamped_lidar))
            self._repolish(self.simple_lidar_bar)
        vel_text = f"{vel_mm_s:+d}" if vel_mm_s != 0 else "0"
        self.simple_lidar_label.setText(
            f"Distance: {dist_text} mm, Vel: {vel_text} mm/s "
            f"(Trigger ≤ {self.loop_trigger_spin.value()} mm)"
        )
        if clamped_lidar == 3:
            self.lidar_cal_note.setText(
                f"{LIDAR_I2C_BUS} offline: check VL53L4CD Qwiic connection and power."
            )
        else:
            self.lidar_cal_note.setText(
                f"Live VL53L4CD: {dist_text} mm, Vel {vel_text} mm/s, "
                f"{LIDAR_STATE_LABELS[clamped_lidar]}, Loop: {GRIP_CONTROLLER_LABELS.get(ctrl_state, 'IDLE')}"
            )
        self._set_ctrl_banner(ctrl_state, slip_boost)

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
                self._set_status("Good", "Connected")
                self.connection_note.setText("Arduino is responding again.")
            return
        kind, values = parse_line(message)
        if kind == "ACK" and len(values) == 2:
            try:
                channel, angle = map(int, values)
            except ValueError:
                return
            if channel in self.cards:
                self.cards[channel].set_ack(angle)
                if channel in CONTINUOUS_SERVO_CHANNELS and (
                    self._grip_sync_active or angle == DEFAULT_ANGLE
                ):
                    self.cards[channel].sync_continuous_state(angle)
        elif kind == "FSR" and len(values) == 9:
            try:
                parsed = [int(v) for v in values]
            except ValueError:
                return
            nets, raws, states = parsed[:3], parsed[3:6], parsed[6:9]
            for finger_index, ch in enumerate(CONTINUOUS_SERVO_CHANNELS):
                base_grip = self.fsr_grip_spins[ch].value() if ch in self.fsr_grip_spins else DEFAULT_FSR_GRIP_THRESHOLD
                max_thr = self.fsr_max_spins[ch].value() if ch in self.fsr_max_spins else DEFAULT_FSR_MAX_THRESHOLD
                active_limit = min(max_thr, base_grip + self._slip_force_boost)
                if ch in self.cards:
                    self.cards[ch].update_fsr(
                        nets[finger_index], raws[finger_index], states[finger_index], active_limit
                    )
                if ch in self.simple_fsr_bars and ch in self.simple_fsr_labels:
                    bar = self.simple_fsr_bars[ch]
                    bar.setRange(0, max(1, active_limit))
                    bar.setValue(max(0, min(nets[finger_index], active_limit)))
                    f_state = states[finger_index] if states[finger_index] in FSR_STATE_LABELS else 0
                    if bar.property("fsrState") != str(f_state):
                        bar.setProperty("fsrState", str(f_state))
                        self._repolish(bar)
                    self.simple_fsr_labels[ch].setText(f"{nets[finger_index]} / {active_limit}")
                if ch in self.fsr_cal_notes:
                    pin = FSR_PINS.get(ch, "FSR")
                    label = FSR_STATE_LABELS.get(states[finger_index], "UNKNOWN")
                    self.fsr_cal_notes[ch].setText(
                        f"Live {pin}: Net {nets[finger_index]} · Raw {raws[finger_index]} · {label}"
                    )
        elif kind == "ACKFSRCAL" and len(values) == 5:
            try:
                finger, contact, grip, max_thr, invert = map(int, values)
            except ValueError:
                return
            if 0 <= finger < len(CONTINUOUS_SERVO_CHANNELS):
                ch = CONTINUOUS_SERVO_CHANNELS[finger]
                if ch in self.fsr_cal_notes:
                    self.fsr_cal_notes[ch].setText(
                        f"ARDUINO CONFIRMED FSR: Contact {contact} / Stop {grip} / Max {max_thr}"
                        + (" (Inverted close)" if invert else "")
                    )
        elif kind == "ACKFSRGUARD" and len(values) == 1:
            enabled = values[0] == "1"
            self.grip_master_status.setText(
                "ARDUINO CONFIRMED: FSR Auto-Stop Guard ENABLED."
                if enabled
                else "ARDUINO CONFIRMED: FSR Auto-Stop Guard DISABLED."
            )
        elif kind == "ACKFSRTARE" and len(values) == 3:
            self.grip_master_status.setText(
                f"FSRs ZEROED (unloaded baselines: {values[0]} / {values[1]} / {values[2]})."
            )
        elif kind == "GRIPSTATE" and len(values) == 1:
            state = values[0]
            if state == "DONE":
                self._grip_sync_active = False
                self._set_ctrl_banner(0, 0)
                self.grip_master_status.setText(
                    "AUTO-GRIP COMPLETE: All 3 fingers stopped at their FSR grip limits."
                )
            elif state == "HOLDING":
                self._grip_sync_active = True
                self._set_ctrl_banner(3, self._slip_force_boost)
                self.grip_master_status.setText(
                    "CLOSED-LOOP HOLDING: Maintaining FSR force and monitoring Optical + LiDAR slip."
                )
            elif state == "DWELL":
                self._set_ctrl_banner(1, 0)
                self.grip_master_status.setText(
                    "PALM LiDAR OBJECT DETECTED: Confirming dwell before closing …"
                )
            elif state == "IDLE":
                self._grip_sync_active = False
                self._set_ctrl_banner(0, 0)
                self.grip_master_status.setText("CONTROLLER IDLE / READY.")
            elif state == "RELEASING":
                self._grip_sync_active = True
                self._set_ctrl_banner(4, 0)
                self.grip_master_status.setText("TIMED RELEASE: Opening all 3 fingers …")
            elif state == "RELEASED":
                self._grip_sync_active = False
                self._set_ctrl_banner(0, 0, "RELEASED")
                for ch in CONTINUOUS_SERVO_CHANNELS:
                    if ch in self.cards:
                        self.cards[ch].sync_continuous_state(DEFAULT_ANGLE)
                self.grip_master_status.setText(
                    "OBJECT RELEASED. Move object clear of palm LiDAR to re-arm auto-grip."
                )
            elif state == "TIMEOUT":
                self._grip_sync_active = False
                self._set_ctrl_banner(0, 0, "TIMEOUT")
                self.grip_master_status.setText(
                    f"AUTO-GRIP TIMEOUT ({self.loop_grip_timeout_spin.value():.1f}s): "
                    "Stopped fingers without full FSR contact."
                )
            elif state == "STOPPED":
                self._grip_sync_active = False
                self._set_ctrl_banner(0, 0)
                self.grip_master_status.setText("ALL 3 FINGERS STOPPED.")
            elif state == "CLOSING":
                self._set_ctrl_banner(2, self._slip_force_boost)
                self.grip_master_status.setText(
                    "AUTO-GRIP CLOSING: Slowing on contact, stopping at FSR grip limits …"
                )
            elif state == "CLOSE_HOLD":
                self.grip_master_status.setText("CLOSING ALL 3 FINGERS (FSR guard active) …")
            elif state == "OPEN_HOLD":
                self.grip_master_status.setText("OPENING ALL 3 FINGERS …")
        elif kind == "LIDAR" and len(values) == 5:
            try:
                dist_mm, vel_mm_s, lidar_state, ctrl_state, slip_boost = map(int, values)
            except ValueError:
                return
            self._update_lidar_and_loop_display(
                dist_mm, vel_mm_s, lidar_state, ctrl_state, slip_boost
            )
        elif kind == "SLIP" and len(values) == 4:
            try:
                confidence, slip_state, opt_flag, lidar_flag = map(int, values)
            except ValueError:
                return
            self._update_slip_display(
                confidence, slip_state, bool(opt_flag), bool(lidar_flag)
            )
        elif kind == "SLIP" and len(values) == 2:
            source = values[0]
            try:
                boost = int(values[1])
            except ValueError:
                return
            self._set_ctrl_banner(3, boost, f"SLIP ({source})!")
            self.grip_master_status.setText(
                f"SLIP DETECTED ({source}): Force target boosted +{boost} ADC."
            )
        elif kind == "ACKLOOPCAL" and len(values) in (6, 7):
            try:
                trig, dwell, vel, rel_ms, boost, deadband = map(int, values[:6])
                timeout_ms = (
                    int(values[6])
                    if len(values) == 7
                    else int(round(self.loop_grip_timeout_spin.value() * 1000))
                )
            except ValueError:
                return
            self.lidar_cal_note.setText(
                f"ARDUINO CONFIRMED LOOP: Trigger ≤ {trig} mm ({dwell} ms), Slip ≥ {vel} mm/s, "
                f"Release {rel_ms} ms, Boost +{boost}, Deadband {deadband}, "
                f"Timeout {timeout_ms / 1000.0:.1f} s."
            )
        elif kind == "ACKAUTOMODE" and len(values) == 2:
            auto_lidar = values[0] == "1"
            closed_loop = values[1] == "1"
            self.lidar_cal_note.setText(
                f"ARDUINO CONFIRMED MODES: Auto-LiDAR {'ON' if auto_lidar else 'OFF'}, "
                f"Closed-Loop Hold {'ON' if closed_loop else 'OFF'}."
            )
        elif kind == "GRIPSTOP" and len(values) == 2:
            try:
                finger, net = map(int, values)
            except ValueError:
                return
            if 0 <= finger < len(CONTINUOUS_SERVO_CHANNELS):
                ch = CONTINUOUS_SERVO_CHANNELS[finger]
                if ch in self.cards:
                    self.cards[ch].sync_continuous_state(DEFAULT_ANGLE)
                self.grip_master_status.setText(
                    f"{SERVO_CHANNELS[ch]} stopped at FSR limit (pressure: {net})."
                )
        elif kind == "OPT" and len(values) == 6:
            try:
                dx, dy, mag, total_x, total_y, state = map(int, values)
            except ValueError:
                return
            self._update_optical_display(dx, dy, mag, total_x, total_y, state)
        elif kind == "ACKOPTZERO":
            self.opt_cal_note.setText("ARDUINO CONFIRMED: Total X / Y reset to 0.")
        elif kind == "ACKOPTCAL" and len(values) == 2:
            try:
                motion_thr, squal_thr = map(int, values)
            except ValueError:
                return
            self.opt_cal_note.setText(
                f"ARDUINO CONFIRMED OPTICAL: Mag ≥ {motion_thr} & SQUAL ≥ {squal_thr}."
            )
        elif kind == "ACKOPTLED" and len(values) == 1:
            led_on = values[0] == "1"
            self.opt_cal_note.setText(
                f"ARDUINO CONFIRMED: Optical sensor LED {'ON' if led_on else 'OFF'}."
            )
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
                    f"Warning: Arduino reports {reported_count} servos, "
                    f"app expects {len(SERVO_CHANNELS)}. Re-upload firmware."
                )
        elif kind == "ERR":
            self.connection_note.setText(f"Arduino: {','.join(values)}")

    def _failed(self, reason: str) -> None:
        self._last_error = reason
        self.connection_note.setText(f"Connection error: {reason}")
        self._set_status("Bad", "Error")

    def _worker_finished(self) -> None:
        self.arduino_ready = False
        self._grip_sync_active = False
        self.pending.clear()
        for card in self.cards.values():
            card.reset_continuous_state()
            card.set_offline()
        for ch, note in self.fsr_cal_notes.items():
            note.setText(f"Live {FSR_PINS.get(ch, 'FSR')}: waiting for Arduino connection …")
        self._update_optical_display(0, 0, 0, 0, 0, 3)
        self._update_lidar_and_loop_display(0, 0, 3, 0, 0)
        self._update_slip_display(0, 0, False, False)
        self.simple_lidar_plot.clear()
        for ch in CONTINUOUS_SERVO_CHANNELS:
            if ch in self.simple_fsr_bars and ch in self.simple_fsr_labels:
                grip_limit = self.fsr_grip_spins[ch].value() if ch in self.fsr_grip_spins else DEFAULT_FSR_GRIP_THRESHOLD
                self.simple_fsr_bars[ch].setRange(0, max(1, grip_limit))
                self.simple_fsr_bars[ch].setValue(0)
                self.simple_fsr_bars[ch].setProperty("fsrState", "0")
                self._repolish(self.simple_fsr_bars[ch])
                self.simple_fsr_labels[ch].setText(f"0 / {grip_limit}")
        self.opt_cal_note.setText(
            f"SPI pins: {OPTICAL_SPI_PINS}. Connect Arduino and move a surface 15–35 mm below the lens to tune."
        )
        self.lidar_cal_note.setText(
            f"{LIDAR_I2C_BUS}: Connect Arduino to monitor live VL53L4CD distance and closed-loop status."
        )
        self.grip_master_status.setText(
            "Connect Arduino to control all 3 fingers together with live FSR pressure limits."
        )
        self._set_status("Bad" if self._last_error else "Off",
                         "Error" if self._last_error else "Offline")
        if not self._last_error:
            self.connection_note.setText("Disconnected. Servo targets are kept in memory.")
        self.connect_button.setText("Connect")
        self.connect_button.setObjectName("ConnectAction")
        self._repolish(self.connect_button)
        self.ports.setEnabled(True)
        self.refresh_button.setEnabled(True)
        if self.worker:
            self.worker.deleteLater()
            self.worker = None
        self.refresh_ports()

    def _center_all(self) -> None:
        self._grip_sync_active = False
        self._set_ctrl_banner(0, 0)
        for card in self.cards.values():
            card.set_angle(DEFAULT_ANGLE)
        if self.arduino_ready and self.worker:
            self.pending.clear()
            self.worker.send("CENTER\n")
            self.grip_master_status.setText("CENTER ALL SENT: All 3 fingers stopped.")

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
