"""Brutalist PySide6 control panel for the dishwasher robot gripper."""

from __future__ import annotations

import time
from collections import deque

import serial.tools.list_ports
from PySide6.QtCore import QSettings, QTimer, Qt, QPointF
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QMainWindow,
    QProgressBar, QPushButton, QScrollArea, QSpinBox, QTabWidget, QVBoxLayout, QWidget,
)

from .config import (
    APP_TITLE, CALIBRATION_STEP_US, COMMAND_INTERVAL_MS, CONTINUOUS_SERVO_CHANNELS,
    DEFAULT_ACCELERATION_DEG_S2, DEFAULT_ANGLE, DEFAULT_CALIBRATION_US,
    DEFAULT_CONTINUOUS_CALIBRATION_US, DEFAULT_FSR_CONTACT_THRESHOLD,
    DEFAULT_FSR_GRIP_THRESHOLD, DEFAULT_FSR_MAX_THRESHOLD, DEFAULT_MAX_SPEED_DEG_S,
    DEFAULT_OPTICAL_MOTION_THRESHOLD, DEFAULT_OPTICAL_SQUAL_THRESHOLD,
    DEFAULT_LIDAR_THRESHOLD_MM, DEFAULT_LIDAR_REARM_MM,
    DEFAULT_SLIP_LIDAR_SPEED_THRESHOLD, MIN_SLIP_LIDAR_SPEED_THRESHOLD,
    MAX_SLIP_LIDAR_SPEED_THRESHOLD,
    FSR_PINS, FSR_STATE_LABELS, MAX_ACCELERATION_DEG_S2, MAX_FSR_THRESHOLD,
    MAX_MAX_SPEED_DEG_S, MAX_ANGLE, MAX_OPTICAL_MOTION_THRESHOLD,
    MAX_OPTICAL_SQUAL_THRESHOLD, MIN_ACCELERATION_DEG_S2, MIN_FSR_THRESHOLD,
    MIN_MAX_SPEED_DEG_S, MIN_ANGLE, MIN_OPTICAL_MOTION_THRESHOLD,
    MIN_OPTICAL_SQUAL_THRESHOLD, OPTICAL_SPI_PINS, OPTICAL_STATE_LABELS,
    MIN_LIDAR_THRESHOLD_MM, MAX_LIDAR_THRESHOLD_MM,
    PULSE_MAX_ALLOWED_US, PULSE_MIN_ALLOWED_US, SERVO_CHANNELS,
)
from .calibration import ServoCalibration
from .protocol import (
    grip_command, parse_line, set_angle, set_calibration, set_fsr_calibration,
    set_fsr_guard, set_motion_profile, set_optical_calibration, set_optical_led,
    set_lidar_config, set_slip_lidar_threshold, set_adaptive_grip,
    tare_fsr, zero_optical,
)
from .serial_worker import SerialWorker
from .servo_card import ServoCard


class DistanceHistoryWidget(QWidget):
    """Compact rolling distance plot; 120 samples at 10 Hz show about 12 seconds."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.samples: deque[int] = deque(maxlen=120)
        self.setMinimumHeight(105)
        self.setMaximumHeight(145)
        self.setToolTip("Recent VL53L4CD distance measurements; approximately 12 seconds of history.")

    def add_sample(self, distance_mm: int) -> None:
        self.samples.append(distance_mm)
        self.update()

    def clear(self) -> None:
        self.samples.clear()
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#101214"))
        area = self.rect().adjusted(38, 8, -8, -20)
        if area.width() <= 1 or area.height() <= 1:
            return

        painter.setPen(QPen(QColor("#303640"), 1))
        for i in range(4):
            y = area.top() + (area.height() * i / 3.0)
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))

        if len(self.samples) < 2:
            painter.setPen(QColor("#8B929E"))
            painter.drawText(area, Qt.AlignmentFlag.AlignCenter, "WAITING FOR DISTANCE SAMPLES")
            return

        low = min(self.samples)
        high = max(self.samples)
        if high - low < 50:
            low -= 25
            high += 25
        else:
            margin = max(10, int((high - low) * 0.08))
            low -= margin
            high += margin
        span = max(1, high - low)

        painter.setPen(QColor("#8B929E"))
        painter.drawText(2, area.top() + 5, f"{high} mm")
        painter.drawText(2, area.bottom(), f"{low} mm")

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
        painter.setPen(QPen(QColor("#E5A93C"), 2))
        painter.drawPath(path)
        painter.setPen(QColor("#8B929E"))
        painter.drawText(
            self.rect().adjusted(38, 0, -8, 0),
            Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignRight,
            "OLDER  ←                         →  NOW",
        )
        painter.end()


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
        self.fsr_contact_spins: dict[int, QSpinBox] = {}
        self.fsr_grip_spins: dict[int, QSpinBox] = {}
        self.fsr_max_spins: dict[int, QSpinBox] = {}
        self.fsr_invert_boxes: dict[int, QCheckBox] = {}
        self.fsr_cal_notes: dict[int, QLabel] = {}
        self._grip_sync_active = False
        self._opt_state = 3
        self._lidar_sensor_state = 0
        self._lidar_auto_confirmed = False
        self._slip_state = 0
        self._slip_confidence = 0
        self._adaptive_targets = {
            index: DEFAULT_FSR_GRIP_THRESHOLD for index in range(len(CONTINUOUS_SERVO_CHANNELS))
        }

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
        control_layout.addWidget(self._build_grip_master_panel())
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

        calibration_layout.addSpacing(6)
        calibration_layout.addWidget(
            self._build_section_banner(
                "OPTICAL SLIP SENSOR  ·  PAA5100JE CALIBRATION",
                f"SPI {OPTICAL_SPI_PINS} · read-only movement detection thresholds",
                continuous=True,
                badge_text="SPI · READ-ONLY",
            )
        )
        calibration_layout.addWidget(self._build_optical_calibration_card())
        calibration_layout.addWidget(self._build_lidar_slip_calibration_card())
        calibration_layout.addStretch(1)
        self.tabs.addTab(calibration_page, "02  CALIBRATION")

        footer = QLabel("SERIAL 115200 BAUD  /  ANGLE IS COMMANDED, NOT MEASURED")
        footer.setObjectName("SmallCaps")
        content.addWidget(footer)

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
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(12)
        title_label = QLabel(title)
        title_label.setObjectName("Kicker")
        layout.addWidget(title_label)
        subtitle_label = QLabel(subtitle)
        subtitle_label.setObjectName("Description")
        layout.addWidget(subtitle_label)
        layout.addStretch(1)
        badge = QLabel(badge_text or ("360° CONTINUOUS" if continuous else "180° POSITIONAL"))
        badge.setObjectName("TypeBadgeCont" if continuous else "TypeBadgePos")
        layout.addWidget(badge)
        return banner

    def _build_grip_master_panel(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("GripMasterPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        top = QHBoxLayout()
        top.setSpacing(10)
        title = QLabel("MASTER 3-FINGER GRIP & FSR CONTROL")
        title.setObjectName("Kicker")
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

        tare_btn = QPushButton("ZERO / TARE FSRs (UNLOADED)")
        tare_btn.setObjectName("Quick")
        tare_btn.setToolTip("Set current unloaded FSR readings as zero baseline on all 3 fingers")
        tare_btn.clicked.connect(self._tare_fsrs)
        top.addWidget(tare_btn)
        layout.addLayout(top)

        actions = QHBoxLayout()
        actions.setSpacing(8)
        auto_grip_btn = QPushButton("▶  AUTO-GRIP (CLOSE TO FSR LIMIT)")
        auto_grip_btn.setObjectName("Primary")
        auto_grip_btn.setToolTip(
            "Closes all 3 fingers simultaneously, slows on contact, and stops each finger at its FSR limit"
        )
        auto_grip_btn.clicked.connect(self._trigger_auto_grip)
        actions.addWidget(auto_grip_btn, 2)

        hold_open_btn = QPushButton("⟲  HOLD OPEN ALL 3")
        hold_open_btn.setObjectName("HoldReverse")
        hold_open_btn.setToolTip("Hold to open all 3 fingers simultaneously; release to stop")
        hold_open_btn.pressed.connect(lambda: self._grip_hold_action("OPEN_HOLD"))
        hold_open_btn.released.connect(lambda: self._grip_hold_action("STOP"))
        actions.addWidget(hold_open_btn, 2)

        hold_close_btn = QPushButton("⟳  HOLD CLOSE ALL 3")
        hold_close_btn.setObjectName("HoldForward")
        hold_close_btn.setToolTip(
            "Hold to close all 3 fingers simultaneously (FSR auto-stop stops each finger at its limit)"
        )
        hold_close_btn.pressed.connect(lambda: self._grip_hold_action("CLOSE_HOLD"))
        hold_close_btn.released.connect(lambda: self._grip_hold_action("STOP"))
        actions.addWidget(hold_close_btn, 2)

        stop_btn = QPushButton("■  STOP FINGERS")
        stop_btn.setObjectName("StopButton")
        stop_btn.setToolTip("Immediately stop all 3 continuous finger servos")
        stop_btn.clicked.connect(self._stop_all_fingers)
        actions.addWidget(stop_btn, 1)
        layout.addLayout(actions)

        self.adaptive_grip_checkbox = QCheckBox("Adaptive grip · tighten gradually on confirmed slip")
        self.adaptive_grip_checkbox.setToolTip(
            "After AUTO-GRIP reaches its FSR targets, confirmed slip raises the affected finger target "
            "in 40-count steps with a 1.2 s minimum interval. Optical-only slip adjusts finger 1; "
            "LiDAR evidence adjusts all fingers. Each finger is capped by its configured FSR maximum."
        )
        adaptive_saved = self.settings.value("grip/adaptive_enabled", False, type=bool)
        if adaptive_saved and not self.fsr_guard_checkbox.isChecked():
            adaptive_saved = False
            self.settings.setValue("grip/adaptive_enabled", False)
        self.adaptive_grip_checkbox.setChecked(adaptive_saved)
        self.adaptive_grip_checkbox.toggled.connect(self._toggle_adaptive_grip)
        layout.addWidget(self.adaptive_grip_checkbox)

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
        opt_title = QLabel("PAA5100JE OPTICAL FLOW · MOVEMENT MONITOR (READ-ONLY)")
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

        zero_opt_btn = QPushButton("ZERO TOTAL X/Y")
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
        layout.addWidget(opt_panel)

        lidar_panel = QFrame()
        lidar_panel.setObjectName("SubPanel")
        lidar_layout = QVBoxLayout(lidar_panel)
        lidar_layout.setContentsMargins(10, 8, 10, 8)
        lidar_layout.setSpacing(8)

        lidar_header = QHBoxLayout()
        lidar_title = QLabel("VL53L4CD LiDAR · DISTANCE & APPROACH SPEED")
        lidar_title.setObjectName("Kicker")
        lidar_header.addWidget(lidar_title)
        lidar_header.addStretch(1)
        self.lidar_state_badge = QLabel("SENSOR OFFLINE")
        self.lidar_state_badge.setObjectName("OptState3")
        lidar_header.addWidget(self.lidar_state_badge)
        lidar_layout.addLayout(lidar_header)

        readouts = QGridLayout()
        readouts.setHorizontalSpacing(18)
        self.lidar_distance_label = QLabel("— mm")
        self.lidar_distance_label.setObjectName("Hero")
        self.lidar_speed_label = QLabel("— mm/s")
        self.lidar_speed_label.setObjectName("Hero")
        distance_col = QVBoxLayout()
        distance_caption = QLabel("DISTANCE")
        distance_caption.setObjectName("SmallCaps")
        distance_col.addWidget(distance_caption)
        distance_col.addWidget(self.lidar_distance_label)
        speed_col = QVBoxLayout()
        speed_caption = QLabel("RADIAL SPEED  ·  − = APPROACHING")
        speed_caption.setObjectName("SmallCaps")
        speed_col.addWidget(speed_caption)
        speed_col.addWidget(self.lidar_speed_label)
        readouts.addLayout(distance_col, 0, 0)
        readouts.addLayout(speed_col, 0, 1)
        lidar_layout.addLayout(readouts)

        plot_caption = QLabel("DISTANCE HISTORY · APPROX. LAST 12 SECONDS")
        plot_caption.setObjectName("SmallCaps")
        lidar_layout.addWidget(plot_caption)
        self.lidar_plot = DistanceHistoryWidget()
        lidar_layout.addWidget(self.lidar_plot)

        controls = QHBoxLayout()
        self.lidar_threshold_spin = QSpinBox()
        self.lidar_threshold_spin.setRange(MIN_LIDAR_THRESHOLD_MM, MAX_LIDAR_THRESHOLD_MM)
        self.lidar_threshold_spin.setSingleStep(10)
        self.lidar_threshold_spin.setSuffix(" mm")
        saved_threshold = self.settings.value(
            "lidar/threshold_mm", DEFAULT_LIDAR_THRESHOLD_MM, type=int
        )
        self.lidar_threshold_spin.setValue(
            max(MIN_LIDAR_THRESHOLD_MM, min(MAX_LIDAR_THRESHOLD_MM, saved_threshold))
        )
        threshold_col = QVBoxLayout()
        threshold_lbl = QLabel("TRIGGER DISTANCE")
        threshold_lbl.setObjectName("SmallCaps")
        threshold_col.addWidget(threshold_lbl)
        threshold_col.addWidget(self.lidar_threshold_spin)
        controls.addLayout(threshold_col)

        self.lidar_auto_checkbox = QCheckBox("Automatic grip on entry")
        self.lidar_auto_checkbox.setToolTip(
            "When armed, entering the threshold starts the existing three-finger FSR-controlled auto-grip."
        )
        self.lidar_auto_checkbox.setChecked(
            self.settings.value("lidar/auto_enabled", False, type=bool)
        )
        self.lidar_auto_checkbox.toggled.connect(self._toggle_lidar_auto)
        controls.addWidget(self.lidar_auto_checkbox, 1, alignment=Qt.AlignmentFlag.AlignVCenter)
        lidar_layout.addLayout(controls)

        self.lidar_status_label = QLabel(
            "Connect Arduino. Automatic grip requires the FSR Auto-Stop Guard."
        )
        self.lidar_status_label.setObjectName("Description")
        self.lidar_status_label.setWordWrap(True)
        lidar_layout.addWidget(self.lidar_status_label)

        slip_panel = QFrame()
        slip_panel.setObjectName("SubPanel")
        slip_layout = QVBoxLayout(slip_panel)
        slip_layout.setContentsMargins(10, 8, 10, 8)
        slip_layout.setSpacing(6)
        slip_header = QHBoxLayout()
        slip_title = QLabel("SLIP DETECTION · SENSOR FUSION (STAGE 2 / ADAPTIVE)")
        slip_title.setObjectName("Kicker")
        slip_header.addWidget(slip_title)
        slip_header.addStretch(1)
        self.slip_state_badge = QLabel("WAITING FOR SENSORS")
        self.slip_state_badge.setObjectName("OptState3")
        slip_header.addWidget(self.slip_state_badge)
        slip_layout.addLayout(slip_header)
        self.slip_confidence_bar = QProgressBar()
        self.slip_confidence_bar.setRange(0, 100)
        self.slip_confidence_bar.setValue(0)
        self.slip_confidence_bar.setFormat("%p% evidence")
        slip_layout.addWidget(self.slip_confidence_bar)
        self.slip_detail_label = QLabel(
            "Optical skin motion: — · LiDAR range motion: —. Detection only; no motor or pressure changes are made."
        )
        self.slip_detail_label.setObjectName("Description")
        self.slip_detail_label.setWordWrap(True)
        slip_layout.addWidget(self.slip_detail_label)
        lidar_layout.addWidget(slip_panel)

        self.lidar_threshold_spin.valueChanged.connect(self._lidar_threshold_changed)
        layout.addWidget(lidar_panel)
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
        name = QLabel("PAA5100JE OPTICAL FLOW SENSOR")
        name.setObjectName("ServoName")
        header.addWidget(name)
        header.addStretch(1)
        badge = QLabel("READ-ONLY MONITOR")
        badge.setObjectName("TypeBadgeCont")
        header.addWidget(badge)
        layout.addLayout(header)

        sub = QFrame()
        sub.setObjectName("SubPanel")
        sub_layout = QVBoxLayout(sub)
        sub_layout.setContentsMargins(10, 8, 10, 8)
        sub_layout.setSpacing(8)
        kicker = QLabel("MOVEMENT ACTIVATION THRESHOLDS (NO MOTOR ACTION)")
        kicker.setObjectName("Kicker")
        sub_layout.addWidget(kicker)

        fields = QHBoxLayout()
        fields.setSpacing(10)

        motion_col = QVBoxLayout()
        motion_col.setSpacing(4)
        motion_lbl = QLabel("MOVEMENT THRESHOLD (MAGNITUDE)")
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
        squal_lbl = QLabel("MIN SURFACE QUALITY (SQUAL)")
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
        reset_btn = QPushButton("RESET OPTICAL DEFAULTS")
        reset_btn.setObjectName("Quick")
        reset_btn.clicked.connect(self._reset_optical_defaults)
        actions.addWidget(reset_btn)
        apply_btn = QPushButton("SAVE / APPLY OPTICAL")
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

    def _build_lidar_slip_calibration_card(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("CalibrationCard")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(8)
        title = QLabel("VL53L4CD · SLIP MOVEMENT SENSITIVITY")
        title.setObjectName("ServoName")
        layout.addWidget(title)
        description = QLabel(
            "Minimum palm-to-object range speed used as LiDAR slip evidence. Lower values are more sensitive; "
            "the optical skin sensor can independently confirm slip even when LiDAR sees no range change."
        )
        description.setObjectName("Description")
        description.setWordWrap(True)
        layout.addWidget(description)
        row = QHBoxLayout()
        self.lidar_slip_sensitivity_spin = QSpinBox()
        self.lidar_slip_sensitivity_spin.setRange(
            MIN_SLIP_LIDAR_SPEED_THRESHOLD, MAX_SLIP_LIDAR_SPEED_THRESHOLD
        )
        self.lidar_slip_sensitivity_spin.setSingleStep(5)
        self.lidar_slip_sensitivity_spin.setSuffix(" mm/s")
        saved = self.settings.value(
            "lidar/slip_speed_threshold_mm_s", DEFAULT_SLIP_LIDAR_SPEED_THRESHOLD, type=int
        )
        self.lidar_slip_sensitivity_spin.setValue(
            max(MIN_SLIP_LIDAR_SPEED_THRESHOLD,
                min(MAX_SLIP_LIDAR_SPEED_THRESHOLD, saved))
        )
        row.addWidget(self.lidar_slip_sensitivity_spin)
        row.addStretch(1)
        apply_btn = QPushButton("SAVE / APPLY LIDAR SENSITIVITY")
        apply_btn.setObjectName("Primary")
        apply_btn.clicked.connect(self._apply_lidar_slip_sensitivity)
        row.addWidget(apply_btn)
        layout.addLayout(row)
        self.lidar_slip_sensitivity_note = QLabel(
            "Lower threshold = more sensitive. Default: 20 mm/s."
        )
        self.lidar_slip_sensitivity_note.setObjectName("Description")
        layout.addWidget(self.lidar_slip_sensitivity_note)
        return panel

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
        pin_name = FSR_PINS.get(channel, "ANALOG")
        title = QLabel(f"FSR SENSOR ({pin_name}) · GRIP LIMIT")
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
            ("CONTACT THR", "contact", DEFAULT_FSR_CONTACT_THRESHOLD),
            ("GRIP STOP THR", "grip", DEFAULT_FSR_GRIP_THRESHOLD),
            ("MAX SAFETY THR", "max", DEFAULT_FSR_MAX_THRESHOLD),
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

        apply_btn = QPushButton("APPLY FSR SETTINGS")
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
        self.worker.send(set_fsr_guard(self.fsr_guard_checkbox.isChecked()))
        self.worker.send(set_adaptive_grip(self.adaptive_grip_checkbox.isChecked()))
        for finger_index, channel in enumerate(CONTINUOUS_SERVO_CHANNELS):
            contact, grip, max_thr, invert = self._fsr_values(channel)
            self.worker.send(
                set_fsr_calibration(finger_index, contact, grip, max_thr, invert)
            )
            self._adaptive_targets[finger_index] = grip
            self.cards[channel].update_fsr(0, 0, 0, grip)
        self.worker.send(
            set_lidar_config(
                self.lidar_auto_checkbox.isChecked(),
                self.lidar_threshold_spin.value(),
                DEFAULT_LIDAR_REARM_MM,
            )
        )
        self.worker.send(
            set_optical_calibration(
                self.opt_motion_spin.value(), self.opt_squal_spin.value()
            )
        )
        self.worker.send(set_slip_lidar_threshold(self.lidar_slip_sensitivity_spin.value()))
        self.worker.send(set_optical_led(self.opt_led_checkbox.isChecked()))
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
        finger_index = CONTINUOUS_SERVO_CHANNELS.index(channel)
        self._adaptive_targets[finger_index] = grip
        if channel in self.cards:
            self.cards[channel].update_fsr(0, 0, 0, grip)
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

    def _toggle_adaptive_grip(self, enabled: bool) -> None:
        if enabled and not self.fsr_guard_checkbox.isChecked():
            self.adaptive_grip_checkbox.blockSignals(True)
            self.adaptive_grip_checkbox.setChecked(False)
            self.adaptive_grip_checkbox.blockSignals(False)
            self.grip_master_status.setText(
                "ENABLE THE FSR AUTO-STOP GUARD BEFORE USING ADAPTIVE GRIP."
            )
            return
        self.settings.setValue("grip/adaptive_enabled", enabled)
        self.settings.sync()
        if self.arduino_ready and self.worker:
            self.worker.send(set_adaptive_grip(enabled))
        self.grip_master_status.setText(
            "ADAPTIVE GRIP ENABLED: confirmed slip can increase finger targets gradually, "
            "within each finger's FSR maximum."
            if enabled else "ADAPTIVE GRIP DISABLED: auto-grip stops at the calibrated FSR targets."
        )

    def _trigger_auto_grip(self) -> None:
        if not self.arduino_ready or not self.worker:
            self.grip_master_status.setText("CONNECT THE ARDUINO BEFORE STARTING AUTO-GRIP.")
            return
        self._grip_sync_active = True
        for finger_index, ch in enumerate(CONTINUOUS_SERVO_CHANNELS):
            self.pending.pop(ch, None)
            self._adaptive_targets[finger_index] = self.fsr_grip_spins[ch].value()
        self.worker.send(grip_command("START"))
        self.grip_master_status.setText(
            "AUTO-GRIP ACTIVE: Closing all 3 fingers until each FSR reaches its stop limit …"
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
        for ch in CONTINUOUS_SERVO_CHANNELS:
            self.pending.pop(ch, None)
            self.cards[ch].sync_continuous_state(DEFAULT_ANGLE)
        if self.arduino_ready and self.worker:
            self.worker.send(grip_command("STOP"))
        self.grip_master_status.setText("ALL 3 FINGERS STOPPED.")

    def _toggle_fsr_guard(self, enabled: bool) -> None:
        if not enabled and self.adaptive_grip_checkbox.isChecked():
            self.adaptive_grip_checkbox.setChecked(False)
        self.settings.setValue("fsr/guard_enabled", enabled)
        self.settings.sync()
        if not enabled and self.lidar_auto_checkbox.isChecked():
            self.lidar_auto_checkbox.setChecked(False)
        if self.arduino_ready and self.worker:
            self.worker.send(set_fsr_guard(enabled))
        self.grip_master_status.setText(
            "FSR Auto-Stop Guard ENABLED: fingers stop automatically at their grip limit."
            if enabled
            else "WARNING: FSR Auto-Stop Guard DISABLED (manual pressure monitoring only)."
        )

    def _send_lidar_config(self, enabled: bool) -> None:
        if self.arduino_ready and self.worker:
            self.worker.send(
                set_lidar_config(enabled, self.lidar_threshold_spin.value(), DEFAULT_LIDAR_REARM_MM)
            )

    def _toggle_lidar_auto(self, enabled: bool) -> None:
        if enabled and not self.fsr_guard_checkbox.isChecked():
            self.lidar_auto_checkbox.blockSignals(True)
            self.lidar_auto_checkbox.setChecked(False)
            self.lidar_auto_checkbox.blockSignals(False)
            self.lidar_status_label.setText(
                "ENABLE THE FSR AUTO-STOP GUARD BEFORE ENABLING AUTOMATIC GRIPPING."
            )
            return
        self.settings.setValue("lidar/auto_enabled", enabled)
        self.settings.sync()
        if not self.arduino_ready or not self.worker:
            self.lidar_status_label.setText(
                "SETTING SAVED. CONNECT THE ARDUINO TO APPLY IT."
                if enabled else "AUTOMATIC GRIPPING DISABLED. DISTANCE MONITORING REMAINS ACTIVE WHEN CONNECTED."
            )
            return
        if enabled and self._lidar_sensor_state != 1:
            self.lidar_auto_checkbox.blockSignals(True)
            self.lidar_auto_checkbox.setChecked(False)
            self.lidar_auto_checkbox.blockSignals(False)
            self.settings.setValue("lidar/auto_enabled", False)
            self.settings.sync()
            self.lidar_status_label.setText("LIDAR SENSOR IS NOT REPORTING VALID DISTANCE DATA.")
            return
        self._send_lidar_config(enabled)
        self.lidar_status_label.setText(
            "AUTO-GRIP ENABLED. WAITING FOR A CLEAR ZONE BEFORE ARMING."
            if enabled else "AUTOMATIC GRIPPING DISABLED. DISTANCE MONITORING CONTINUES."
        )

    def _lidar_threshold_changed(self, value: int) -> None:
        self.settings.setValue("lidar/threshold_mm", value)
        self.settings.sync()
        if self.arduino_ready and self.worker:
            self._send_lidar_config(self.lidar_auto_checkbox.isChecked())

    def _update_slip_display(
        self, confidence: int, state: int, optical_active: bool, lidar_active: bool
    ) -> None:
        self._slip_confidence = max(0, min(100, confidence))
        self._slip_state = state
        self.slip_confidence_bar.setValue(self._slip_confidence)
        labels = {
            0: ("SENSOR DATA UNAVAILABLE", "OptState3"),
            1: ("NO SLIP EVIDENCE", "OptState0"),
            2: ("MOVEMENT SUSPECTED", "OptState2"),
            3: ("CORROBORATED MOVEMENT", "OptState1"),
            4: ("OPTICAL SLIP DETECTED", "OptState1"),
            5: ("LIDAR MOVEMENT DETECTED", "OptState1"),
        }
        label, style = labels.get(state, labels[0])
        self.slip_state_badge.setText(label)
        self.slip_state_badge.setObjectName(style)
        adaptive_note = (
            "Adaptive tightening arms after AUTO-GRIP completes; per-finger FSR maxima remain in force."
            if self.adaptive_grip_checkbox.isChecked()
            else "Adaptive grip is disabled; this display is diagnostic only."
        )
        self.slip_detail_label.setText(
            f"Optical skin motion: {'DETECTED' if optical_active else 'not detected / unavailable'} · "
            f"LiDAR palm-to-object range motion: {'DETECTED' if lidar_active else 'not detected / unavailable'} · "
            + adaptive_note
        )
        self._repolish(self.slip_state_badge)

    def _update_lidar_display(
        self, distance: int, speed: int, state: int, auto_enabled: bool, armed: bool
    ) -> None:
        self._lidar_sensor_state = state
        if state == 1 and distance >= 0:
            self.lidar_plot.add_sample(distance)
            self.lidar_distance_label.setText(f"{distance} mm")
            self.lidar_speed_label.setText(f"{speed:+d} mm/s")
            if speed < -20:
                trend = "APPROACHING"
            elif speed > 20:
                trend = "RECEDING"
            else:
                trend = "STATIONARY"
            self.lidar_state_badge.setText("SENSOR ONLINE")
            self.lidar_state_badge.setObjectName("OptState0")
            self.lidar_status_label.setText(
                f"{trend} · AUTO-GRIP {'ARMED' if armed else ('WAITING FOR CLEAR ZONE' if auto_enabled else 'DISABLED')} · "
                f"TRIGGER AT {self.lidar_threshold_spin.value()} mm"
            )
        elif state == 0:
            self.lidar_distance_label.setText("— mm")
            self.lidar_speed_label.setText("— mm/s")
            self.lidar_state_badge.setText("SENSOR OFFLINE")
            self.lidar_state_badge.setObjectName("OptState3")
            self.lidar_status_label.setText("VL53L4CD NOT FOUND. CHECK QWIIC / I²C CONNECTION.")
        else:
            self.lidar_distance_label.setText("NO TARGET")
            self.lidar_speed_label.setText(f"{speed:+d} mm/s")
            self.lidar_state_badge.setText("NO VALID RANGE")
            self.lidar_state_badge.setObjectName("OptState2")
            self.lidar_status_label.setText(
                "SENSOR ONLINE, BUT NO VALID RANGE IS AVAILABLE. AUTOMATIC TRIGGER IS INHIBITED."
            )
        self._repolish(self.lidar_state_badge)
        if auto_enabled != self.lidar_auto_checkbox.isChecked():
            self.lidar_auto_checkbox.blockSignals(True)
            self.lidar_auto_checkbox.setChecked(auto_enabled)
            self.lidar_auto_checkbox.blockSignals(False)
            self.settings.setValue("lidar/auto_enabled", auto_enabled)
            self.settings.sync()

    def _tare_fsrs(self) -> None:
        if not self.arduino_ready or not self.worker:
            self.grip_master_status.setText("CONNECT THE ARDUINO TO ZERO / TARE FSR SENSORS.")
            return
        self.worker.send(tare_fsr())
        self.grip_master_status.setText("ZERO / TARE SENT (KEEP ALL FINGERS UNLOADED) …")

    def _apply_lidar_slip_sensitivity(self) -> None:
        threshold = self.lidar_slip_sensitivity_spin.value()
        self.settings.setValue("lidar/slip_speed_threshold_mm_s", threshold)
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.lidar_slip_sensitivity_note.setText("COULD NOT SAVE LIDAR SENSITIVITY.")
            return
        if self.arduino_ready and self.worker:
            self.worker.send(set_slip_lidar_threshold(threshold))
            self.lidar_slip_sensitivity_note.setText(
                f"ACTIVE: LiDAR slip evidence at ≥ {threshold} mm/s."
            )
        else:
            self.lidar_slip_sensitivity_note.setText(
                f"SAVED: ≥ {threshold} mm/s. Sent on next connection."
            )

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
        trigger = f"Mag ≥ {motion_thr}" + (f" & SQUAL ≥ {squal_thr}" if squal_thr > 0 else "")
        return (
            f"X: {dx}  |  Y: {dy}  |  Mag: {mag}  |  Total X: {total_x}  |  Total Y: {total_y}  ·  "
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
        if clamped_state == 3:
            self.opt_cal_note.setText(
                f"SENSOR OFFLINE: Check PAA5100JE SPI wiring ({OPTICAL_SPI_PINS}) and power."
            )
        else:
            self.opt_cal_note.setText(
                f"Live PAA5100JE: {OPTICAL_STATE_LABELS[clamped_state]}  ·  "
                f"X: {dx}  |  Y: {dy}  |  Mag: {mag}  |  Total X: {total_x}  |  Total Y: {total_y}"
            )

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
                grip_limit = self._adaptive_targets.get(
                    finger_index,
                    self.fsr_grip_spins[ch].value() if ch in self.fsr_grip_spins else DEFAULT_FSR_GRIP_THRESHOLD,
                )
                if ch in self.cards:
                    self.cards[ch].update_fsr(
                        nets[finger_index], raws[finger_index], states[finger_index], grip_limit
                    )
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
                self._adaptive_targets[finger] = grip
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
                self.grip_master_status.setText(
                    "AUTO-GRIP COMPLETE: All 3 fingers stopped at their FSR grip limits."
                )
            elif state == "TIMEOUT":
                self._grip_sync_active = False
                self.grip_master_status.setText(
                    "AUTO-GRIP TIMEOUT (6s): Stopped fingers that did not reach target pressure."
                )
            elif state == "STOPPED":
                self._grip_sync_active = False
                self.grip_master_status.setText("ALL 3 FINGERS STOPPED.")
            elif state == "ADAPTIVE_READY":
                self._grip_sync_active = False
                self.grip_master_status.setText(
                    "INITIAL GRIP COMPLETE · ADAPTIVE MONITORING ACTIVE. Confirmed slip can trigger small, FSR-limited adjustments."
                )
            elif state == "CLOSING":
                self.grip_master_status.setText(
                    "AUTO-GRIP CLOSING: Slowing on contact, stopping at FSR grip limits …"
                )
            elif state == "CLOSE_HOLD":
                self.grip_master_status.setText("CLOSING ALL 3 FINGERS (FSR guard active) …")
            elif state == "OPEN_HOLD":
                self.grip_master_status.setText("OPENING ALL 3 FINGERS …")
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
        elif kind == "SLIP" and len(values) == 4:
            try:
                confidence, state, optical_active, lidar_active = map(int, values)
            except ValueError:
                return
            self._update_slip_display(
                confidence, state, bool(optical_active), bool(lidar_active)
            )
        elif kind == "LIDAR" and len(values) == 5:
            try:
                distance, speed, state, auto_enabled, armed = map(int, values)
            except ValueError:
                return
            self._update_lidar_display(distance, speed, state, bool(auto_enabled), bool(armed))
        elif kind == "LIDARTRIGGER" and len(values) == 1:
            self._grip_sync_active = True
            for finger_index, ch in enumerate(CONTINUOUS_SERVO_CHANNELS):
                self._adaptive_targets[finger_index] = self.fsr_grip_spins[ch].value()
            self.grip_master_status.setText(
                f"LIDAR TRIGGERED AT {values[0]} mm · STARTING FSR-CONTROLLED AUTO-GRIP."
            )
            self.lidar_status_label.setText(
                f"OBJECT ENTERED {self.lidar_threshold_spin.value()} mm THRESHOLD · GRIP STARTED."
            )
        elif kind == "LIDARSTATE" and len(values) == 1:
            state = values[0]
            if state == "WAITING_FOR_CLEAR":
                self.lidar_status_label.setText("AUTO-GRIP ENABLED · WAITING FOR OBJECT TO MOVE OUTSIDE THE RE-ARM ZONE.")
            elif state == "ARMED":
                self.lidar_status_label.setText("AUTO-GRIP ARMED · WAITING FOR AN OBJECT TO ENTER THE THRESHOLD.")
            elif state == "DISABLED":
                self.lidar_status_label.setText("AUTOMATIC GRIPPING DISABLED.")
            elif state == "DISABLED_FSR_GUARD":
                self.lidar_auto_checkbox.setChecked(False)
                self.lidar_status_label.setText("AUTOMATIC GRIPPING DISABLED BECAUSE THE FSR GUARD WAS TURNED OFF.")
        elif kind == "ACKOPTZERO":
            self.opt_cal_note.setText("ARDUINO CONFIRMED: Total X / Y reset to 0.")
        elif kind == "ACKADAPTIVE" and len(values) == 1:
            self.grip_master_status.setText(
                "ARDUINO CONFIRMED: ADAPTIVE GRIP ENABLED."
                if values[0] == "1" else "ARDUINO CONFIRMED: ADAPTIVE GRIP DISABLED."
            )
        elif kind == "ADAPTIVE" and values:
            if values[0] in ("STEP", "RESET") and len(values) == 3:
                try:
                    finger, target = map(int, values[1:])
                except ValueError:
                    return
                if not 0 <= finger < len(CONTINUOUS_SERVO_CHANNELS):
                    return
                self._adaptive_targets[finger] = target
                channel = CONTINUOUS_SERVO_CHANNELS[finger]
                if channel in self.cards:
                    self.cards[channel].update_fsr(0, 0, 0, target)
                if values[0] == "STEP":
                    self.grip_master_status.setText(
                        f"ADAPTIVE ADJUSTMENT · FINGER {finger + 1} TARGET RAISED TO {target} / FSR COUNTS."
                    )
            elif values[0] == "LIMITS_REACHED_OR_SENSOR_FAULT":
                self.grip_master_status.setText(
                    "ADAPTIVE GRIP DID NOT TIGHTEN: all eligible fingers are at their configured limits or have sensor faults."
                )
        elif kind == "ACKSLIPCFG" and len(values) == 1:
            self.lidar_slip_sensitivity_note.setText(
                f"ARDUINO CONFIRMED LIDAR SLIP THRESHOLD: ≥ {values[0]} mm/s."
            )
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
                    f"WARNING: ARDUINO REPORTS {reported_count} SERVOS, "
                    f"APP EXPECTS {len(SERVO_CHANNELS)}. RE-UPLOAD FIRMWARE."
                )
        elif kind == "ERR":
            error = ",".join(values)
            self.connection_note.setText(f"ARDUINO: {error}")
            if error == "LIDAR_REQUIRES_FSR_GUARD":
                self.lidar_status_label.setText("AUTOMATIC GRIPPING REQUIRES THE FSR AUTO-STOP GUARD.")
                self.lidar_auto_checkbox.setChecked(False)
            elif error == "LIDAR_SENSOR_OFFLINE":
                self.lidar_status_label.setText("VL53L4CD IS OFFLINE; AUTOMATIC GRIPPING CANNOT BE ENABLED.")
                self.lidar_auto_checkbox.setChecked(False)
            elif error == "ADAPTIVE_REQUIRES_FSR_GUARD":
                self.adaptive_grip_checkbox.setChecked(False)
                self.grip_master_status.setText("ADAPTIVE GRIP REQUIRES THE FSR AUTO-STOP GUARD.")

    def _failed(self, reason: str) -> None:
        self._last_error = reason
        self.connection_note.setText(f"CONNECTION ERROR: {reason}")
        self._set_status("Bad", "■ ERROR")

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
        self._update_lidar_display(-1, 0, 0, False, False)
        self.lidar_plot.clear()
        self.opt_cal_note.setText(
            f"SPI pins: {OPTICAL_SPI_PINS}. Connect Arduino and move a surface 15–35 mm below the lens to tune."
        )
        self.grip_master_status.setText(
            "Connect Arduino to control all 3 fingers together with live FSR pressure limits."
        )
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
        self._grip_sync_active = False
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
        # Disarm autonomous triggering before the final finger-stop commands.
        lidar_off = set_lidar_config(False, self.lidar_threshold_spin.value(), DEFAULT_LIDAR_REARM_MM)
        return (lidar_off,) + tuple(
            set_angle(channel, DEFAULT_ANGLE) for channel in CONTINUOUS_SERVO_CHANNELS
        )

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
