"""Modern PySide6 dashboard for the gripper prototype."""

from __future__ import annotations

import serial.tools.list_ports
from PySide6.QtCore import QSettings, QTimer, Qt
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QMainWindow,
    QPushButton, QScrollArea, QSpinBox, QVBoxLayout, QWidget,
)

from .config import (
    APP_TITLE,
    CALIBRATION_STEP_US,
    COMMAND_INTERVAL_MS,
    CONTINUOUS_SERVO_CHANNELS,
    DEFAULT_ANGLE,
    DEFAULT_CALIBRATION_US,
    DEFAULT_CONTINUOUS_CALIBRATION_US,
    FSR_ADC_CHANNEL,
    FSR_ADC_LABEL,
    FSR_ADC_MAX,
    FSR_ADC_REFERENCE_MV,
    JOYSTICK_GROUPS,
    MAX_ANGLE,
    MIN_ANGLE,
    PULSE_MAX_ALLOWED_US,
    PULSE_MIN_ALLOWED_US,
    SERVO_CHANNELS,
)
from .calibration import ServoCalibration
from .protocol import (
    parse_line,
    set_angle,
    set_calibration,
    set_joystick_area,
    set_joystick_enabled,
)
from .serial_worker import SerialWorker
from .servo_card import ServoCard


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1080, 780)
        self.setMinimumSize(850, 630)
        self.worker: SerialWorker | None = None
        self.arduino_ready = False
        self.pending: dict[int, int] = {}
        self.cards: dict[int, ServoCard] = {}
        self._last_error = ""
        self.settings = QSettings("MDU-C2", "Gripper Control")
        self.cal_inputs: dict[int, tuple[QSpinBox, QSpinBox, QSpinBox]] = {}
        self.cal_test_buttons: dict[int, tuple[QPushButton, QPushButton, QPushButton]] = {}
        self.cal_notes: dict[int, QLabel] = {}
        self.joystick_mode = 0
        self.joystick_area = max(
            0, min(1, self.settings.value("joystick/area", 0, type=int))
        )
        self.joystick_seen = False
        self.fsr_seen = False

        self._build_ui()
        self.refresh_ports()

        self.send_timer = QTimer(self)
        self.send_timer.setInterval(COMMAND_INTERVAL_MS)
        self.send_timer.timeout.connect(self._flush_pending)
        self.send_timer.start()

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
        content.setContentsMargins(34, 30, 34, 30)
        content.setSpacing(22)

        header = QHBoxLayout()
        titles = QVBoxLayout()
        titles.setSpacing(7)
        kicker = QLabel("MDU   /   DISHWASHER ROBOT   /   CONTROL STATION")
        kicker.setObjectName("Kicker")
        titles.addWidget(kicker)
        title = QLabel("Gripper control")
        title.setObjectName("Hero")
        titles.addWidget(title)
        subtitle = QLabel("Livekontroll av servomotorer  ·  Arduino UNO R4 WiFi + PCA9685")
        subtitle.setObjectName("BodyMuted")
        titles.addWidget(subtitle)
        header.addLayout(titles)
        header.addStretch(1)
        self.status = QLabel("●  OFFLINE")
        self.status.setObjectName("Off")
        header.addWidget(self.status, alignment=Qt.AlignmentFlag.AlignTop)
        content.addLayout(header)

        panel = QFrame()
        panel.setObjectName("Panel")
        connection = QVBoxLayout(panel)
        connection.setContentsMargins(22, 19, 22, 19)
        connection.setSpacing(12)
        section = QLabel("ANSLUTNING")
        section.setObjectName("Kicker")
        connection.addWidget(section)
        connection_row = QHBoxLayout()
        connection_row.setSpacing(10)
        self.ports = QComboBox()
        self.ports.setMinimumWidth(300)
        connection_row.addWidget(self.ports, 1)
        refresh = QPushButton("Uppdatera portar")
        refresh.clicked.connect(self.refresh_ports)
        connection_row.addWidget(refresh)
        self.refresh_button = refresh
        self.connect_button = QPushButton("Anslut")
        self.connect_button.setObjectName("Primary")
        self.connect_button.clicked.connect(self._toggle_connection)
        self.connect_button.setMinimumWidth(125)
        connection_row.addWidget(self.connect_button)
        connection.addLayout(connection_row)
        self.connection_note = QLabel("Välj Arduino-porten och tryck Anslut.")
        self.connection_note.setObjectName("Description")
        connection.addWidget(self.connection_note)
        content.addWidget(panel)

        section_row = QHBoxLayout()
        section_title = QLabel("SERVO CONTROL")
        section_title.setObjectName("Kicker")
        section_row.addWidget(section_title)
        section_row.addStretch()
        center = QPushButton("↺  Alla till 90°")
        center.setToolTip("Sätter alla servonas målvinklar till 90°")
        center.clicked.connect(self._center_all)
        section_row.addWidget(center)
        content.addLayout(section_row)

        servo_grid = QGridLayout()
        servo_grid.setHorizontalSpacing(16)
        servo_grid.setVerticalSpacing(16)
        for index, (channel, name) in enumerate(SERVO_CHANNELS.items()):
            card = ServoCard(
                channel,
                name,
                continuous=channel in CONTINUOUS_SERVO_CHANNELS,
            )
            card.changed.connect(self._servo_changed)
            self.cards[channel] = card
            servo_grid.addWidget(card, index // 2, index % 2)
        content.addLayout(servo_grid)

        joystick_panel = QFrame()
        joystick_panel.setObjectName("Panel")
        joystick_layout = QVBoxLayout(joystick_panel)
        joystick_layout.setContentsMargins(22, 18, 22, 20)
        joystick_layout.setSpacing(10)
        joystick_title_row = QHBoxLayout()
        joystick_title = QLabel("JOYSTICK CONTROL")
        joystick_title.setObjectName("Kicker")
        joystick_title_row.addWidget(joystick_title)
        joystick_title_row.addStretch(1)
        self.joystick_status = QLabel("● EJ ANSLUTEN")
        self.joystick_status.setObjectName("Off")
        joystick_title_row.addWidget(self.joystick_status)
        joystick_layout.addLayout(joystick_title_row)

        joystick_info = QGridLayout()
        joystick_info.setHorizontalSpacing(18)
        joystick_info.setVerticalSpacing(6)
        mode_label = QLabel("Läge")
        mode_label.setObjectName("Description")
        self.joystick_mode_label = QLabel("Väntar på Arduino")
        self.joystick_mode_label.setObjectName("SectionTitle")
        x_label = QLabel("X-axel")
        x_label.setObjectName("Description")
        self.joystick_x_label = QLabel("–")
        y_label = QLabel("Y-axel")
        y_label.setObjectName("Description")
        self.joystick_y_label = QLabel("–")
        joystick_info.addWidget(mode_label, 0, 0)
        joystick_info.addWidget(self.joystick_mode_label, 0, 1)
        joystick_info.addWidget(x_label, 1, 0)
        joystick_info.addWidget(self.joystick_x_label, 1, 1)
        joystick_info.addWidget(y_label, 2, 0)
        joystick_info.addWidget(self.joystick_y_label, 2, 1)
        joystick_layout.addLayout(joystick_info)

        joystick_controls = QHBoxLayout()
        self.joystick_area_checkbox = QCheckBox("Fingerläge (kanal 6–8)")
        self.joystick_area_checkbox.setChecked(self.joystick_area == 1)
        self.joystick_area_checkbox.setToolTip(
            "Växla mellan Övriga-läget (kanal 0–5) och fingerläget (kanal 6–8)."
        )
        self.joystick_area_checkbox.toggled.connect(self._joystick_area_changed)
        joystick_controls.addWidget(self.joystick_area_checkbox)
        self.joystick_enabled_checkbox = QCheckBox("Joystick aktiv")
        self.joystick_enabled_checkbox.setChecked(
            self.settings.value("joystick/enabled", True, type=bool)
        )
        self.joystick_enabled_checkbox.toggled.connect(self._joystick_enabled_changed)
        joystick_controls.addWidget(self.joystick_enabled_checkbox)
        joystick_controls.addStretch(1)
        joystick_hint = QLabel(
            "Tryck på joystick-knappen för nästa läge · släpp till neutral efter lägesbyte"
        )
        joystick_hint.setObjectName("Description")
        joystick_controls.addWidget(joystick_hint)
        joystick_layout.addLayout(joystick_controls)
        content.addWidget(joystick_panel)

        calibration_panel = QFrame()
        calibration_panel.setObjectName("Panel")
        calibration_layout = QVBoxLayout(calibration_panel)
        calibration_layout.setContentsMargins(22, 18, 22, 20)
        calibration_layout.setSpacing(11)
        cal_title = QLabel("SERVO CALIBRATION  /  PWM")
        cal_title.setObjectName("Kicker")
        calibration_layout.addWidget(cal_title)
        cal_description = QLabel(
            "Positionsservon använder tre punkter: 0° min, 90° center och 180° max. "
            "För 360°-servon betyder punkterna bakåt, stopp och framåt. "
            "Ändra i 10 µs-steg, spara vid stopp och testa sedan en punkt i taget utan last."
        )
        cal_description.setObjectName("Description")
        cal_description.setWordWrap(True)
        calibration_layout.addWidget(cal_description)

        for channel in SERVO_CHANNELS:
            calibration_column = QVBoxLayout()
            calibration_column.setSpacing(8)
            cal_row = QHBoxLayout()
            cal_row.setSpacing(10)
            title_label = QLabel(f"Kanal {channel} · {SERVO_CHANNELS[channel]}")
            title_label.setMinimumWidth(75)
            cal_row.addWidget(title_label)

            continuous = channel in CONTINUOUS_SERVO_CHANNELS
            default_calibration = (
                DEFAULT_CONTINUOUS_CALIBRATION_US
                if continuous
                else DEFAULT_CALIBRATION_US
            )
            old_min = self.settings.value(
                f"servo/{channel}/min_us", default_calibration[0], type=int
            )
            old_max = self.settings.value(
                f"servo/{channel}/max_us", default_calibration[2], type=int
            )
            midpoint = (old_min + old_max) // 2
            saved_center = self.settings.value(
                f"servo/{channel}/center_us", midpoint, type=int
            )
            values = (old_min, saved_center, old_max)
            # One-time migration for the old generic 1000/1500/2000 defaults.
            # Keep deliberate user calibration values untouched.
            migration_key = f"servo/{channel}/continuous_defaults_v2"
            if (
                continuous
                and not self.settings.value(migration_key, False, type=bool)
                and values == DEFAULT_CALIBRATION_US
            ):
                values = DEFAULT_CONTINUOUS_CALIBRATION_US
                self.settings.setValue(f"servo/{channel}/min_us", values[0])
                self.settings.setValue(f"servo/{channel}/center_us", values[1])
                self.settings.setValue(f"servo/{channel}/max_us", values[2])
            if continuous:
                self.settings.setValue(migration_key, True)
                self.settings.sync()
            boxes: list[QSpinBox] = []
            point_labels = (
                ("Bakåt · min", "Stopp · center", "Framåt · max")
                if channel in CONTINUOUS_SERVO_CHANNELS
                else ("0° · min", "90° · center", "180° · max")
            )
            for angle, value, label_text in (
                (MIN_ANGLE, values[0], point_labels[0]),
                (DEFAULT_ANGLE, values[1], point_labels[1]),
                (MAX_ANGLE, values[2], point_labels[2]),
            ):
                group = QVBoxLayout()
                group.setSpacing(2)
                label = QLabel(label_text)
                label.setObjectName("Description")
                group.addWidget(label)
                box = QSpinBox()
                box.setRange(PULSE_MIN_ALLOWED_US, PULSE_MAX_ALLOWED_US)
                box.setSingleStep(CALIBRATION_STEP_US)
                box.setSuffix(" µs")
                box.setValue(max(PULSE_MIN_ALLOWED_US, min(PULSE_MAX_ALLOWED_US, value)))
                box.setToolTip(
                    f"Pulsbredden som används när målvinkeln är {angle}°. "
                    f"Tillåtet programintervall: {PULSE_MIN_ALLOWED_US}–{PULSE_MAX_ALLOWED_US} µs."
                )
                group.addWidget(box)
                cal_row.addLayout(group)
                boxes.append(box)

            calibration_column.addLayout(cal_row)
            action_row = QHBoxLayout()
            action_row.setSpacing(8)
            action_row.addStretch(1)

            self.cal_inputs[channel] = (boxes[0], boxes[1], boxes[2])
            test_buttons: list[QPushButton] = []
            for angle in (MIN_ANGLE, DEFAULT_ANGLE, MAX_ANGLE):
                test_button = QPushButton(
                    f"Testa {self._servo_command_label(channel, angle)}"
                )
                test_button.setObjectName("Quick")
                test_button.setToolTip(
                    "Skickar endast denna målvinkel. Ingen automatisk svepning utförs."
                )
                test_button.clicked.connect(
                    lambda checked=False, ch=channel, a=angle: self._test_calibration_point(ch, a)
                )
                action_row.addWidget(test_button)
                test_buttons.append(test_button)
            self.cal_test_buttons[channel] = (
                test_buttons[0], test_buttons[1], test_buttons[2]
            )

            cancel_button = QPushButton("Avbryt / 90°")
            cancel_button.setObjectName("Quick")
            cancel_button.setToolTip("Avbryt testläget och kommendera 90° på denna kanal")
            cancel_button.clicked.connect(
                lambda checked=False, ch=channel: self._cancel_calibration(ch)
            )
            action_row.addWidget(cancel_button)

            apply_button = QPushButton("Spara / tillämpa")
            apply_button.setToolTip(
                "Sparar 0/90/180-punkterna på Mac och skickar dem till Arduino"
            )
            apply_button.clicked.connect(lambda checked=False, ch=channel: self._apply_calibration(ch))
            action_row.addWidget(apply_button)
            calibration_column.addLayout(action_row)
            note = QLabel(
                "Start: 1000 / 1500 / 2000 µs. För 360° är center stopp. Testa ändpunkter försiktigt; "
                "backa direkt vid surr eller mekaniskt stopp."
            )
            note.setObjectName("Description")
            note.setWordWrap(True)
            self.cal_notes[channel] = note
            calibration_column.addWidget(note)
            calibration_layout.addLayout(calibration_column)

        content.addWidget(calibration_panel)

        future_panel = QFrame()
        future_panel.setObjectName("Panel")
        future_layout = QVBoxLayout(future_panel)
        future_layout.setContentsMargins(20, 18, 20, 20)
        future_layout.setSpacing(12)
        future_title = QLabel("READY FOR EXPANSION")
        future_title.setObjectName("Kicker")
        future_layout.addWidget(future_title)
        future_desc = QLabel("Sensorer visas live när Arduino skickar telemetri.")
        future_desc.setObjectName("Description")
        future_layout.addWidget(future_desc)
        future_row = QHBoxLayout()
        future_row.setSpacing(12)
        for title, detail in (
            ("LiDAR", "I²C  /  avstånd"),
            ("FSR", "A0–A2  /  tryck"),
            ("Slip detection", "framtida sensor"),
        ):
            tile = QFrame()
            tile.setObjectName("FutureTile")
            tile_layout = QVBoxLayout(tile)
            tile_layout.setContentsMargins(15, 12, 15, 12)
            name = QLabel(title)
            name.setObjectName("SectionTitle")
            tile_layout.addWidget(name)
            if title == "FSR":
                secondary = QLabel(
                    f"{FSR_ADC_LABEL}  ·  VOLT 0,00–{FSR_ADC_REFERENCE_MV / 1000:.2f} V"
                )
                secondary.setObjectName("Description")
                tile_layout.addWidget(secondary)
                self.fsr_value_label = QLabel("Ej ansluten")
                self.fsr_value_label.setObjectName("SectionTitle")
                self.fsr_value_label.setToolTip(
                    "Spänning beräknad från FSR på A1 med 5,00 V ADC-referens. "
                    "Värdet är inte omräknat till kraft."
                )
                tile_layout.addWidget(self.fsr_value_label)
                self.fsr_raw_label = QLabel("RAW –")
                self.fsr_raw_label.setObjectName("Description")
                tile_layout.addWidget(self.fsr_raw_label)
            else:
                secondary = QLabel(detail + "    ·    EJ AKTIV")
                secondary.setObjectName("Description")
                tile_layout.addWidget(secondary)
            future_row.addWidget(tile, 1)
        future_layout.addLayout(future_row)
        content.addWidget(future_panel)
        content.addStretch(1)

        footer = QLabel(
            "SERIAL 115200 BAUD    •    MÅLVINKEL, INTE UPPMÄTT SERVOPOSITION"
        )
        footer.setObjectName("SmallCaps")
        content.addWidget(footer)

    def refresh_ports(self) -> None:
        previous = self.ports.currentData()
        self.ports.clear()
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            self.ports.addItem(f"{port.device}  ·  {port.description}", port.device)
        if not ports:
            self.ports.addItem("Ingen seriell port hittades", None)
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
            self.connection_note.setText("Kopplar från …")
            self.connect_button.setEnabled(False)
            self.worker.stop(self._continuous_stop_commands())
            return
        selected_port = self.ports.currentData()
        if not selected_port:
            return
        self.arduino_ready = False
        self._last_error = ""
        self.status.setText("●  CONNECTING")
        self.status.setObjectName("Waiting")
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)
        self.connection_note.setText("Öppnar port och väntar på Arduino …")
        self.connect_button.setEnabled(False)
        self.ports.setEnabled(False)
        self.refresh_button.setEnabled(False)
        self.worker = SerialWorker(selected_port, self)
        self.worker.port_opened.connect(self._port_opened)
        self.worker.ready.connect(self._arduino_is_ready)
        self.worker.received.connect(self._received)
        self.worker.failed.connect(self._failed)
        self.worker.finished.connect(self._worker_finished)
        self.worker.start()

    def _port_opened(self, port: str) -> None:
        self.connection_note.setText(f"Port öppen: {port}. Väntar på Arduino READY …")
        self.connect_button.setText("Koppla från")
        self.connect_button.setObjectName("Disconnect")
        self.connect_button.style().unpolish(self.connect_button)
        self.connect_button.style().polish(self.connect_button)
        self.connect_button.setEnabled(True)

    def _arduino_is_ready(self) -> None:
        if not self.worker or not self.worker.isRunning():
            return
        self.arduino_ready = True
        self.status.setText("●  CONNECTED")
        self.status.setObjectName("Good")
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)
        self.connection_note.setText("Arduino ansluten. Skicka nya vinklar med reglagen nedan.")
        # Apply saved calibration before sending any angles. The serial
        # worker guarantees FIFO ordering for messages queued together.
        for ch in self.cards:
            low, center, high = self._calibration_values(ch)
            self.worker.send(set_calibration(ch, low, center, high))
            # Always start a new connection at the safe default. Do not replay
            # a stale GUI target and cause a surprise move after USB reconnect.
            self.cards[ch].set_angle(DEFAULT_ANGLE)
            self.pending[ch] = DEFAULT_ANGLE
        # Queue the startup SET commands explicitly before JOYAREA/JOY. The
        # normal pending timer is intentionally bypassed here so joystick
        # enable cannot overtake the safe 90° synchronization on a reconnect.
        for ch in self.cards:
            self.worker.send(set_angle(ch, DEFAULT_ANGLE))
        self.pending.clear()
        # Send this after the manual startup synchronization because an
        # explicit SET is the arbitration event that pauses joystick control.
        self.worker.send(set_joystick_area(self.joystick_area))
        self.worker.send(set_joystick_enabled(self.joystick_enabled_checkbox.isChecked()))
        self.connection_note.setText(
            "Arduino ansluten. Servona hålls på 90° tills du väljer en ny målvinkel."
        )

    def _joystick_enabled_changed(self, enabled: bool) -> None:
        self.settings.setValue("joystick/enabled", enabled)
        self.settings.sync()
        if self.arduino_ready and self.worker:
            self.worker.send(set_joystick_enabled(enabled))
        if not enabled:
            self.joystick_status.setText("● AVSTÄNGD")
            self.joystick_status.setObjectName("Off")
            self._repolish(self.joystick_status)

    def _joystick_area_changed(self, fingers: bool) -> None:
        self.joystick_area = 1 if fingers else 0
        self.joystick_mode = 0
        self.settings.setValue("joystick/area", self.joystick_area)
        self.settings.sync()
        if self.arduino_ready and self.worker:
            # The firmware stops the previous group and requires neutral before
            # accepting movement in the newly selected group.
            self.worker.send(set_joystick_area(self.joystick_area))
        group = JOYSTICK_GROUPS[self.joystick_area]
        self.joystick_mode_label.setText(
            f"{group['name']} · välj läge med joystick-knappen"
        )

    @staticmethod
    def _joystick_direction(speed: int) -> str:
        if speed == 0:
            return "stopp"
        return f"{('+' if speed > 0 else '')}{speed}"

    def _update_joystick_display(
        self,
        enabled: bool,
        area: int,
        mode: int,
        raw_x: int,
        raw_y: int,
        speed_x: int,
        speed_y: int,
        armed: bool,
    ) -> None:
        self.joystick_seen = True
        if area in range(len(JOYSTICK_GROUPS)):
            self.joystick_area = area
        group = JOYSTICK_GROUPS[self.joystick_area]
        modes = group["modes"]
        if 0 <= mode < len(modes):
            self.joystick_mode = mode
        mode_info = modes[self.joystick_mode]
        x_channels = mode_info["x_channels"]
        y_channels = mode_info["y_channels"]
        channel_names = lambda channels: ", ".join(
            f"{SERVO_CHANNELS[channel]} · CH{channel}" for channel in channels
        )
        self.joystick_mode_label.setText(
            f"{group['name']} · {self.joystick_mode + 1} / {len(modes)} · "
            f"{mode_info['name']}"
        )
        self.joystick_x_label.setText(
            f"{channel_names(x_channels)} · ADC {raw_x} · "
            f"{self._joystick_direction(speed_x)}"
        )
        self.joystick_y_label.setText(
            f"{channel_names(y_channels) if y_channels else 'Ej använd i detta fingerläge'} "
            f"· ADC {raw_y} · {self._joystick_direction(speed_y)}"
        )
        self.joystick_area_checkbox.blockSignals(True)
        self.joystick_area_checkbox.setChecked(self.joystick_area == 1)
        self.joystick_area_checkbox.blockSignals(False)
        self.joystick_enabled_checkbox.blockSignals(True)
        self.joystick_enabled_checkbox.setChecked(enabled)
        self.joystick_enabled_checkbox.blockSignals(False)
        if not enabled:
            status_text, status_object = "● AVSTÄNGD", "Off"
        elif not armed:
            status_text, status_object = "● NEUTRAL KRÄVS", "Waiting"
        else:
            status_text, status_object = "● AKTIV", "Good"
        self.joystick_status.setText(status_text)
        self.joystick_status.setObjectName(status_object)
        self._repolish(self.joystick_status)
        for channel, card in self.cards.items():
            axis = ""
            if channel in x_channels:
                axis = "X"
            if channel in y_channels:
                axis = "Y"
            card.set_joystick_axis(axis)

    @staticmethod
    def _repolish(widget: QWidget) -> None:
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    @staticmethod
    def _servo_command_label(channel: int, angle: int) -> str:
        if channel in CONTINUOUS_SERVO_CHANNELS:
            return {MIN_ANGLE: "bakåt", DEFAULT_ANGLE: "stopp", MAX_ANGLE: "framåt"}[angle]
        return f"{angle}°"

    def _apply_calibration(self, channel: int) -> None:
        low, center, high = self._calibration_values(channel)
        calibration = ServoCalibration(low, center, high)
        validation_error = calibration.validate()
        if validation_error:
            self.cal_notes[channel].setText(f"Fel: {validation_error}")
            return
        if self.arduino_ready and self.cards[channel].angle() != DEFAULT_ANGLE:
            self.cal_notes[channel].setText(
                f"Sätt först denna servo till {self._servo_command_label(channel, DEFAULT_ANGLE)} "
                f"eller tryck Avbryt / {self._servo_command_label(channel, DEFAULT_ANGLE)}. "
                "Kalibrering tillämpas inte under rörelse."
            )
            return
        self.settings.setValue(f"servo/{channel}/min_us", low)
        self.settings.setValue(f"servo/{channel}/center_us", center)
        self.settings.setValue(f"servo/{channel}/max_us", high)
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.cal_notes[channel].setText("Kunde inte spara kalibreringen på Mac.")
            return
        if self.arduino_ready and self.worker:
            # CAL itself does not move the servo. Re-sending 90° applies the
            # new center while the calibration procedure is in its safe pose.
            self.worker.send(set_calibration(channel, low, center, high))
            self.worker.send(set_angle(channel, DEFAULT_ANGLE))
            self.cal_notes[channel].setText(
                f"✓ Aktiv: {low} / {center} / {high} µs. Testa sedan en punkt i taget."
            )
        else:
            self.cal_notes[channel].setText(
                f"Sparat: {low} / {center} / {high} µs. Används vid nästa anslutning."
            )

    def _calibration_values(self, channel: int) -> tuple[int, int, int]:
        boxes = self.cal_inputs[channel]
        return boxes[0].value(), boxes[1].value(), boxes[2].value()

    def _test_calibration_point(self, channel: int, angle: int) -> None:
        if not self.arduino_ready:
            self.cal_notes[channel].setText(
                "Anslut Arduino innan du testar en kalibreringspunkt."
            )
            return
        # A button sends one explicit target only; there is no automatic sweep.
        self.cards[channel].set_angle(angle)
        self.pending[channel] = angle
        low, center, high = self._calibration_values(channel)
        self.cal_notes[channel].setText(
            f"Testar kommenderat {self._servo_command_label(channel, angle)}. "
            f"Aktiv mapping: {low} / {center} / {high} µs."
        )

    def _cancel_calibration(self, channel: int) -> None:
        self.cards[channel].set_angle(DEFAULT_ANGLE)
        if self.arduino_ready:
            self.pending[channel] = DEFAULT_ANGLE
        self.cal_notes[channel].setText(
            f"Kalibreringstest avbruten; kanal kommenderad till "
            f"{self._servo_command_label(channel, DEFAULT_ANGLE)}. "
            "Osparade pulsändringar är inte aktiva."
        )

    def _servo_changed(self, channel: int, angle: int) -> None:
        self.pending[channel] = angle

    def _flush_pending(self) -> None:
        if not self.arduino_ready or not self.worker:
            return
        to_send = self.pending
        self.pending = {}
        for channel, angle in to_send.items():
            self.worker.send(set_angle(channel, angle))

    def _received(self, message: str) -> None:
        kind, values = parse_line(message)
        if kind == "FSR" and len(values) in (2, 3):
            try:
                channel, raw_value = int(values[0]), int(values[1])
                millivolts = (
                    int(values[2])
                    if len(values) == 3
                    else (raw_value * FSR_ADC_REFERENCE_MV + FSR_ADC_MAX // 2) // FSR_ADC_MAX
                )
            except ValueError:
                return
            if (
                channel == FSR_ADC_CHANNEL
                and 0 <= raw_value <= FSR_ADC_MAX
                and 0 <= millivolts <= FSR_ADC_REFERENCE_MV
            ):
                self._update_fsr_display(raw_value, millivolts)
        elif kind == "JOY" and len(values) in (7, 8):
            try:
                numbers = list(map(int, values))
            except ValueError:
                return
            if len(numbers) == 8:
                area, enabled, mode, raw_x, raw_y, speed_x, speed_y, armed = numbers
            else:
                # Compatibility with firmware from before the area selector.
                area = 0
                enabled, mode, raw_x, raw_y, speed_x, speed_y, armed = numbers
            self._update_joystick_display(
                bool(enabled), area, mode, raw_x, raw_y, speed_x, speed_y, bool(armed)
            )
        elif kind in ("ACK", "STATE") and len(values) == 2:
            try:
                channel, angle = int(values[0]), int(values[1])
            except ValueError:
                return
            if channel in self.cards:
                self.cards[channel].set_ack(angle)
        elif kind == "ACKCAL" and len(values) in (3, 4):
            try:
                numbers = list(map(int, values))
            except ValueError:
                return
            channel = numbers[0]
            if channel in self.cal_notes:
                if len(numbers) == 4:
                    _, low, center, high = numbers
                    self.cal_notes[channel].setText(
                        f"✓ Arduino använder {low} / {center} / {high} µs."
                    )
                else:
                    # Compatibility with the previous firmware response.
                    _, low, high = numbers
                    self.cal_notes[channel].setText(
                        f"✓ Äldre firmware använder {low}–{high} µs; ladda upp ny firmware."
                    )
        elif kind == "ERR":
            self.connection_note.setText(f"Arduino: {','.join(values)}")

    def _update_fsr_display(self, raw_value: int, millivolts: int) -> None:
        self.fsr_seen = True
        volts = f"{millivolts / 1000:.2f}".replace(".", ",")
        self.fsr_value_label.setText(f"{volts} V")
        self.fsr_raw_label.setText(f"RAW {raw_value} / {FSR_ADC_MAX}")
        self.fsr_value_label.setStyleSheet("color: #5BDAC9;")

    def _failed(self, reason: str) -> None:
        self._last_error = reason
        self.connection_note.setText(f"Anslutningsfel: {reason}")
        self.status.setText("●  ERROR")
        self.status.setObjectName("Bad")
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)

    def _worker_finished(self) -> None:
        self.arduino_ready = False
        self.pending.clear()
        self.joystick_seen = False
        self.fsr_seen = False
        self.joystick_status.setText("● EJ ANSLUTEN")
        self.joystick_status.setObjectName("Off")
        self._repolish(self.joystick_status)
        self.joystick_mode_label.setText("Väntar på Arduino")
        self.joystick_x_label.setText("–")
        self.joystick_y_label.setText("–")
        self.fsr_value_label.setText("Ej ansluten")
        self.fsr_raw_label.setText("RAW –")
        self.fsr_value_label.setStyleSheet("color: #F1F6FD;")
        for card in self.cards.values():
            card.set_joystick_axis("")
            card.reset_continuous_state()
        for card in self.cards.values():
            card.set_offline()
        self.status.setText("●  ERROR" if self._last_error else "●  OFFLINE")
        self.status.setObjectName("Bad" if self._last_error else "Off")
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)
        if not self._last_error:
            self.connection_note.setText("Frånkopplad. Servo-reglagen behåller sina målvinklar.")
        self.connect_button.setText("Anslut")
        self.connect_button.setObjectName("Primary")
        self.connect_button.style().unpolish(self.connect_button)
        self.connect_button.style().polish(self.connect_button)
        self.ports.setEnabled(True)
        self.refresh_button.setEnabled(True)
        if self.worker:
            self.worker.deleteLater()
            self.worker = None
        self.refresh_ports()

    def _center_all(self) -> None:
        for card in self.cards.values():
            card.set_angle(DEFAULT_ANGLE)
        # Even if a card already showed 90°, ask Arduino to center it.
        if self.arduino_ready:
            for channel in self.cards:
                self.pending[channel] = DEFAULT_ANGLE

    def _continuous_stop_commands(self) -> tuple[str, ...]:
        if not self.arduino_ready:
            return ()
        return tuple(set_angle(channel, DEFAULT_ANGLE) for channel in CONTINUOUS_SERVO_CHANNELS)

    def closeEvent(self, event) -> None:
        if self.worker is not None:
            self.worker.stop(self._continuous_stop_commands())
            self.worker.wait(2000)
        super().closeEvent(event)
