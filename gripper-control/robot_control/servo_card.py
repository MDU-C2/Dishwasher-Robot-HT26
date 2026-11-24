"""Reusable position- and continuous-servo control card with optional FSR telemetry."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QProgressBar, QPushButton, QSlider,
    QSpinBox, QVBoxLayout,
)

from .config import (
    DEFAULT_ANGLE, DEFAULT_FSR_GRIP_THRESHOLD, FSR_PINS, FSR_STATE_LABELS,
    MAX_ANGLE, MIN_ANGLE,
)


class ServoCard(QFrame):
    changed = Signal(int, int)  # channel, target angle

    def __init__(self, channel: int, name: str, continuous: bool = False, parent=None):
        super().__init__(parent)
        self.channel = channel
        self.continuous = continuous
        self.setObjectName("ServoCard")
        self.setProperty("continuous", continuous)
        self.setMinimumWidth(380)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 14, 16, 14)
        outer.setSpacing(10)

        meta_row = QHBoxLayout()
        meta_row.setSpacing(8)
        channel_tag = QLabel(f"Ch {channel}, Pin {channel}")
        channel_tag.setObjectName("ChannelTag")
        meta_row.addWidget(channel_tag)
        meta_row.addStretch()

        type_badge = QLabel("360° continuous" if continuous else "180° positional")
        type_badge.setObjectName("TypeBadgeCont" if continuous else "TypeBadgePos")
        meta_row.addWidget(type_badge)
        outer.addLayout(meta_row)

        top = QHBoxLayout()
        top.setSpacing(12)
        left = QVBoxLayout()
        left.setSpacing(2)
        servo_name = QLabel(name)
        servo_name.setObjectName("ServoName")
        left.addWidget(servo_name)
        subtitle = QLabel(
            "Hold to drive, release stops"
            if continuous
            else "Target angle (0° to 180°)"
        )
        subtitle.setObjectName("Description")
        subtitle.setWordWrap(True)
        left.addWidget(subtitle)
        top.addLayout(left, 1)
        self._command = DEFAULT_ANGLE
        self._fsr_state = 0
        self._fsr_limit = DEFAULT_FSR_GRIP_THRESHOLD
        self.value_text = QLabel("Stop" if continuous else f"{DEFAULT_ANGLE}°")
        self.value_text.setObjectName("Angle")
        self.value_text.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        top.addWidget(self.value_text)
        outer.addLayout(top)

        self.slider = None
        self.spinbox = None
        self.fsr_bar = None
        self.fsr_state_badge = None
        self.fsr_value_label = None

        if continuous:
            instruction = QLabel("Hold to drive")
            instruction.setObjectName("SmallCaps")
            outer.addWidget(instruction)

            direction_row = QHBoxLayout()
            direction_row.setSpacing(8)
            self.reverse_button = QPushButton("Hold Reverse")
            self.reverse_button.setObjectName("HoldReverse")
            self.reverse_button.setToolTip("Hold to rotate in reverse direction")

            self.stop_button = QPushButton("Stop")
            self.stop_button.setObjectName("StopButton")
            self.stop_button.setToolTip("Stop this continuous servo immediately (90°)")
            self.stop_button.clicked.connect(lambda: self.set_angle(DEFAULT_ANGLE))

            self.forward_button = QPushButton("Hold Forward")
            self.forward_button.setObjectName("HoldForward")
            self.forward_button.setToolTip("Hold to rotate in forward direction")

            direction_row.addWidget(self.reverse_button, 2)
            direction_row.addWidget(self.stop_button, 1)
            direction_row.addWidget(self.forward_button, 2)
            outer.addLayout(direction_row)

            self.reverse_button.pressed.connect(lambda: self._continuous_pressed(MIN_ANGLE))
            self.forward_button.pressed.connect(lambda: self._continuous_pressed(MAX_ANGLE))
            self.reverse_button.released.connect(self._continuous_released)
            self.forward_button.released.connect(self._continuous_released)

            fsr_panel = QFrame()
            fsr_panel.setObjectName("SubPanel")
            fsr_layout = QVBoxLayout(fsr_panel)
            fsr_layout.setContentsMargins(10, 8, 10, 8)
            fsr_layout.setSpacing(6)

            fsr_header = QHBoxLayout()
            fsr_header.setSpacing(8)
            pin_name = FSR_PINS.get(channel, "Analog")
            fsr_title = QLabel(f"FSR Force ({pin_name})")
            fsr_title.setObjectName("Kicker")
            fsr_header.addWidget(fsr_title)
            fsr_header.addStretch()
            self.fsr_state_badge = QLabel(FSR_STATE_LABELS[0])
            self.fsr_state_badge.setObjectName("FsrState0")
            fsr_header.addWidget(self.fsr_state_badge)
            fsr_layout.addLayout(fsr_header)

            self.fsr_bar = QProgressBar()
            self.fsr_bar.setRange(0, DEFAULT_FSR_GRIP_THRESHOLD)
            self.fsr_bar.setValue(0)
            self.fsr_bar.setTextVisible(False)
            self.fsr_bar.setProperty("fsrState", "0")
            fsr_layout.addWidget(self.fsr_bar)

            self.fsr_value_label = QLabel(
                f"Pressure: 0 / {DEFAULT_FSR_GRIP_THRESHOLD} (Raw: 0)"
            )
            self.fsr_value_label.setObjectName("Description")
            fsr_layout.addWidget(self.fsr_value_label)
            outer.addWidget(fsr_panel)
        else:
            target_label = QLabel("Target angle")
            target_label.setObjectName("SmallCaps")
            outer.addWidget(target_label)

            self.slider = QSlider(Qt.Orientation.Horizontal)
            self.slider.setRange(MIN_ANGLE, MAX_ANGLE)
            self.slider.setSingleStep(1)
            self.slider.setPageStep(10)
            self.slider.setValue(DEFAULT_ANGLE)
            self.slider.setToolTip("Requested angle, not measured physical position")
            outer.addWidget(self.slider)

            ticks = QHBoxLayout()
            tick_labels = ("0°", "45°", "90°", "135°", "180°")
            for idx, label in enumerate(tick_labels):
                tick = QLabel(label)
                tick.setObjectName("Tick")
                ticks.addWidget(tick)
                if idx < len(tick_labels) - 1:
                    ticks.addStretch()
            outer.addLayout(ticks)

            controls_box = QFrame()
            controls_box.setObjectName("SubPanel")
            controls_layout = QVBoxLayout(controls_box)
            controls_layout.setContentsMargins(10, 8, 10, 8)
            controls_layout.setSpacing(8)

            nudge_row = QHBoxLayout()
            nudge_row.setSpacing(6)
            target_input_label = QLabel("Angle")
            target_input_label.setObjectName("Description")
            nudge_row.addWidget(target_input_label)
            self.spinbox = QSpinBox()
            self.spinbox.setRange(MIN_ANGLE, MAX_ANGLE)
            self.spinbox.setSuffix("°")
            self.spinbox.setFixedWidth(82)
            self.spinbox.setValue(DEFAULT_ANGLE)
            nudge_row.addWidget(self.spinbox)
            nudge_row.addStretch()
            for delta in (-10, -1, 1, 10):
                sign = "+" if delta > 0 else ""
                nudge_btn = QPushButton(f"{sign}{delta}°")
                nudge_btn.setObjectName("Nudge")
                nudge_btn.setToolTip(f"Nudge target angle by {sign}{delta}°")
                nudge_btn.clicked.connect(
                    lambda checked=False, d=delta: self.set_angle(self.angle() + d)
                )
                nudge_row.addWidget(nudge_btn)
            controls_layout.addLayout(nudge_row)

            presets_row = QHBoxLayout()
            presets_row.setSpacing(6)
            quick_label = QLabel("Presets")
            quick_label.setObjectName("Description")
            presets_row.addWidget(quick_label)
            presets_row.addStretch()
            for angle in (0, 45, 90, 135, 180):
                button = QPushButton(f"{angle}°")
                button.setObjectName("Quick")
                button.clicked.connect(lambda checked=False, a=angle: self.set_angle(a))
                presets_row.addWidget(button)
            controls_layout.addLayout(presets_row)

            outer.addWidget(controls_box)

        self.feedback = QLabel("Disconnected")
        self.feedback.setObjectName("Description")
        self.feedback.setStyleSheet("color: #8A9099;")
        outer.addWidget(self.feedback)

        if not continuous:
            self.slider.valueChanged.connect(self._slider_changed)
            self.spinbox.valueChanged.connect(self.slider.setValue)

    def set_angle(self, value: int) -> None:
        value = max(MIN_ANGLE, min(MAX_ANGLE, int(value)))
        if self.continuous:
            self._command = value
            self._update_continuous_display(value)
            self.changed.emit(self.channel, value)
        else:
            self.slider.setValue(value)

    def angle(self) -> int:
        return self._command if self.continuous else self.slider.value()

    def set_ack(self, angle: int) -> None:
        if self.continuous:
            if angle == MIN_ANGLE:
                label = "reverse"
            elif angle == MAX_ANGLE:
                label = "forward"
            elif angle == DEFAULT_ANGLE:
                label = "stop"
            elif angle < DEFAULT_ANGLE:
                label = "reverse (slow)"
            else:
                label = "forward (slow)"
            self.feedback.setText(f"Confirmed: {label}")
        else:
            self.feedback.setText(f"Target confirmed: {angle}°")
        self.feedback.setStyleSheet("color: #D8DCE2;")

    def set_offline(self) -> None:
        self.feedback.setText("Disconnected")
        self.feedback.setStyleSheet("color: #8A9099;")

    def update_fsr(self, net: int, raw: int, state: int, grip_limit: int) -> None:
        if not self.continuous or not self.fsr_bar or not self.fsr_state_badge or not self.fsr_value_label:
            return
        safe_limit = max(1, int(grip_limit))
        self._fsr_limit = safe_limit
        self.fsr_bar.setRange(0, safe_limit)
        self.fsr_bar.setValue(max(0, min(int(net), safe_limit)))
        self.fsr_value_label.setText(
            f"Pressure: {net} / {safe_limit} (Raw: {raw})"
        )
        clamped_state = state if state in FSR_STATE_LABELS else 0
        if clamped_state != self._fsr_state:
            self._fsr_state = clamped_state
            self.fsr_state_badge.setObjectName(f"FsrState{clamped_state}")
            self.fsr_state_badge.setText(FSR_STATE_LABELS[clamped_state])
            self.fsr_state_badge.style().unpolish(self.fsr_state_badge)
            self.fsr_state_badge.style().polish(self.fsr_state_badge)
            self.fsr_bar.setProperty("fsrState", str(clamped_state))
            self.fsr_bar.style().unpolish(self.fsr_bar)
            self.fsr_bar.style().polish(self.fsr_bar)

    def sync_continuous_state(self, value: int) -> None:
        """Update continuous visual state from firmware telemetry without emitting serial commands."""
        if self.continuous:
            value = max(MIN_ANGLE, min(MAX_ANGLE, int(value)))
            self._command = value
            self._update_continuous_display(value)

    def _slider_changed(self, value: int) -> None:
        self._command = value
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(value)
        self.spinbox.blockSignals(False)
        self.value_text.setText(f"{value}°")
        self.changed.emit(self.channel, value)

    def _continuous_pressed(self, value: int) -> None:
        self.set_angle(value)

    def _continuous_released(self) -> None:
        self.set_angle(DEFAULT_ANGLE)

    def _update_continuous_display(self, value: int) -> None:
        if value == MIN_ANGLE:
            self.value_text.setText("Reverse")
        elif value == MAX_ANGLE:
            self.value_text.setText("Forward")
        elif value == DEFAULT_ANGLE:
            self.value_text.setText("Stop")
        elif value < DEFAULT_ANGLE:
            self.value_text.setText("Rev slow")
        else:
            self.value_text.setText("Fwd slow")

    def reset_continuous_state(self) -> None:
        """Reset the visual hold-button state without sending serial data."""
        if self.continuous:
            self._command = DEFAULT_ANGLE
            self._update_continuous_display(DEFAULT_ANGLE)
            self.update_fsr(0, 0, 0, self._fsr_limit)
