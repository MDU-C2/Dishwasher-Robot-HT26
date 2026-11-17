"""Reusable position- and continuous-servo control card."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QSlider,
    QSpinBox, QVBoxLayout,
)

from .config import DEFAULT_ANGLE, MAX_ANGLE, MIN_ANGLE


class ServoCard(QFrame):
    changed = Signal(int, int)  # channel, target angle

    def __init__(self, channel: int, name: str, continuous: bool = False, parent=None):
        super().__init__(parent)
        self.channel = channel
        self.continuous = continuous
        self.setObjectName("ServoCard")
        self.setProperty("continuous", continuous)
        self.setMinimumWidth(390)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(20, 18, 20, 18)
        outer.setSpacing(11)

        meta_row = QHBoxLayout()
        meta_row.setSpacing(8)
        channel_tag = QLabel(f"CH {channel:02d}")
        channel_tag.setObjectName("ChannelTag")
        meta_row.addWidget(channel_tag)

        eyebrow = QLabel(f"PCA9685 · PIN {channel}")
        eyebrow.setObjectName("Kicker")
        meta_row.addWidget(eyebrow)
        meta_row.addStretch()

        type_badge = QLabel("360° CONTINUOUS" if continuous else "180° POSITIONAL")
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
            "Hold button to run · Release to stop"
            if continuous
            else "Target angle control · 0° to 180°"
        )
        subtitle.setObjectName("Description")
        subtitle.setWordWrap(True)
        left.addWidget(subtitle)
        top.addLayout(left)
        top.addStretch()
        self._command = DEFAULT_ANGLE
        self.value_text = QLabel("STOP" if continuous else f"{DEFAULT_ANGLE:03d}°")
        self.value_text.setObjectName("Angle")
        self.value_text.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        top.addWidget(self.value_text)
        outer.addLayout(top)

        self.slider = None
        self.spinbox = None
        if continuous:
            outer.addSpacing(6)
            instruction = QLabel("HOLD DIRECTION TO DRIVE · RELEASE OR PRESS STOP")
            instruction.setObjectName("SmallCaps")
            outer.addWidget(instruction)

            direction_row = QHBoxLayout()
            direction_row.setSpacing(8)
            self.reverse_button = QPushButton("⟲  HOLD REVERSE")
            self.reverse_button.setObjectName("HoldReverse")
            self.reverse_button.setToolTip("Hold to rotate in reverse direction")

            self.stop_button = QPushButton("■  STOP")
            self.stop_button.setObjectName("StopButton")
            self.stop_button.setToolTip("Stop this continuous servo immediately (90°)")
            self.stop_button.clicked.connect(lambda: self.set_angle(DEFAULT_ANGLE))

            self.forward_button = QPushButton("⟳  HOLD FORWARD")
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
        else:
            outer.addSpacing(4)
            target_label = QLabel("COMMANDED TARGET ANGLE")
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
            tick_labels = ("0° (MIN)", "45°", "90° (CENTER)", "135°", "180° (MAX)")
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
            target_input_label = QLabel("Set angle")
            target_input_label.setObjectName("Description")
            nudge_row.addWidget(target_input_label)
            self.spinbox = QSpinBox()
            self.spinbox.setRange(MIN_ANGLE, MAX_ANGLE)
            self.spinbox.setSuffix(" °")
            self.spinbox.setFixedWidth(96)
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
            quick_label = QLabel("Quick jump")
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

        self.feedback = QLabel("Arduino not connected")
        self.feedback.setObjectName("Description")
        self.feedback.setStyleSheet("color: #8B949E;")
        outer.addSpacing(4)
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
            label = "reverse" if angle == MIN_ANGLE else "forward" if angle == MAX_ANGLE else "stop"
            self.feedback.setText(f"✓ Arduino confirmed: {label}")
        else:
            self.feedback.setText(f"✓ Arduino accepted target: {angle}°")
        self.feedback.setStyleSheet("color: #4ADE80;")

    def set_offline(self) -> None:
        self.feedback.setText("Arduino not connected")
        self.feedback.setStyleSheet("color: #8B949E;")

    def _slider_changed(self, value: int) -> None:
        self._command = value
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(value)
        self.spinbox.blockSignals(False)
        self.value_text.setText(f"{value:03d}°")
        self.changed.emit(self.channel, value)

    def _continuous_pressed(self, value: int) -> None:
        self.set_angle(value)

    def _continuous_released(self) -> None:
        self.set_angle(DEFAULT_ANGLE)

    def _update_continuous_display(self, value: int) -> None:
        if value == MIN_ANGLE:
            self.value_text.setText("REVERSE")
        elif value == MAX_ANGLE:
            self.value_text.setText("FORWARD")
        else:
            self.value_text.setText("STOP")

    def reset_continuous_state(self) -> None:
        """Reset the visual hold-button state without sending serial data."""
        if self.continuous:
            self._command = DEFAULT_ANGLE
            self._update_continuous_display(DEFAULT_ANGLE)
