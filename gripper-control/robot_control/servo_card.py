"""Reusable position- and continuous-servo control card."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QSlider,
    QSpinBox, QVBoxLayout, QWidget,
)

from .config import DEFAULT_ANGLE, MAX_ANGLE, MIN_ANGLE


class ServoCard(QFrame):
    changed = Signal(int, int)  # channel, target angle

    def __init__(self, channel: int, name: str, continuous: bool = False, parent=None):
        super().__init__(parent)
        self.channel = channel
        self.continuous = continuous
        self.setObjectName("ServoCard")
        self.setMinimumWidth(370)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(24, 21, 24, 22)
        outer.setSpacing(12)

        eyebrow = QLabel(f"PCA9685  /  CHANNEL {channel:02d}")
        eyebrow.setObjectName("Kicker")
        outer.addWidget(eyebrow)

        top = QHBoxLayout()
        top.setSpacing(12)
        left = QVBoxLayout()
        left.setSpacing(2)
        servo_name = QLabel(name)
        servo_name.setObjectName("ServoName")
        left.addWidget(servo_name)
        subtitle = QLabel(
            "Speed control  ·  360° · center = stop"
            if continuous
            else "Position control  ·  0–180°"
        )
        subtitle.setObjectName("Description")
        left.addWidget(subtitle)
        top.addLayout(left)
        top.addStretch()
        self._command = DEFAULT_ANGLE
        self.value_text = QLabel("STOPP" if continuous else f"{DEFAULT_ANGLE:03d}°")
        self.value_text.setObjectName("Angle")
        top.addWidget(self.value_text)
        outer.addLayout(top)

        self.slider = None
        self.spinbox = None
        if continuous:
            outer.addSpacing(12)
            instruction = QLabel("HÅLL IN EN KNAPP · SLÄPP FÖR STOPP")
            instruction.setObjectName("SmallCaps")
            outer.addWidget(instruction)
            direction_row = QHBoxLayout()
            direction_row.setSpacing(10)
            self.reverse_button = QPushButton("⟲  Bakåt")
            self.reverse_button.setObjectName("HoldReverse")
            self.reverse_button.setToolTip("Håll inne för att rotera åt ena hållet")
            self.forward_button = QPushButton("⟳  Framåt")
            self.forward_button.setObjectName("HoldForward")
            self.forward_button.setToolTip("Håll inne för att rotera åt andra hållet")
            direction_row.addWidget(self.reverse_button, 1)
            direction_row.addWidget(self.forward_button, 1)
            outer.addLayout(direction_row)
            self.reverse_button.pressed.connect(lambda: self._continuous_pressed(MIN_ANGLE))
            self.forward_button.pressed.connect(lambda: self._continuous_pressed(MAX_ANGLE))
            self.reverse_button.released.connect(self._continuous_released)
            self.forward_button.released.connect(self._continuous_released)
        else:
            outer.addSpacing(12)
            target_label = QLabel("KOMMENDERAD MÅLVINKEL")
            target_label.setObjectName("SmallCaps")
            outer.addWidget(target_label)

            self.slider = QSlider(Qt.Orientation.Horizontal)
            self.slider.setRange(MIN_ANGLE, MAX_ANGLE)
            self.slider.setSingleStep(1)
            self.slider.setPageStep(10)
            self.slider.setValue(DEFAULT_ANGLE)
            self.slider.setToolTip("Önskad vinkel, inte uppmätt verklig vinkel")
            outer.addWidget(self.slider)

            ticks = QHBoxLayout()
            for idx, label in enumerate(("0°", "90°", "180°")):
                tick = QLabel(label)
                tick.setObjectName("Tick")
                ticks.addWidget(tick)
                if idx < 2:
                    ticks.addStretch()
            outer.addLayout(ticks)

            bottom = QHBoxLayout()
            bottom.setSpacing(8)
            target_input_label = QLabel("Ange vinkel")
            target_input_label.setObjectName("Description")
            bottom.addWidget(target_input_label)
            self.spinbox = QSpinBox()
            self.spinbox.setRange(MIN_ANGLE, MAX_ANGLE)
            self.spinbox.setSuffix(" °")
            self.spinbox.setFixedWidth(105)
            self.spinbox.setValue(DEFAULT_ANGLE)
            bottom.addWidget(self.spinbox)
            bottom.addStretch()
            for angle in (0, 90, 180):
                button = QPushButton(f"{angle}°")
                button.setObjectName("Quick")
                button.clicked.connect(lambda checked=False, a=angle: self.set_angle(a))
                bottom.addWidget(button)
            outer.addLayout(bottom)

        self.feedback = QLabel("Arduino ej ansluten")
        self.feedback.setObjectName("Description")
        outer.addSpacing(8)
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
            label = "bakåt" if angle == MIN_ANGLE else "framåt" if angle == MAX_ANGLE else "stopp"
            self.feedback.setText(f"✓ Arduino bekräftade: {label}")
        else:
            self.feedback.setText(f"✓ Arduino bekräftade målvärde: {angle}°")
        self.feedback.setStyleSheet("color: #5BDAC9;")

    def set_offline(self) -> None:
        self.feedback.setText("Arduino ej ansluten")
        self.feedback.setStyleSheet("color: #92A3B6;")

    def set_joystick_axis(self, axis: str) -> None:
        """Highlight this card when its channel is controlled by X or Y."""
        self.setProperty("joystickAxis", axis)
        self.style().unpolish(self)
        self.style().polish(self)

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
            self.value_text.setText("BAKÅT")
        elif value == MAX_ANGLE:
            self.value_text.setText("FRAMÅT")
        else:
            self.value_text.setText("STOPP")

    def reset_continuous_state(self) -> None:
        """Reset the visual hold-button state without sending serial data."""
        if self.continuous:
            self._command = DEFAULT_ANGLE
            self._update_continuous_display(DEFAULT_ANGLE)
