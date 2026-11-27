"""Monochrome 2D XY wrist joystick widget controlling Servo 01 (Ch 0) and Servo 02 (Ch 1)."""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from .config import DEFAULT_ANGLE, MAX_ANGLE, MIN_ANGLE, WRIST_JOYSTICK_CHANNELS
from .theme import TOKENS

_PAD_MARGIN = 14.0
_PUCK_RADIUS = 9.0


class WristJoystickWidget(QWidget):
    """Draggable 2D XY pad with position-hold and configurable axis orientation."""

    angles_changed = Signal(int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ch0_angle = DEFAULT_ANGLE
        self._ch1_angle = DEFAULT_ANGLE
        self._invert_x = False
        self._invert_y = False
        self._swap_axes = False
        self._dragging = False
        self.setCursor(Qt.CursorShape.CrossCursor)
        self.setToolTip(
            "Drag ball to steer Wrist (Servo 01 & Servo 02) simultaneously; "
            "release holds position, double-click centers to 90°."
        )

    def sizeHint(self) -> QSize:
        return QSize(136, 136)

    def minimumSizeHint(self) -> QSize:
        return QSize(120, 120)

    @staticmethod
    def _clamp_angle(angle: int) -> int:
        return max(MIN_ANGLE, min(MAX_ANGLE, int(angle)))

    def pad_rect(self) -> QRectF:
        """Return the inner coordinate travel rectangle inset by pad margin."""
        return QRectF(
            _PAD_MARGIN,
            _PAD_MARGIN,
            max(1.0, float(self.width()) - 2.0 * _PAD_MARGIN),
            max(1.0, float(self.height()) - 2.0 * _PAD_MARGIN),
        )

    def angles(self) -> tuple[int, int]:
        """Return current (ch0_angle, ch1_angle) in degrees."""
        return (self._ch0_angle, self._ch1_angle)

    def set_angles_silent(self, ch0_angle: int, ch1_angle: int) -> None:
        """Update both wrist angles without emitting angles_changed."""
        new_ch0 = self._clamp_angle(ch0_angle)
        new_ch1 = self._clamp_angle(ch1_angle)
        if new_ch0 == self._ch0_angle and new_ch1 == self._ch1_angle:
            return
        self._ch0_angle = new_ch0
        self._ch1_angle = new_ch1
        self.update()

    def set_channel_angle_silent(self, channel: int, angle: int) -> None:
        """Update a single wrist servo channel without emitting angles_changed."""
        clamped = self._clamp_angle(angle)
        ch0, ch1 = WRIST_JOYSTICK_CHANNELS
        if channel == ch0:
            if clamped != self._ch0_angle:
                self._ch0_angle = clamped
                self.update()
        elif channel == ch1:
            if clamped != self._ch1_angle:
                self._ch1_angle = clamped
                self.update()

    def set_axis_options(self, invert_x: bool, invert_y: bool, swap_axes: bool) -> None:
        """Configure axis inversion and X/Y swap without changing current angles."""
        inv_x = bool(invert_x)
        inv_y = bool(invert_y)
        swap = bool(swap_axes)
        if (inv_x, inv_y, swap) == (self._invert_x, self._invert_y, self._swap_axes):
            return
        self._invert_x = inv_x
        self._invert_y = inv_y
        self._swap_axes = swap
        self.update()

    def center(self) -> None:
        """Reset both wrist servos to 90 degrees and emit angles_changed."""
        self._ch0_angle = DEFAULT_ANGLE
        self._ch1_angle = DEFAULT_ANGLE
        self.update()
        self.angles_changed.emit(self._ch0_angle, self._ch1_angle)

    def pos_to_angles(self, x: float, y: float) -> tuple[int, int]:
        """Convert widget pixel coordinates (x, y) to clamped (ch0, ch1) angles."""
        area = self.pad_rect()
        nx = max(0.0, min(1.0, (float(x) - area.left()) / area.width()))
        ny = max(0.0, min(1.0, (float(y) - area.top()) / area.height()))
        x_norm = (1.0 - nx) if self._invert_x else nx
        y_norm = ny if self._invert_y else (1.0 - ny)
        span = float(MAX_ANGLE - MIN_ANGLE)
        x_angle = self._clamp_angle(round(MIN_ANGLE + x_norm * span))
        y_angle = self._clamp_angle(round(MIN_ANGLE + y_norm * span))
        if self._swap_axes:
            return (y_angle, x_angle)
        return (x_angle, y_angle)

    def angles_to_pos(self) -> QPointF:
        """Convert current (ch0, ch1) angles to widget pixel coordinates."""
        area = self.pad_rect()
        span = float(MAX_ANGLE - MIN_ANGLE)
        x_angle, y_angle = (
            (self._ch1_angle, self._ch0_angle)
            if self._swap_axes
            else (self._ch0_angle, self._ch1_angle)
        )
        x_norm = (x_angle - MIN_ANGLE) / span
        y_norm = (y_angle - MIN_ANGLE) / span
        nx = (1.0 - x_norm) if self._invert_x else x_norm
        ny = y_norm if self._invert_y else (1.0 - y_norm)
        return QPointF(area.left() + nx * area.width(), area.top() + ny * area.height())

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = True
            pos = event.position()
            ch0, ch1 = self.pos_to_angles(pos.x(), pos.y())
            self._ch0_angle = ch0
            self._ch1_angle = ch1
            self.update()
            self.angles_changed.emit(ch0, ch1)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._dragging or (event.buttons() & Qt.MouseButton.LeftButton):
            pos = event.position()
            ch0, ch1 = self.pos_to_angles(pos.x(), pos.y())
            if ch0 != self._ch0_angle or ch1 != self._ch1_angle:
                self._ch0_angle = ch0
                self._ch1_angle = ch1
                self.angles_changed.emit(ch0, ch1)
            self.update()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = False
            self.update()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = False
            self.center()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        outer = QRectF(
            0.5,
            0.5,
            max(1.0, float(self.width()) - 1.0),
            max(1.0, float(self.height()) - 1.0),
        )
        painter.setPen(QPen(QColor(TOKENS["rule_2"]), 1.0))
        painter.setBrush(QColor(TOKENS["paper_3"]))
        painter.drawRoundedRect(outer, 6.0, 6.0)

        area = self.pad_rect()
        cx = area.center().x()
        cy = area.center().y()

        painter.setPen(QPen(QColor(TOKENS["rule"]), 1.0))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(area)
        painter.drawLine(QPointF(area.left(), cy), QPointF(area.right(), cy))
        painter.drawLine(QPointF(cx, area.top()), QPointF(cx, area.bottom()))
        painter.drawEllipse(QPointF(cx, cy), 4.0, 4.0)

        tick_half = 3.0
        for frac in (0.25, 0.75):
            tx = area.left() + frac * area.width()
            ty = area.top() + frac * area.height()
            painter.drawLine(QPointF(tx, cy - tick_half), QPointF(tx, cy + tick_half))
            painter.drawLine(QPointF(cx - tick_half, ty), QPointF(cx + tick_half, ty))
            painter.drawLine(QPointF(tx, area.top()), QPointF(tx, area.top() + tick_half))
            painter.drawLine(QPointF(tx, area.bottom() - tick_half), QPointF(tx, area.bottom()))
            painter.drawLine(QPointF(area.left(), ty), QPointF(area.left() + tick_half, ty))
            painter.drawLine(QPointF(area.right() - tick_half, ty), QPointF(area.right(), ty))

        x_servo = "S02" if self._swap_axes else "S01"
        y_servo = "S01" if self._swap_axes else "S02"
        x_arrow = "←" if self._invert_x else "→"
        y_arrow = "↓" if self._invert_y else "↑"
        font = painter.font()
        font.setPixelSize(9)
        painter.setFont(font)
        painter.setPen(QColor(TOKENS["muted"]))
        painter.drawText(
            QRectF(6.0, 2.0, max(1.0, float(self.width()) - 12.0), 11.0),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            f"Y:{y_servo} {y_arrow}",
        )
        painter.drawText(
            QRectF(6.0, max(0.0, float(self.height()) - 13.0), max(1.0, float(self.width()) - 12.0), 11.0),
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            f"X:{x_servo} {x_arrow}",
        )

        puck = self.angles_to_pos()
        if self._dragging:
            painter.setPen(QPen(QColor(TOKENS["rule_2"]), 1.0))
            painter.drawLine(QPointF(area.left(), puck.y()), QPointF(area.right(), puck.y()))
            painter.drawLine(QPointF(puck.x(), area.top()), QPointF(puck.x(), area.bottom()))

        painter.setPen(QPen(QColor(TOKENS["cool"] if self._dragging else TOKENS["rule_2"]), 1.2))
        painter.setBrush(QColor(TOKENS["paper_2"]))
        painter.drawEllipse(puck, _PUCK_RADIUS + 2.0, _PUCK_RADIUS + 2.0)

        puck_fill = QColor(TOKENS["accent_hover"] if self._dragging else TOKENS["accent"])
        painter.setPen(QPen(QColor(TOKENS["ink"]), 0.8))
        painter.setBrush(puck_fill)
        painter.drawEllipse(puck, _PUCK_RADIUS, _PUCK_RADIUS)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(TOKENS["accent_ink"]))
        painter.drawEllipse(puck, 1.8, 1.8)
        painter.end()
