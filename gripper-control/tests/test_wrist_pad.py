import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QPointF, Qt
from PySide6.QtGui import QMouseEvent
from PySide6.QtWidgets import QApplication

from robot_control.config import (
    DEFAULT_OPTICAL_MOTION_THRESHOLD,
    DEFAULT_OPTICAL_SQUAL_THRESHOLD,
    WRIST_JOYSTICK_CHANNELS,
)
from robot_control.wrist_pad import WristJoystickWidget

_APP = QApplication.instance() or QApplication([])


def _mouse_event(
    event_type: QEvent.Type,
    pos: QPointF,
    button: Qt.MouseButton = Qt.MouseButton.LeftButton,
    buttons: Qt.MouseButton = Qt.MouseButton.LeftButton,
) -> QMouseEvent:
    return QMouseEvent(
        event_type,
        pos,
        pos,
        button,
        buttons,
        Qt.KeyboardModifier.NoModifier,
    )


class WristJoystickWidgetTests(unittest.TestCase):
    def setUp(self):
        self.pad = WristJoystickWidget()
        self.pad.resize(136, 136)

    def test_config_defaults_and_channels(self):
        self.assertEqual(WRIST_JOYSTICK_CHANNELS, (0, 1))
        self.assertEqual(DEFAULT_OPTICAL_MOTION_THRESHOLD, 3)
        self.assertEqual(DEFAULT_OPTICAL_SQUAL_THRESHOLD, 0)

    def test_default_corners_center_and_clamping(self):
        area = self.pad.pad_rect()
        cx, cy = area.center().x(), area.center().y()

        self.assertEqual(self.pad.pos_to_angles(cx, cy), (90, 90))
        self.assertEqual(self.pad.pos_to_angles(area.left(), area.top()), (0, 180))
        self.assertEqual(self.pad.pos_to_angles(area.right(), area.top()), (180, 180))
        self.assertEqual(self.pad.pos_to_angles(area.left(), area.bottom()), (0, 0))
        self.assertEqual(self.pad.pos_to_angles(area.right(), area.bottom()), (180, 0))

        # Out-of-bounds points clamp to pad bounds
        self.assertEqual(self.pad.pos_to_angles(-100.0, -50.0), (0, 180))
        self.assertEqual(self.pad.pos_to_angles(500.0, 500.0), (180, 0))

        # Round-trip angles_to_pos -> pos_to_angles
        for ch0, ch1 in ((0, 180), (180, 0), (90, 90), (45, 135), (135, 45)):
            self.pad.set_angles_silent(ch0, ch1)
            pos = self.pad.angles_to_pos()
            self.assertEqual(self.pad.pos_to_angles(pos.x(), pos.y()), (ch0, ch1))

    def test_invert_x_and_invert_y(self):
        area = self.pad.pad_rect()
        cx, cy = area.center().x(), area.center().y()

        self.pad.set_axis_options(invert_x=True, invert_y=False, swap_axes=False)
        self.assertEqual(self.pad.pos_to_angles(area.left(), cy), (180, 90))
        self.assertEqual(self.pad.pos_to_angles(area.right(), cy), (0, 90))
        self.assertEqual(self.pad.pos_to_angles(area.left(), area.top()), (180, 180))
        self.pad.set_angles_silent(180, 45)
        pos = self.pad.angles_to_pos()
        self.assertEqual(self.pad.pos_to_angles(pos.x(), pos.y()), (180, 45))

        self.pad.set_axis_options(invert_x=False, invert_y=True, swap_axes=False)
        self.assertEqual(self.pad.pos_to_angles(cx, area.top()), (90, 0))
        self.assertEqual(self.pad.pos_to_angles(cx, area.bottom()), (90, 180))
        self.assertEqual(self.pad.pos_to_angles(area.right(), area.bottom()), (180, 180))
        self.pad.set_angles_silent(30, 150)
        pos = self.pad.angles_to_pos()
        self.assertEqual(self.pad.pos_to_angles(pos.x(), pos.y()), (30, 150))

    def test_swap_axes_and_combined_inversions(self):
        area = self.pad.pad_rect()
        cx, cy = area.center().x(), area.center().y()

        self.pad.set_axis_options(invert_x=False, invert_y=False, swap_axes=True)
        # X drives Ch 1, Y drives Ch 0
        self.assertEqual(self.pad.pos_to_angles(area.right(), cy), (90, 180))
        self.assertEqual(self.pad.pos_to_angles(cx, area.top()), (180, 90))
        self.assertEqual(self.pad.pos_to_angles(area.left(), area.top()), (180, 0))
        self.assertEqual(self.pad.pos_to_angles(area.right(), area.bottom()), (0, 180))

        self.pad.set_angles_silent(25, 160)
        pos = self.pad.angles_to_pos()
        self.assertEqual(self.pad.pos_to_angles(pos.x(), pos.y()), (25, 160))

        # Combine swap_axes with invert_x and invert_y
        self.pad.set_axis_options(invert_x=True, invert_y=True, swap_axes=True)
        self.assertEqual(self.pad.pos_to_angles(area.left(), area.top()), (0, 180))
        self.assertEqual(self.pad.pos_to_angles(area.right(), area.bottom()), (180, 0))

    def test_mouse_drag_holds_position_on_release(self):
        area = self.pad.pad_rect()
        emitted: list[tuple[int, int]] = []
        self.pad.angles_changed.connect(lambda a, b: emitted.append((a, b)))

        QApplication.sendEvent(
            self.pad,
            _mouse_event(QEvent.Type.MouseButtonPress, QPointF(area.right(), area.top())),
        )
        self.assertEqual(self.pad.angles(), (180, 180))
        self.assertEqual(emitted[-1], (180, 180))

        quarter_pt = QPointF(
            area.left() + 0.25 * area.width(),
            area.top() + 0.75 * area.height(),
        )
        QApplication.sendEvent(
            self.pad,
            _mouse_event(
                QEvent.Type.MouseMove,
                quarter_pt,
                button=Qt.MouseButton.NoButton,
                buttons=Qt.MouseButton.LeftButton,
            ),
        )
        self.assertEqual(self.pad.angles(), (45, 45))
        self.assertEqual(emitted[-1], (45, 45))

        count_before_release = len(emitted)
        QApplication.sendEvent(
            self.pad,
            _mouse_event(
                QEvent.Type.MouseButtonRelease,
                quarter_pt,
                button=Qt.MouseButton.LeftButton,
                buttons=Qt.MouseButton.NoButton,
            ),
        )
        # Release must hold exact position and not snap back to 90, 90
        self.assertEqual(self.pad.angles(), (45, 45))
        self.assertEqual(len(emitted), count_before_release)

    def test_double_click_and_center_reset_to_90(self):
        area = self.pad.pad_rect()
        emitted: list[tuple[int, int]] = []
        self.pad.angles_changed.connect(lambda a, b: emitted.append((a, b)))

        self.pad.set_angles_silent(15, 165)
        self.assertEqual(emitted, [])

        QApplication.sendEvent(
            self.pad,
            _mouse_event(QEvent.Type.MouseButtonDblClick, area.topLeft()),
        )
        self.assertEqual(self.pad.angles(), (90, 90))
        self.assertEqual(emitted[-1], (90, 90))

        self.pad.set_channel_angle_silent(0, 140)
        self.pad.set_channel_angle_silent(1, 20)
        self.pad.set_channel_angle_silent(2, 55)  # ignored non-wrist channel
        self.assertEqual(self.pad.angles(), (140, 20))

        self.pad.center()
        self.assertEqual(self.pad.angles(), (90, 90))
        self.assertEqual(emitted[-1], (90, 90))


if __name__ == "__main__":
    unittest.main()
