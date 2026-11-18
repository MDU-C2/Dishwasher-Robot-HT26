"""Refined dark, monochrome brutalist styling for the desktop control panel."""

STYLESHEET = """
QWidget {
    background: transparent;
    color: #F1F1F1;
    font-family: "Arial", "Helvetica Neue", sans-serif;
    font-size: 13px;
}
QMainWindow, QScrollArea#AppScroll, QWidget#Page {
    background: #0D0E10;
}
QScrollArea#AppScroll { border: 0; }
QScrollBar:vertical {
    background: #0D0E10;
    width: 12px;
    border-left: 1px solid #2A2E35;
}
QScrollBar::handle:vertical {
    background: #3A404A;
    min-height: 36px;
    border: 1px solid #58606E;
}
QScrollBar::handle:vertical:hover {
    background: #6B7280;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
    background: none;
    border: none;
    height: 0px;
}

QFrame#Panel, QFrame#ServoCard, QFrame#CalibrationCard {
    background: #15171A;
    border: 2px solid #2E333B;
    border-radius: 0;
}
QFrame#ServoCard { background: #181B1F; }
QFrame#CalibrationCard { background: #15171A; }
QFrame#ServoCard:hover, QFrame#CalibrationCard:hover {
    border-color: #58606E;
}
QFrame#ServoCard[continuous="true"], QFrame#CalibrationCard[continuous="true"] {
    border-left: 4px solid #E5A93C;
}
QFrame#ServoCard[continuous="false"], QFrame#CalibrationCard[continuous="false"] {
    border-left: 4px solid #ECEFF4;
}

QFrame#MotionPanel, QFrame#SubPanel {
    background: #101214;
    border: 1px solid #2A2E35;
    border-radius: 0;
    padding: 4px;
}
QFrame#GripMasterPanel {
    background: #16191E;
    border: 2px solid #E5A93C;
    border-left: 6px solid #E5A93C;
    border-radius: 0;
}

QProgressBar {
    background: #0D0E10;
    border: 1px solid #4B5262;
    border-radius: 0;
    height: 10px;
    text-align: center;
    color: transparent;
}
QProgressBar::chunk { background: #E5A93C; }
QProgressBar[fsrState="0"]::chunk, QProgressBar[optState="2"]::chunk { background: #4B5563; }
QProgressBar[fsrState="1"]::chunk { background: #FACC15; }
QProgressBar[fsrState="2"]::chunk, QProgressBar[optState="0"]::chunk { background: #22C55E; }
QProgressBar[optState="1"]::chunk { background: #F97316; }
QProgressBar[fsrState="3"]::chunk, QProgressBar[fsrState="4"]::chunk,
QProgressBar[optState="3"]::chunk { background: #EF4444; }

QLabel#FsrState0, QLabel#OptState2 {
    color: #9CA3AF;
    background: #16181C;
    border: 1px solid #374151;
    padding: 2px 6px;
    font-size: 10px;
    font-weight: 900;
}
QLabel#FsrState1 {
    color: #FACC15;
    background: #2A2008;
    border: 1px solid #EAB308;
    padding: 2px 6px;
    font-size: 10px;
    font-weight: 900;
}
QLabel#OptState1 {
    color: #FDBA74;
    background: #2D1606;
    border: 1px solid #F97316;
    padding: 2px 6px;
    font-size: 10px;
    font-weight: 900;
}
QLabel#FsrState2, QLabel#OptState0 {
    color: #4ADE80;
    background: #0D2818;
    border: 1px solid #22C55E;
    padding: 2px 6px;
    font-size: 10px;
    font-weight: 900;
}
QLabel#FsrState3, QLabel#FsrState4, QLabel#OptState3 {
    color: #F87171;
    background: #2D1215;
    border: 1px solid #EF4444;
    padding: 2px 6px;
    font-size: 10px;
    font-weight: 900;
}

QFrame#SectionBanner {
    background: #16181C;
    border: 1px solid #2A2E35;
    border-left: 4px solid #F1F1F1;
    padding: 8px 12px;
}
QFrame#SectionBanner[continuous="true"] {
    border-left: 4px solid #E5A93C;
}

QLabel#Kicker, QLabel#SmallCaps {
    color: #D0D4DC;
    font-size: 11px;
    font-weight: 900;
    letter-spacing: 1.4px;
}
QLabel#Hero {
    color: #FFFFFF;
    font-size: 36px;
    font-weight: 900;
    letter-spacing: -0.5px;
}
QLabel#BodyMuted, QLabel#Description, QLabel#Tick {
    color: #9CA3AF;
}
QLabel#SectionTitle, QLabel#ServoName {
    color: #FFFFFF;
    font-size: 18px;
    font-weight: 900;
}
QLabel#ServoName { font-size: 20px; }
QLabel#Angle {
    color: #FFFFFF;
    font-size: 44px;
    font-weight: 900;
}
QLabel#ChannelTag {
    background: #F1F1F1;
    color: #0D0E10;
    border: 2px solid #F1F1F1;
    padding: 4px 8px;
    font-weight: 900;
    font-size: 11px;
}
QLabel#TypeBadgePos {
    background: #252A32;
    color: #E5E9F0;
    border: 1px solid #4C566A;
    padding: 3px 7px;
    font-size: 10px;
    font-weight: 900;
    letter-spacing: 0.8px;
}
QLabel#TypeBadgeCont {
    background: #2C2212;
    color: #FACC15;
    border: 1px solid #A16207;
    padding: 3px 7px;
    font-size: 10px;
    font-weight: 900;
    letter-spacing: 0.8px;
}

QLabel#Good {
    color: #4ADE80;
    background: #0D2818;
    border: 2px solid #22C55E;
    padding: 6px 12px;
    font-weight: 900;
    letter-spacing: 0.8px;
}
QLabel#Waiting {
    color: #FACC15;
    background: #2A2008;
    border: 2px solid #EAB308;
    padding: 6px 12px;
    font-weight: 900;
    letter-spacing: 0.8px;
}
QLabel#Bad {
    color: #F87171;
    background: #2D1215;
    border: 2px solid #EF4444;
    padding: 6px 12px;
    font-weight: 900;
    letter-spacing: 0.8px;
}
QLabel#Off {
    color: #9CA3AF;
    background: #16181C;
    border: 2px solid #374151;
    padding: 6px 12px;
    font-weight: 900;
    letter-spacing: 0.8px;
}

QTabWidget#MainTabs::pane {
    background: #0D0E10;
    border: 2px solid #2E333B;
    top: -2px;
}
QTabBar::tab {
    background: #181B1F;
    color: #B8BEC8;
    border: 2px solid #2E333B;
    border-bottom: 0;
    padding: 11px 20px;
    margin-right: 5px;
    font-weight: 900;
    letter-spacing: 1px;
}
QTabBar::tab:hover:!selected {
    background: #22262C;
    color: #FFFFFF;
}
QTabBar::tab:selected {
    background: #F1F1F1;
    color: #0D0E10;
    border-color: #F1F1F1;
}
QTabBar::tab:!selected { margin-top: 4px; }

QComboBox, QSpinBox {
    background: #0D0E10;
    color: #F1F1F1;
    border: 2px solid #4B5262;
    border-radius: 0;
    padding: 7px 10px;
    min-height: 20px;
    selection-background-color: #E8E8E8;
    selection-color: #111111;
}
QComboBox:focus, QSpinBox:focus { border-color: #FFFFFF; }
QComboBox:disabled, QSpinBox:disabled {
    background: #141619;
    color: #6B7280;
    border-color: #2A2E35;
}
QComboBox::drop-down { border: 0; width: 25px; }
QComboBox QAbstractItemView {
    background: #181B1F;
    color: #F1F1F1;
    border: 2px solid #D8D8D8;
    selection-background-color: #E8E8E8;
    selection-color: #111111;
}

QCheckBox {
    spacing: 8px;
    color: #E5E7EB;
    font-weight: 700;
}
QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 2px solid #9AA2AE;
    background: #0D0E10;
    border-radius: 0;
}
QCheckBox::indicator:hover {
    border-color: #FFFFFF;
}
QCheckBox::indicator:checked {
    background: #F1F1F1;
    border-color: #FFFFFF;
}

QPushButton {
    background: #20242A;
    color: #F1F1F1;
    border: 2px solid #454C58;
    border-radius: 0;
    padding: 9px 13px;
    min-height: 20px;
    font-weight: 900;
}
QPushButton:hover {
    background: #2E343D;
    border-color: #8E97A4;
}
QPushButton:pressed { background: #48505C; }
QPushButton:disabled {
    background: #16181B;
    color: #5E6570;
    border-color: #2A2E35;
}
QPushButton#Primary {
    background: #F1F1F1;
    border: 2px solid #FFFFFF;
    color: #0D0E10;
}
QPushButton#Primary:hover { background: #D8DCE2; }
QPushButton#Primary:pressed { background: #B8BEC6; }
QPushButton#Disconnect {
    background: #2D1215;
    border: 2px solid #EF4444;
    color: #FCA5A5;
}
QPushButton#Disconnect:hover { background: #451A1E; }
QPushButton#Quick {
    padding: 6px 10px;
    min-height: 18px;
    background: #1E2228;
    border: 1px solid #4B5262;
}
QPushButton#Quick:hover {
    background: #2D333C;
    border-color: #D8D8D8;
}
QPushButton#Nudge {
    padding: 6px 8px;
    min-height: 18px;
    background: #1A1D21;
    border: 1px solid #4B5262;
    font-weight: 900;
}
QPushButton#Nudge:hover {
    background: #2D323B;
    border-color: #D8D8D8;
}

QPushButton#HoldReverse, QPushButton#HoldForward {
    min-height: 42px;
    font-size: 13px;
}
QPushButton#HoldReverse {
    background: #1B1E23;
    border: 2px solid #6B7280;
}
QPushButton#HoldForward {
    background: #262A31;
    border: 2px solid #9CA3AF;
}
QPushButton#HoldReverse:hover, QPushButton#HoldForward:hover {
    border-color: #F1F1F1;
}
QPushButton#HoldReverse:pressed, QPushButton#HoldForward:pressed {
    background: #F1F1F1;
    color: #111111;
    border-color: #FFFFFF;
}
QPushButton#StopButton {
    background: #2A1618;
    border: 2px solid #EF4444;
    color: #FCA5A5;
    min-height: 42px;
    font-size: 13px;
    font-weight: 900;
}
QPushButton#StopButton:hover {
    background: #451A1E;
    border-color: #F87171;
}
QPushButton#StopButton:pressed {
    background: #EF4444;
    color: #111111;
}

QSlider::groove:horizontal {
    background: #3A404A;
    height: 8px;
    border-radius: 0;
}
QSlider::sub-page:horizontal {
    background: #E5E5E5;
    height: 8px;
    border-radius: 0;
}
QSlider::handle:horizontal {
    background: #FFFFFF;
    border: 2px solid #0D0E10;
    width: 18px;
    margin: -7px 0;
    border-radius: 0;
}
QSlider::handle:horizontal:hover {
    background: #E5E5E5;
    border-color: #FFFFFF;
}

QToolTip {
    background: #F1F1F1;
    color: #0D0E10;
    border: 2px solid #FFFFFF;
    padding: 5px 8px;
    font-weight: 700;
}
"""
