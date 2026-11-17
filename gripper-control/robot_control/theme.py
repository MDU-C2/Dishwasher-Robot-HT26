"""Dark, monochrome brutalist styling for the desktop control panel."""

STYLESHEET = """
QWidget {
    background: transparent;
    color: #F1F1F1;
    font-family: "Arial", "Helvetica Neue", sans-serif;
    font-size: 13px;
}
QMainWindow, QScrollArea#AppScroll, QWidget#Page {
    background: #101112;
}
QScrollArea#AppScroll { border: 0; }
QFrame#Panel, QFrame#ServoCard, QFrame#CalibrationCard {
    background: #191A1B;
    border: 2px solid #D8D8D8;
    border-radius: 0;
}
QFrame#ServoCard { background: #1E1F20; }
QFrame#CalibrationCard { background: #191A1B; }
QLabel#Kicker, QLabel#SmallCaps {
    color: #F1F1F1;
    font-size: 11px;
    font-weight: 900;
    letter-spacing: 1.4px;
}
QLabel#Hero {
    color: #FFFFFF;
    font-size: 38px;
    font-weight: 1000;
    letter-spacing: -1px;
}
QLabel#BodyMuted, QLabel#Description, QLabel#Tick {
    color: #B5B5B5;
}
QLabel#SectionTitle, QLabel#ServoName {
    color: #FFFFFF;
    font-size: 18px;
    font-weight: 900;
}
QLabel#ServoName { font-size: 20px; }
QLabel#Angle {
    color: #FFFFFF;
    font-size: 48px;
    font-weight: 1000;
}
QLabel#ChannelTag {
    background: #F1F1F1;
    color: #111111;
    border: 2px solid #F1F1F1;
    padding: 5px 8px;
    font-weight: 900;
}
QLabel#Good, QLabel#Waiting, QLabel#Off, QLabel#Bad {
    color: #E2E2E2;
    font-weight: 900;
}
QTabWidget#MainTabs::pane {
    background: #101112;
    border: 2px solid #D8D8D8;
    top: -2px;
}
QTabBar::tab {
    background: #242526;
    color: #E8E8E8;
    border: 2px solid #D8D8D8;
    border-bottom: 0;
    padding: 12px 18px;
    margin-right: 5px;
    font-weight: 900;
}
QTabBar::tab:selected {
    background: #F1F1F1;
    color: #111111;
}
QTabBar::tab:!selected { margin-top: 5px; }
QComboBox, QSpinBox {
    background: #101112;
    color: #F1F1F1;
    border: 2px solid #BEBEBE;
    border-radius: 0;
    padding: 8px 10px;
    min-height: 20px;
    selection-background-color: #E8E8E8;
    selection-color: #111111;
}
QComboBox:focus, QSpinBox:focus { border-color: #FFFFFF; }
QComboBox::drop-down { border: 0; width: 25px; }
QComboBox QAbstractItemView {
    background: #191A1B;
    color: #F1F1F1;
    border: 2px solid #D8D8D8;
    selection-background-color: #E8E8E8;
    selection-color: #111111;
}
QPushButton {
    background: #242526;
    color: #F1F1F1;
    border: 2px solid #D8D8D8;
    border-radius: 0;
    padding: 9px 13px;
    min-height: 20px;
    font-weight: 900;
}
QPushButton:hover { background: #3A3B3C; }
QPushButton:pressed { background: #555657; }
QPushButton:disabled {
    background: #222324;
    color: #777777;
    border-color: #666666;
}
QPushButton#Primary {
    background: #F1F1F1;
    border: 2px solid #FFFFFF;
    color: #111111;
}
QPushButton#Primary:hover { background: #D0D0D0; }
QPushButton#Disconnect {
    background: #343536;
    border-color: #FFFFFF;
    color: #FFFFFF;
}
QPushButton#Quick { padding: 7px 9px; min-height: 16px; }
QPushButton#HoldReverse, QPushButton#HoldForward {
    min-height: 42px;
    font-size: 14px;
}
QPushButton#HoldReverse { background: #202122; }
QPushButton#HoldForward { background: #3B3C3D; }
QSlider::groove:horizontal {
    background: #666666;
    height: 7px;
    border-radius: 0;
}
QSlider::sub-page:horizontal {
    background: #E5E5E5;
    height: 7px;
    border-radius: 0;
}
QSlider::handle:horizontal {
    background: #FFFFFF;
    border: 2px solid #111111;
    width: 18px;
    margin: -8px 0;
    border-radius: 0;
}
QToolTip {
    background: #F1F1F1;
    color: #111111;
    border: 2px solid #FFFFFF;
    padding: 5px;
}
"""
