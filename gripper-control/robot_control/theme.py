"""Dark graphite / cyan styling for the Qt desktop UI."""

STYLESHEET = """
QWidget {
    background: transparent;
    color: #E9F2FA;
    font-family: "SF Pro Display", "Inter", "Arial";
    font-size: 13px;
}
QWidget#AppBackground, QScrollArea#AppScroll, QWidget#Page {
    background-color: #0A111C;
}
QScrollArea#AppScroll { border: none; }
QFrame#Panel, QFrame#ServoCard, QFrame#FutureTile {
    background-color: #141F2E;
    border: 1px solid #28394A;
    border-radius: 18px;
}
QFrame#ServoCard[joystickAxis="X"] {
    border: 2px solid #58D7C6;
}
QFrame#ServoCard[joystickAxis="Y"] {
    border: 2px solid #E9B46B;
}
QFrame#FutureTile {
    background-color: #111B29;
    border-radius: 12px;
}
QLabel#Kicker {
    color: #58D7C6;
    font-size: 11px;
    font-weight: 800;
    letter-spacing: 2px;
}
QLabel#Hero {
    color: #F6F9FD;
    font-size: 30px;
    font-weight: 800;
}
QLabel#BodyMuted, QLabel#Description, QLabel#Tick {
    color: #92A3B6;
}
QLabel#SectionTitle {
    color: #F1F6FD;
    font-size: 16px;
    font-weight: 750;
}
QLabel#ServoName {
    color: #F5F9FE;
    font-size: 20px;
    font-weight: 750;
}
QLabel#Angle {
    color: #F7FBFF;
    font-size: 54px;
    font-weight: 850;
}
QLabel#SmallCaps {
    color: #6E859C;
    font-size: 11px;
    font-weight: 750;
    letter-spacing: 1px;
}
QLabel#Good { color: #59D9BE; font-weight: 700; }
QLabel#Waiting { color: #E9B46B; font-weight: 700; }
QLabel#Off { color: #9AABBC; font-weight: 700; }
QLabel#Bad { color: #F1848E; font-weight: 700; }
QComboBox, QSpinBox {
    background-color: #0E1724;
    border: 1px solid #34465A;
    border-radius: 9px;
    padding: 10px 12px;
    color: #F7FAFC;
    min-height: 19px;
    selection-background-color: #1B867D;
}
QComboBox:focus, QSpinBox:focus { border: 1px solid #58D7C6; }
QComboBox::drop-down { border: none; width: 26px; }
QComboBox QAbstractItemView {
    background: #1A2737;
    border: 1px solid #35465B;
    color: #FFFFFF;
    selection-background-color: #256A70;
}
QPushButton {
    background-color: #233448;
    border: 1px solid #3C4D63;
    border-radius: 9px;
    padding: 10px 17px;
    min-height: 19px;
    font-weight: 700;
    color: #EAF3FA;
}
QPushButton:hover { background-color: #30455B; }
QPushButton:pressed { background-color: #1A293B; }
QPushButton:disabled { background-color: #1A2531; color: #718396; border-color: #263443; }
QPushButton#Primary {
    background-color: #5BDAC9;
    color: #092022;
    border: 1px solid #5BDAC9;
}
QPushButton#Primary:hover { background-color: #83EADB; }
QPushButton#Disconnect {
    background-color: #322735;
    border-color: #5C3948;
    color: #F5B8BD;
}
QPushButton#Disconnect:hover { background-color: #4B2F3B; }
QPushButton#Quick { padding: 7px 12px; min-height: 16px; }
QPushButton#HoldReverse, QPushButton#HoldForward {
    min-height: 42px;
    font-size: 14px;
}
QPushButton#HoldReverse {
    background-color: #332936;
    border-color: #8A5363;
}
QPushButton#HoldReverse:pressed { background-color: #5A3545; }
QPushButton#HoldForward {
    background-color: #173B3C;
    border-color: #3E8E88;
}
QPushButton#HoldForward:pressed { background-color: #246461; }
QSlider::groove:horizontal {
    background-color: #304255;
    height: 9px;
    border-radius: 4px;
}
QSlider::sub-page:horizontal {
    background-color: #50CEBD;
    height: 9px;
    border-radius: 4px;
}
QSlider::handle:horizontal {
    background-color: #E9FFFA;
    border: 3px solid #49BDB0;
    width: 18px;
    margin: -8px 0;
    border-radius: 12px;
}
QSlider::handle:horizontal:hover { background-color: #FFFFFF; border-color: #7CE8DA; }
QToolTip { background: #233448; color: white; border: 1px solid #577181; }
"""
