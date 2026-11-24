# Hallmark · shape: app screen · genre: modern-minimal · theme: monochrome (inverted ink) · waive 22 · near-zero chroma surfaces & controls · v2.0.0

TOKENS = {
    "paper": "#0A0B0D",
    "paper_2": "#111316",
    "paper_3": "#07080A",
    "ink": "#F2F4F7",
    "ink_2": "#C6CBD2",
    "muted": "#8A9099",
    "rule": "#22252A",
    "rule_2": "#383C44",
    "accent": "#ECEFF3",
    "accent_hover": "#FFFFFF",
    "accent_ink": "#0A0B0D",
    "accent_soft": "#1F2329",
    "cool": "#D8DCE2",
    "cool_soft": "#1C1F25",
    "ok": "#A3B8A8",
    "ok_soft": "#141A16",
    "bad": "#C87A76",
    "bad_soft": "#1D1313",
    "font_sans": '"Helvetica Neue", Arial',
    "font_mono": '"SF Mono", Menlo, Monaco',
}

STYLESHEET = """
* {{
    font-family: {font_sans};
}}

QMainWindow, QScrollArea#AppScroll, QWidget#Page {{
    background: {paper};
    color: {ink};
}}

QScrollArea#AppScroll {{
    border: none;
}}

QScrollBar:vertical {{
    background: {paper};
    width: 8px;
    margin: 2px;
}}

QScrollBar::handle:vertical {{
    background: {rule};
    border-radius: 4px;
    min-height: 28px;
}}

QScrollBar::handle:vertical:hover {{
    background: {rule_2};
}}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
    background: none;
    height: 0px;
}}

QTabWidget#MainTabs::pane {{
    border: none;
    background: transparent;
    margin-top: 8px;
}}

QTabWidget#MainTabs::tab-bar {{
    alignment: left;
}}

QTabBar::tab {{
    background: transparent;
    color: {muted};
    border: 1px solid transparent;
    border-radius: 6px;
    padding: 7px 14px;
    margin-right: 4px;
    font-size: 13px;
    font-weight: 500;
}}

QTabBar::tab:hover {{
    color: {ink};
    background: {paper_2};
}}

QTabBar::tab:selected {{
    color: {ink};
    background: {paper_2};
    border: 1px solid {rule_2};
}}

QFrame#TopToolbar,
QFrame#Panel,
QFrame#GripMasterPanel,
QFrame#ServoCard,
QFrame#CalibrationCard {{
    background: {paper_2};
    border: 1px solid {rule};
    border-radius: 10px;
}}

/* Console surface ladder: nested wells use deepest surface paper_3 with NO second border */
QFrame#SubPanel,
QFrame#MotionPanel {{
    background: {paper_3};
    border: none;
    border-radius: 6px;
}}

/* Section separators use a quiet hairline instead of pill/stripe chrome */
QFrame#SectionBanner {{
    background: transparent;
    border: none;
    border-bottom: 1px solid {rule};
    border-radius: 0px;
}}

QLabel {{
    color: {ink};
    background: transparent;
}}

QLabel#Hero {{
    font-size: 16px;
    font-weight: 500;
    color: {ink};
}}

QLabel#SectionTitle {{
    font-size: 14px;
    font-weight: 500;
    color: {ink};
}}

QLabel#ServoName {{
    font-size: 14px;
    font-weight: 500;
    color: {ink};
}}

QLabel#Angle {{
    font-family: {font_mono};
    font-size: 15px;
    font-weight: 500;
    color: {cool};
}}

QLabel#Kicker {{
    font-size: 12px;
    font-weight: 500;
    color: {ink_2};
}}

QLabel#SmallCaps {{
    font-size: 11px;
    font-weight: 500;
    color: {muted};
}}

QLabel#BodyMuted,
QLabel#Description {{
    font-size: 12px;
    color: {muted};
}}

QLabel#Tick {{
    font-size: 11px;
    color: {muted};
}}

QLabel#ChannelTag {{
    font-size: 11px;
    font-weight: 500;
    color: {muted};
}}

QLabel#TypeBadgePos,
QLabel#TypeBadgeCont {{
    font-size: 11px;
    font-weight: 500;
    color: {muted};
}}

QLabel#Off,
QLabel#FsrState0, QLabel#OptState3, QLabel#LidarState3,
QLabel#SlipState0, QLabel#SlipState6 {{
    font-size: 11px;
    font-weight: 500;
    color: {muted};
    background: {paper_3};
    border: 1px solid {rule};
    border-radius: 4px;
    padding: 2px 8px;
}}

QLabel#Good,
QLabel#FsrState2, QLabel#OptState0, QLabel#LidarState1,
QLabel#SlipState1 {{
    font-size: 11px;
    font-weight: 500;
    color: {ok};
    background: {ok_soft};
    border: 1px solid {rule};
    border-radius: 4px;
    padding: 2px 8px;
}}

QLabel#FsrState1, QLabel#OptState2, QLabel#LidarState0, QLabel#LidarState2,
QLabel#SlipState2 {{
    font-size: 11px;
    font-weight: 500;
    color: {cool};
    background: {cool_soft};
    border: 1px solid {rule};
    border-radius: 4px;
    padding: 2px 8px;
}}

QLabel#Waiting,
QLabel#FsrState3, QLabel#OptState1 {{
    font-size: 11px;
    font-weight: 500;
    color: {accent};
    background: {accent_soft};
    border: 1px solid {rule};
    border-radius: 4px;
    padding: 2px 8px;
}}

QLabel#Bad,
QLabel#FsrState4,
QLabel#SlipState3, QLabel#SlipState4, QLabel#SlipState5 {{
    font-size: 11px;
    font-weight: 500;
    color: {bad};
    background: {bad_soft};
    border: 1px solid {rule};
    border-radius: 4px;
    padding: 2px 8px;
}}

QLabel#CtrlStateBanner {{
    font-size: 12px;
    font-weight: 500;
    border-radius: 6px;
    padding: 7px 12px;
    color: {ink_2};
    background: {paper_3};
    border: 1px solid {rule};
}}

QLabel#CtrlStateBanner[ctrlState="0"] {{
    color: {muted};
    background: {paper_3};
    border: 1px solid {rule};
}}

QLabel#CtrlStateBanner[ctrlState="1"],
QLabel#CtrlStateBanner[ctrlState="2"] {{
    color: {cool};
    background: {cool_soft};
    border: 1px solid {rule_2};
}}

QLabel#CtrlStateBanner[ctrlState="3"] {{
    color: {ok};
    background: {ok_soft};
    border: 1px solid {rule_2};
}}

QLabel#CtrlStateBanner[ctrlState="4"] {{
    color: {accent};
    background: {accent_soft};
    border: 1px solid {rule_2};
}}

QPushButton {{
    background: {paper_2};
    color: {ink};
    border: 1px solid {rule_2};
    border-radius: 6px;
    padding: 6px 12px;
    font-size: 12px;
    font-weight: 500;
    min-height: 18px;
}}

QPushButton:hover {{
    background: {paper};
    border-color: {muted};
}}

QPushButton:pressed {{
    background: {paper_3};
}}

QPushButton:disabled {{
    background: {paper_2};
    color: {rule_2};
    border-color: {rule};
}}

QPushButton#Primary {{
    background: {accent};
    color: {accent_ink};
    border: 1px solid {accent};
    font-weight: 600;
}}

QPushButton#Primary:hover {{
    background: {accent_hover};
    border-color: {accent_hover};
}}

QPushButton#Primary:pressed {{
    background: {accent};
}}

QPushButton#ConnectAction {{
    background: {paper_2};
    color: {ink};
    border: 1px solid {rule_2};
}}

QPushButton#ConnectAction:hover {{
    border-color: {accent};
    color: {accent};
}}

QPushButton#Disconnect {{
    background: {bad_soft};
    color: {bad};
    border: 1px solid {bad};
}}

QPushButton#Disconnect:hover {{
    background: {paper_3};
}}

QPushButton#Quick,
QPushButton#PoseButton {{
    background: {paper_2};
    color: {ink_2};
    border: 1px solid {rule};
    padding: 5px 10px;
}}

QPushButton#Quick:hover,
QPushButton#PoseButton:hover {{
    color: {ink};
    background: {paper};
    border-color: {rule_2};
}}

QPushButton#Nudge {{
    background: {paper_2};
    color: {ink_2};
    border: 1px solid {rule};
    border-radius: 5px;
    padding: 3px 8px;
    font-size: 11px;
    font-weight: 500;
    min-height: 16px;
}}

QPushButton#Nudge:hover {{
    color: {ink};
    border-color: {rule_2};
}}

QPushButton#HoldReverse,
QPushButton#HoldForward {{
    background: {paper_2};
    color: {ink};
    border: 1px solid {rule_2};
}}

QPushButton#HoldReverse:hover,
QPushButton#HoldForward:hover {{
    background: {cool_soft};
    color: {cool};
    border-color: {cool};
}}

QPushButton#StopButton {{
    background: {bad_soft};
    color: {bad};
    border: 1px solid {bad};
}}

QPushButton#StopButton:hover {{
    background: {paper_3};
    color: {ink};
}}

QPushButton#StopButton:pressed {{
    background: {bad_soft};
}}

QPushButton[hero="true"] {{
    font-size: 13px;
    font-weight: 600;
    padding: 9px 14px;
    min-height: 22px;
    border-radius: 6px;
}}

QPushButton#FilterTab {{
    background: transparent;
    color: {muted};
    border: 1px solid {rule};
    border-radius: 6px;
    padding: 5px 12px;
    font-size: 12px;
    font-weight: 500;
}}

QPushButton#FilterTab:hover {{
    color: {ink};
    border-color: {rule_2};
}}

QPushButton#FilterTab[active="true"] {{
    background: {cool_soft};
    color: {cool};
    border: 1px solid {rule_2};
}}

QComboBox,
QSpinBox,
QDoubleSpinBox,
QLineEdit {{
    background: {paper_3};
    color: {ink};
    border: 1px solid {rule};
    border-radius: 6px;
    padding: 4px 8px;
    font-size: 12px;
    min-height: 18px;
}}

QSpinBox,
QDoubleSpinBox {{
    font-family: {font_mono};
}}

QComboBox:focus,
QSpinBox:focus,
QDoubleSpinBox:focus,
QLineEdit:focus {{
    border: 1px solid {accent};
}}

QComboBox::drop-down {{
    border: none;
    width: 20px;
}}

QComboBox QAbstractItemView {{
    background: {paper_2};
    color: {ink};
    border: 1px solid {rule_2};
    selection-background-color: {cool_soft};
    selection-color: {cool};
}}

QCheckBox {{
    spacing: 8px;
    color: {ink_2};
    font-size: 12px;
}}

QCheckBox::indicator {{
    width: 15px;
    height: 15px;
    border-radius: 4px;
    border: 1px solid {rule_2};
    background: {paper_3};
}}

QCheckBox::indicator:hover {{
    border-color: {accent};
}}

QCheckBox::indicator:checked {{
    background: {accent};
    border: 1px solid {accent};
}}

QSlider::groove:horizontal {{
    height: 6px;
    background: {paper_3};
    border: 1px solid {rule};
    border-radius: 3px;
}}

QSlider::sub-page:horizontal {{
    height: 6px;
    background: {muted};
    border-radius: 3px;
}}

QSlider::handle:horizontal {{
    width: 14px;
    height: 14px;
    margin: -5px 0;
    border-radius: 7px;
    background: {ink};
    border: 2px solid {paper};
}}

QSlider::handle:horizontal:hover {{
    background: {accent};
    border-color: {accent_ink};
}}

QProgressBar {{
    background: {paper_3};
    border: 1px solid {rule};
    border-radius: 3px;
    text-align: center;
    color: transparent;
    min-height: 8px;
    max-height: 8px;
}}

QProgressBar::chunk {{
    border-radius: 2px;
    background: {cool};
}}

QProgressBar[fsrState="0"]::chunk,
QProgressBar[optState="3"]::chunk,
QProgressBar[lidarState="3"]::chunk,
QProgressBar[slipState="0"]::chunk,
QProgressBar[slipState="6"]::chunk {{
    background: {rule_2};
}}

QProgressBar[fsrState="1"]::chunk,
QProgressBar[optState="2"]::chunk,
QProgressBar[lidarState="0"]::chunk,
QProgressBar[lidarState="2"]::chunk,
QProgressBar[slipState="2"]::chunk {{
    background: {cool};
}}

QProgressBar[fsrState="2"]::chunk,
QProgressBar[optState="0"]::chunk,
QProgressBar[lidarState="1"]::chunk,
QProgressBar[slipState="1"]::chunk {{
    background: {ok};
}}

QProgressBar[fsrState="3"]::chunk,
QProgressBar[optState="1"]::chunk {{
    background: {accent};
}}

QProgressBar[fsrState="4"]::chunk,
QProgressBar[slipState="3"]::chunk,
QProgressBar[slipState="4"]::chunk,
QProgressBar[slipState="5"]::chunk {{
    background: {bad};
}}
""".format(**TOKENS)
