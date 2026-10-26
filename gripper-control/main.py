"""Run: python main.py"""

import sys

from PySide6.QtWidgets import QApplication

from robot_control.theme import STYLESHEET
from robot_control.window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLESHEET)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
