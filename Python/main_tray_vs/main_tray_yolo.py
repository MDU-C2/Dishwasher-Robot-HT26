"""
main_tray_yolo.py - Runs the full YuMi vision & pick-and-place system with 2D YOLO tray detection enabled by default.
To switch interactively between YOLO and Normal 2D Thresholding, use `main_tray.py` or the YuMi Control GUI (`yumi_shortcut.py`).
"""
import main_tray

if __name__ == "__main__":
    main_tray.run(forced_tray_mode="yolo")
