import os
import sys
import time
import json
import cv2 as cv
import numpy as np

import tray_camera_yolo as tc
from aruco_tray_tracker import TrayArucoTracker, ARUCO_DICT_TYPE, TARGET_MARKER_ID

def generate_marker(marker_id=TARGET_MARKER_ID, size=400):
    dict_aruco = cv.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    marker = cv.aruco.generateImageMarker(dict_aruco, marker_id, size)
    # Add quiet zone border
    canvas = np.full((size + 100, size + 100), 255, dtype=np.uint8)
    canvas[50:size + 50, 50:size + 50] = marker
    out_path = os.path.join(os.path.dirname(__file__), f"aruco_marker_id{marker_id}.png")
    cv.imwrite(out_path, canvas)
    print(f"[ARUCO] Generated marker image at: {out_path}")
    return out_path

def calibrate_aruco_reference():
    print("\n" + "="*60)
    print("      DISHWASHER TRAY ARUCO REFERENCE CALIBRATION")
    print("="*60)
    print("1. Ensure the ArUco marker is firmly attached to the dishwasher tray.")
    print("2. Position the tray in its nominal starting / home position.")
    print("3. Press 'c' to capture reference when marker is detected.")
    print("4. Press 'q' to cancel without saving.\n")

    generate_marker(TARGET_MARKER_ID)
    tracker = TrayArucoTracker(TARGET_MARKER_ID)

    if not tc.start():
        print("[ERROR] Cannot access overhead tray camera.")
        return False

    win = "ArUco Tray Reference Calibration"
    cv.namedWindow(win, cv.WINDOW_NORMAL)

    captured = False
    while True:
        frame = tc._grab(wait=0.03)
        if frame is None:
            time.sleep(0.05)
            continue

        disp = frame.copy()
        found, corners = tracker.detect(disp)
        tracker.draw(disp)

        if found:
            cv.putText(disp, "Marker detected! Press 'c' to save reference", (20, 40),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv.putText(disp, "Searching for ArUco marker...", (20, 40),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv.imshow(win, disp)
        key = cv.waitKey(1) & 0xFF
        if key == ord('c') and found and corners is not None:
            tracker.save_reference(corners, marker_size_mm=25.0)
            # Synchronize the main tray camera tracker instance with the new reference
            tc.get_tracker().load_reference()
            captured = True
            print("[SUCCESS] Reference ArUco pose successfully captured and saved!")
            break
        elif key == ord('q'):
            print("[CANCELLED] ArUco calibration cancelled by user.")
            break

    cv.destroyAllWindows()
    return captured

if __name__ == "__main__":
    calibrate_aruco_reference()

