import os
import json
import time
import cv2 as cv
import numpy as np

# ===========================================================
# CONFIGURATION
# ===========================================================
USE_CAM = True
CAM_IDX = 0  # Change to 1 if 0 is your laptop webcam and 1 is the USB tray cam

# Global variables to maintain state
POS: dict[str, list[float]] = {}
_cap = None
_ref_img = None

def file_path(p: str) -> str:
    """Helper to find files relative to the script location."""
    if os.path.exists(p):
        return p
    d = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(d, p)
    return path

def load_calibrated_positions(path: str = "slot_positions.json") -> None:
    """Loads the Robot X,Y,Z coordinates for each slot name."""
    global POS
    p = file_path(path)
    if not os.path.exists(p):
        print(f"[TRAY ERROR] Calibration file not found: {p}")
        return
    with open(p, "r") as f:
        POS = json.load(f)
    print(f"[TRAY] Loaded {len(POS)} slot coordinates.")

def load_cfg(p: str = "slot_config.json") -> dict:
    """Loads the pixel bounding boxes [x1, y1, x2, y2] for the 2D camera."""
    path = file_path(p)
    if not os.path.exists(path):
        print(f"[TRAY ERROR] Config file not found: {path}")
        return {}
    with open(path, "r") as f:
        return json.load(f)

def get_live_frame():
    """Maintains a persistent camera connection for smooth video."""
    global _cap
    if not USE_CAM:
        # Fallback to a static image if camera is disabled
        img = cv.imread(file_path("test_images/current_state.jpg"))
        return img

    if _cap is None or not _cap.isOpened():
        print(f"[TRAY] Opening 2D Camera at index {CAM_IDX}...")
        _cap = cv.VideoCapture(CAM_IDX)
        # Set resolution for better detection
        _cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
        _cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        time.sleep(1.0) # Warmup

    ret, frame = _cap.read()
    if not ret:
        print("[TRAY ERROR] Failed to grab frame from 2D camera.")
        return None
    return frame

def get_free_slots(thresh: float = 50):
    """
    Analyzes the tray and returns slot status and an annotated image.
    Used by main.py to update the UI window.
    """
    global _ref_img
    
    # 1. Load Reference Image (Only once)
    if _ref_img is None:
        rf_path = file_path("test_images/empty_tray.jpg")
        _ref_img = cv.imread(rf_path)
        if _ref_img is None:
            print(f"[TRAY ERROR] Reference image missing: {rf_path}")
            return {}, None

    # 2. Get Current Frame
    current_frame = get_live_frame()
    if current_frame is None:
        return {}, None

    # 3. Load Slot Pixel Configuration
    cfg = load_cfg("slot_config.json")
    results = {}
    
    # Overlay frame - start with a copy of current frame
    display_frame = current_frame.copy()

    for name, box in cfg.items():
        try:
            x1, y1, x2, y2 = box
            
            # Extract Region of Interest (ROI)
            roi_ref = _ref_img[y1:y2, x1:x2]
            roi_cur = current_frame[y1:y2, x1:x2]

            if roi_ref.shape != roi_cur.shape:
                results[name] = "unknown"
                continue

            # Compare current pixels to empty tray pixels
            diff = cv.absdiff(roi_ref, roi_cur)
            mean_diff = float(np.mean(diff))

            # Determine Status
            is_free = mean_diff < thresh
            status = "free" if is_free else "occupied"
            results[name] = status

            # --- VISUAL FEEDBACK ---
            # Green for Free, Red for Occupied
            color = (0, 255, 0) if is_free else (0, 0, 255)
            cv.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
            
            # Label
            label_text = f"{name}: {status}"
            cv.putText(display_frame, label_text, (x1, y1 - 10), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        except Exception as e:
            print(f"[TRAY ERROR] Processing {name}: {e}")

    return results, display_frame

def get_robot_coords_for_slot(name: str) -> list[float]:
    """Returns [X, Y, Z] from slot_positions.json for the specified slot name."""
    if name not in POS:
        raise KeyError(f"Slot '{name}' not found in calibrated positions.")
    return list(POS[name])

def verify_placement(name: str, thresh: float = 20) -> bool:
    """Returns True if the specified slot is now 'occupied'."""
    res, _ = get_free_slots(thresh)
    return res.get(name) == "occupied"

def release_camera():
    """Closes the 2D camera resource."""
    global _cap
    if _cap is not None:
        _cap.release()
        _cap = None
        print("[TRAY] 2D Camera released.")

# ===========================================================
# TEST SCRIPT
# ===========================================================
if __name__ == "__main__":
    # Test block to run this file standalone
    try:
        load_calibrated_positions()
        while True:
            slots, img = get_free_slots(thresh=25)
            if img is not None:
                cv.imshow("Tray Test (Press Q to quit)", img)
            
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        release_camera()
        cv.destroyAllWindows()