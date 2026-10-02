import os
import json
<<<<<<< Updated upstream
=======
import math
>>>>>>> Stashed changes
import time
import cv2 as cv
import numpy as np

# ===========================================================
# CONFIGURATION
# ===========================================================
USE_CAM = True
<<<<<<< Updated upstream
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
=======
CAM_IDX = 0 

POS: dict[str, list[float]] = {}   # slot_name -> [X, Y, Z] robot coords
_cap = None                         # persistent VideoCapture
_ref_img = None                     # reference image (empty tray)
_slot_states: dict[str, str] = {}   # slot_name -> "free" | "occupied"  (in-memory)
_last_frame = None                  # last captured frame – reused for display (no re-grab)


def file_path(p: str) -> str:
    
    if os.path.exists(p):
        return p
    d = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(d, p)


def load_calibrated_positions(path: str = "slot_positions.json") -> None:
    
>>>>>>> Stashed changes
    global POS
    p = file_path(path)
    if not os.path.exists(p):
        print(f"[TRAY ERROR] Calibration file not found: {p}")
        return
    with open(p, "r") as f:
        POS = json.load(f)
<<<<<<< Updated upstream
    print(f"[TRAY] Loaded {len(POS)} slot coordinates.")

def load_cfg(p: str = "slot_config.json") -> dict:
    """Loads the pixel bounding boxes [x1, y1, x2, y2] for the 2D camera."""
=======
    print(f"[TRAY] Loaded {len(POS)} slot coordinates from '{p}'.")


def _sync_positions(cfg: dict, pos_path: str = "slot_positions.json") -> None:
    global POS
    p = file_path(pos_path)
    existing: dict = {}
    if os.path.exists(p):
        with open(p, "r") as f:
            existing = json.load(f)

    if existing:
        last_coords = list(existing.values())[-1]
        seed_x, seed_y, seed_z = last_coords[0], last_coords[1], last_coords[2]
    else:
        seed_x, seed_y, seed_z = 350.0, -20.0, 45.0   # safe defaults

    added = []
    for name in cfg.keys():
        if name not in existing:
            seed_y -= 100.0
            existing[name] = [seed_x, seed_y, seed_z]
            added.append(name)

    if added:
        with open(p, "w") as f:
            json.dump(existing, f, indent=4)
        print(f"[TRAY] Auto-added robot coords for: {added}  →  saved to '{p}'.")

    POS = existing
    print(f"[TRAY] Robot positions in memory: {len(POS)} slot(s).")


def load_cfg(p: str = "slot_config.json") -> dict:
    
>>>>>>> Stashed changes
    path = file_path(p)
    if not os.path.exists(path):
        print(f"[TRAY ERROR] Config file not found: {path}")
        return {}
    with open(path, "r") as f:
        return json.load(f)

<<<<<<< Updated upstream
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
=======

def get_live_frame():
    global _cap, _last_frame
    if not USE_CAM:
        img = cv.imread(file_path("test_images/current_state.jpg"))
        _last_frame = img
        return img

    # Re-open only when needed
    if _cap is None or not _cap.isOpened():
        # Try AVFoundation (Mac native) first, then any backend
        backends = [(CAM_IDX, cv.CAP_AVFOUNDATION),
                    (CAM_IDX, cv.CAP_ANY),
                    (1,       cv.CAP_AVFOUNDATION),
                    (1,       cv.CAP_ANY)]
        for idx, backend in backends:
            cap = cv.VideoCapture(idx, backend)
            if cap.isOpened():
                _cap = cap
                print(f"[TRAY] Camera opened: index={idx} backend={backend}")
                break
            cap.release()
        else:
            print("[TRAY ERROR] Could not open any camera.")
            return None

        _cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
        _cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        # Flush warmup frames – first reads are often dark/empty
        for _ in range(5):
            _cap.grab()
        time.sleep(0.3)

    ret, frame = _cap.read()
    if not ret or frame is None:
        print("[TRAY ERROR] Failed to grab frame from 2D camera.")
        return None
    _last_frame = frame
    return frame
    
def release_camera() -> None:
    
    global _cap
    if _cap is not None:
        _cap.release()
        _cap = None
        print("[TRAY] 2D camera released.")

#click 2 points
def calibrate_slots(image=None,
                    config_out: str = "slot_config.json",
                    ref_out: str = "test_images/empty_tray.jpg") -> dict:
    if image is None:
        print("[TRAY] Grabbing calibration frame from camera...")
        image = get_live_frame()
        if image is None:
            print("[TRAY ERROR] Cannot calibrate: no image available.")
            return {}

    # ---- state shared with the mouse callback ----
    state = {
        "clicks": [],   # raw (x,y) click points collected
        "boxes": [],    # completed [x1,y1,x2,y2] boxes
        "img": image.copy(),
    }

    COLOUR_PENDING  = (0, 200, 255)   # orange – first click placed
    COLOUR_DONE     = (0, 220, 60)    # green  – completed box
    COLOUR_LABEL    = (255, 255, 255) # white

    def _redraw(s):
        canvas = image.copy()
        # draw finished boxes
        for idx, box in enumerate(s["boxes"]):
            x1, y1, x2, y2 = box
            cv.rectangle(canvas, (x1, y1), (x2, y2), COLOUR_DONE, 2)
            label = f"slot_{idx + 1}"
            cv.putText(canvas, label, (x1, y1 - 6),
                       cv.FONT_HERSHEY_SIMPLEX, 0.55, COLOUR_LABEL, 1)
        # draw pending first click
        if len(s["clicks"]) == 1:
            cx, cy = s["clicks"][0]
            cv.circle(canvas, (cx, cy), 5, COLOUR_PENDING, -1)
            cv.putText(canvas, "click 2nd corner", (cx + 8, cy),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, COLOUR_PENDING, 1)
        s["img"] = canvas

    def _on_mouse(event, x, y, flags, param):
        s = param
        if event == cv.EVENT_LBUTTONDOWN:
            s["clicks"].append((x, y))
            if len(s["clicks"]) == 2:
                (x1, y1), (x2, y2) = s["clicks"]
                box = [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]
                s["boxes"].append(box)
                s["clicks"].clear()
                print(f"  [CALIB] slot_{len(s['boxes'])} -> {box}")
            _redraw(s)

    WIN = "Tray Calibration  |  click top-left then bottom-right per slot  |  u=undo  q=done"
    cv.namedWindow(WIN, cv.WINDOW_NORMAL)
    cv.setMouseCallback(WIN, _on_mouse, state)
    _redraw(state)

    print("[TRAY] Calibration window open.")
    print("       Click TOP-LEFT then BOTTOM-RIGHT for each slot.")
    print("       Press 'u' to undo last slot, 'q' when finished.")

    while True:
        cv.imshow(WIN, state["img"])
        key = cv.waitKey(20) & 0xFF
        if key == ord('q') or cv.getWindowProperty(WIN, cv.WND_PROP_VISIBLE) < 1:
            break
        if key == ord('u'):
            if state["boxes"]:
                removed = state["boxes"].pop()
                print(f"  [CALIB] Removed slot_{len(state['boxes']) + 1} -> {removed}")
                state["clicks"].clear()
                _redraw(state)

    cv.destroyWindow(WIN)

    if not state["boxes"]:
        print("[TRAY] No slots defined – calibration cancelled.")
        return {}

    # Build config dict
    cfg = {f"slot_{i + 1}": box for i, box in enumerate(state["boxes"])}

    # Save config
    cfg_path = file_path(config_out)
    os.makedirs(os.path.dirname(cfg_path) or ".", exist_ok=True)
    with open(cfg_path, "w") as f:
        json.dump(cfg, f, indent=4)
    print(f"[TRAY] Saved {len(cfg)} slots to '{cfg_path}'.")

    # Save reference image
    ref_path = file_path(ref_out)
    os.makedirs(os.path.dirname(ref_path), exist_ok=True)
    cv.imwrite(ref_path, image)
    print(f"[TRAY] Reference image saved to '{ref_path}'.")

    # Reset cached reference so get_free_slots reloads it
    global _ref_img
    _ref_img = image.copy()

    # Ensure slot_positions.json covers ALL calibrated slots
    _sync_positions(cfg)

    return cfg

#detect slots
def _scan_camera(thresh: float = 8) -> tuple[dict, object]:
    global _ref_img

    if _ref_img is None:
        rf_path = file_path("test_images/empty_tray.jpg")
        _ref_img = cv.imread(rf_path)
        if _ref_img is None:
            print(f"[TRAY ERROR] Reference image missing: {rf_path}")
            return {}, None

    current_frame = get_live_frame()
    if current_frame is None:
        return {}, None

    cfg = load_cfg("slot_config.json")
    results: dict[str, str] = {}
    display_frame = current_frame.copy()

    for name, box in cfg.items():
        try:
            x1, y1, x2, y2 = box
>>>>>>> Stashed changes
            roi_ref = _ref_img[y1:y2, x1:x2]
            roi_cur = current_frame[y1:y2, x1:x2]

            if roi_ref.shape != roi_cur.shape:
                results[name] = "unknown"
                continue

<<<<<<< Updated upstream
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
=======
            diff      = cv.absdiff(roi_ref, roi_cur)
            mean_diff = float(np.mean(diff))
            is_free   = mean_diff < thresh
            status    = "free" if is_free else "occupied"
            results[name] = status

            print(f"  [SCAN] {name}: mean_diff={mean_diff:.1f}  thresh={thresh}  → {status}")

            color = (0, 255, 0) if is_free else (0, 0, 255)
            cv.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
            cv.putText(display_frame, f"{name}: {status} ({mean_diff:.0f})", (x1, y1 - 10),
>>>>>>> Stashed changes
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        except Exception as e:
            print(f"[TRAY ERROR] Processing {name}: {e}")

    return results, display_frame
<<<<<<< Updated upstream

def get_robot_coords_for_slot(name: str) -> list[float]:
    """Returns [X, Y, Z] from slot_positions.json for the specified slot name."""
=======


def _annotate_from_state(states: dict) -> object:
    if _last_frame is None:
        return None
    frame = _last_frame.copy()
    cfg = load_cfg("slot_config.json")
    for name, box in cfg.items():
        try:
            x1, y1, x2, y2 = box
            status = states.get(name, "unknown")
            color  = (0, 255, 0) if status == "free" else (0, 0, 255)
            cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv.putText(frame, f"{name}: {status}", (x1, y1 - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        except Exception as e:
            print(f"[TRAY ERROR] Annotating {name}: {e}")
    return frame


def get_free_slots(rescan: bool = False, thresh: float = 20) -> tuple[dict, object]:
    global _slot_states

    if rescan or not _slot_states:
        states, frame = _scan_camera(thresh)
        _slot_states.update(states)
        return dict(_slot_states), frame

    # Fast path: just annotate current frame with known state
    frame = _annotate_from_state(_slot_states)
    return dict(_slot_states), frame

def get_closest_free_slot(robot_xy: list[float] | None = None) -> tuple[str | None, list[float] | None]:
    global _slot_states

    if not _slot_states:
        # First call: initialise from camera
        get_free_slots(rescan=True)

    free = {name: POS[name] for name, state in _slot_states.items()
            if state == "free" and name in POS}

    if not free:
        print("[TRAY] No free slots available.")
        return None, None

    if robot_xy is None or len(robot_xy) < 2:
        # Fall back to config order
        chosen = next(iter(free))
    else:
        rx, ry = robot_xy[0], robot_xy[1]
        chosen = min(free, key=lambda n: math.hypot(free[n][0] - rx, free[n][1] - ry))

    coords = list(POS[chosen])

  #mark occupied everytime we send coordinates to the robot
    _slot_states[chosen] = "occupied"
    print(f"[TRAY] Slot '{chosen}' selected @ {coords}  →  marked occupied.")

    return chosen, coords

def get_robot_coords_for_slot(name: str) -> list[float]:
    
>>>>>>> Stashed changes
    if name not in POS:
        raise KeyError(f"Slot '{name}' not found in calibrated positions.")
    return list(POS[name])

<<<<<<< Updated upstream
def verify_placement(name: str, thresh: float = 20) -> bool:
    """Returns True if the specified slot is now 'occupied'."""
    res, _ = get_free_slots(thresh)
    return res.get(name) == "occupied"
=======

def mark_slot_free(name: str) -> None:
    
    global _slot_states
    _slot_states[name] = "free"
    print(f"[TRAY] Slot '{name}' marked free.")


def verify_placement(name: str, thresh: float = 20) -> bool:
    states, _ = get_free_slots(rescan=True, thresh=thresh)
    return states.get(name) == "occupied"
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
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
=======
    import sys

    print("=" * 60)
    print("  TRAY CAMERA – standalone test")
    print("=" * 60)

    load_calibrated_positions()

    answer = input("\nRun interactive slot calibration? (y/n): ").strip().lower()
    if answer == "y":
        calibrate_slots()  
        print("\n[TEST] Calibration complete.  Reference image saved (empty tray).")
    else:
        print("[TEST] Skipping calibration – using existing slot_config.json.")
        
    print("\n[TEST] Taking one snapshot to detect slot states...")
    slots, img = get_free_slots(rescan=True)   # ONE camera grab, then done
    print(f"[TEST] Detected states: {slots}")

    if img is not None:
        WIN = "Slot Detection Result  |  any key to continue"
        cv.namedWindow(WIN, cv.WINDOW_NORMAL)
        cv.imshow(WIN, img)
        print("[TEST] Showing detected slots – press any key to continue.")
        cv.waitKey(0)
        cv.destroyWindow(WIN)

    # --- Step 3: loop until every free slot has been sent ---
    print("\n[TEST] Sending all free slots to robot – one by one ...\n")
    pick_count = 0
    robot_xy = [300.0, -120.0]   # starting reference position (ignored after first pick)

    while True:
        slot_name, coords = get_closest_free_slot(robot_xy)

        if slot_name is None:
            print("[TEST] No more free slots – tray is full.")
            break

        pick_count += 1
        print(f"[TEST] Pick #{pick_count}")
        print(f"[TEST]   → Chosen slot : {slot_name}")
        print(f"[TEST]   → Coords sent : {coords}")
        print(f"[TEST]   → States now  : {_slot_states}")

        # Use the chosen slot's XY as the next reference so the robot
        # always moves to the geometrically closest remaining slot.
        robot_xy = coords[:2]

        # Show annotated frame after each pick (no extra camera grab)
        states, annotated = get_free_slots(rescan=False)
        if annotated is not None:
            free_count = sum(1 for s in states.values() if s == "free")
            win_title = (
                f"After pick #{pick_count}: {slot_name}  |  "
                f"{free_count} slot(s) remaining  |  any key to continue"
            )
            cv.namedWindow(win_title, cv.WINDOW_NORMAL)
            cv.imshow(win_title, annotated)
            print(f"[TEST]   → {free_count} slot(s) remaining – press any key to continue.\n")
            cv.waitKey(0)
            cv.destroyWindow(win_title)

    cv.destroyAllWindows()
    print(f"\n[TEST] Done – {pick_count} slot(s) dispatched.")
>>>>>>> Stashed changes
