"""
tray_camera.py - 2D tray camera: which dishwasher slots are free, and where the robot should place a mug.

API used by main_tray.py / main_trayOriginal.py:
    load_calibrated_positions("slot_positions.json")   robot X,Y,Z for every slot name
    get_free_slots()            -> (states, annotated_frame)   live scan; states = {"slot_1": "free" | "occupied" | "unknown"}
    get_robot_coords_for_slot(name) -> [x, y, z]
    verify_placement(name)      -> True when that slot is now occupied (live scan)
    release_camera()            close the 2D camera (close() is an alias)

Extras: calibrate_slots() (click the slot corners, saves slot_config.json + the empty-tray reference image),
get_closest_free_slot(), mark_slot_free(). Run `python tray_camera.py` to calibrate or to watch the live view.

Files used (looked up next to this script if not in the current folder):
    slot_config.json            pixel boxes [x1, y1, x2, y2] per slot, written by calibrate_slots()
    test_images/empty_tray.jpg  reference photo of the EMPTY tray, written by calibrate_slots()
    slot_positions.json         robot coordinates per slot
"""
import json
import math
import os
import sys
import threading
import time

import cv2 as cv
import numpy as np

# ===========================================================
# CONFIGURATION
# ===========================================================
USE_CAM = True              # False = read test_images/current_state.jpg instead of a camera
CAM_IDX = 0                 # change to 1 if 0 is a laptop webcam and 1 is the USB tray camera
FRAME_W, FRAME_H = 1280, 720
FREE_THRESH = 50            # slot counts as FREE when its mean pixel difference to the empty tray is below this
VERIFY_THRESH = 20          # slot counts as OCCUPIED (placement verified) at or above this difference
OPEN_RETRY_SECONDS = 5      # after a failed camera open, wait this long before trying again
VERBOSE_SCAN = False        # True = print every slot's difference on every scan (very chatty)

POS: dict[str, list[float]] = {}     # slot_name -> [X, Y, Z] robot coords
_cap = None                          # persistent VideoCapture
_ref_img = None                      # reference image (empty tray)
_slot_states: dict[str, str] = {}    # last scan result
_last_frame = None                   # last captured frame
_next_open_try = 0.0
_cfg_cache: dict = {}                # path -> (mtime, cfg)
_last_warn: dict[str, float] = {}
_lock = threading.RLock()            # the display loop and the pick thread both call in here


def _warn(key: str, msg: str, every: float = 30.0) -> None:
    """Print a message at most once per `every` seconds (the display loop calls us many times per second)."""
    now = time.time()
    if now - _last_warn.get(key, 0.0) >= every:
        _last_warn[key] = now
        print(msg)


def file_path(p: str) -> str:
    """Find a file in the current folder, otherwise next to this script."""
    if os.path.exists(p):
        return p
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), p)


# ===========================================================
# FILES
# ===========================================================
def load_calibrated_positions(path: str = "slot_positions.json") -> None:
    """Loads the robot X,Y,Z coordinates for each slot name."""
    global POS
    p = file_path(path)
    if not os.path.exists(p):
        print(f"[TRAY ERROR] Calibration file not found: {p}")
        return
    with open(p, "r") as f:
        POS = json.load(f)
    print(f"[TRAY] Loaded {len(POS)} slot coordinates from '{p}'.")


def _sync_positions(cfg: dict, pos_path: str = "slot_positions.json") -> None:
    """After calibration make sure slot_positions.json has an entry for every calibrated slot."""
    global POS
    p = file_path(pos_path)
    existing: dict = {}
    if os.path.exists(p):
        with open(p, "r") as f:
            existing = json.load(f)

    if existing:
        last = list(existing.values())[-1]
        seed_x, seed_y, seed_z = last[0], last[1], last[2]
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
        print(f"[TRAY] Auto-added robot coords for: {added}  ->  saved to '{p}'. CHECK THEM before running the robot.")

    POS = existing
    print(f"[TRAY] Robot positions in memory: {len(POS)} slot(s).")


def load_cfg(p: str = "slot_config.json") -> dict:
    """Pixel bounding boxes [x1, y1, x2, y2] per slot. Cached; re-read only when the file changes."""
    path = file_path(p)
    if not os.path.exists(path):
        _warn("cfg", f"[TRAY ERROR] Config file not found: {path}")
        return {}
    mtime = os.path.getmtime(path)
    hit = _cfg_cache.get(path)
    if hit and hit[0] == mtime:
        return hit[1]
    with open(path, "r") as f:
        cfg = json.load(f)
    _cfg_cache[path] = (mtime, cfg)
    return cfg


def _load_reference():
    global _ref_img
    if _ref_img is None:
        rf_path = file_path("test_images/empty_tray.jpg")
        _ref_img = cv.imread(rf_path)
        if _ref_img is None:
            _warn("ref", f"[TRAY ERROR] Reference image missing: {rf_path}")
    return _ref_img


# ===========================================================
# CAMERA
# ===========================================================
def _backends() -> list:
    if sys.platform == "darwin":
        return [cv.CAP_AVFOUNDATION, cv.CAP_ANY]
    if sys.platform.startswith("win"):
        return [cv.CAP_DSHOW, cv.CAP_ANY]
    return [cv.CAP_ANY]


def _open_camera():
    for backend in _backends():
        cap = cv.VideoCapture(CAM_IDX, backend)
        if cap.isOpened():
            cap.set(cv.CAP_PROP_FRAME_WIDTH, FRAME_W)
            cap.set(cv.CAP_PROP_FRAME_HEIGHT, FRAME_H)
            for _ in range(5):          # first frames are often dark / empty
                cap.grab()
            time.sleep(0.3)
            print(f"[TRAY] 2D camera opened: index={CAM_IDX} backend={backend}")
            return cap
        cap.release()
    _warn("open", f"[TRAY ERROR] Could not open 2D camera (index {CAM_IDX}). Retrying every {OPEN_RETRY_SECONDS:.0f} s.", 10.0)
    return None


def get_live_frame():
    """Newest frame from the 2D camera (opened on first use). None if there is no camera."""
    global _cap, _last_frame, _next_open_try
    with _lock:
        if not USE_CAM:
            img = cv.imread(file_path("test_images/current_state.jpg"))
            _last_frame = img
            return img

        if _cap is None or not _cap.isOpened():
            if time.time() < _next_open_try:      # don't stall the vision loop retrying every frame
                return None
            _cap = _open_camera()
            if _cap is None:
                _next_open_try = time.time() + OPEN_RETRY_SECONDS
                return None

        ret, frame = _cap.read()
        if not ret or frame is None:
            _warn("grab", "[TRAY ERROR] Failed to grab frame from 2D camera.", 5.0)
            return None
        _last_frame = frame
        return frame


def release_camera() -> None:
    """Closes the 2D camera."""
    global _cap
    with _lock:
        if _cap is not None:
            _cap.release()
            _cap = None
            print("[TRAY] 2D camera released.")


close = release_camera   # old name


# ===========================================================
# CALIBRATION (click the slot corners)
# ===========================================================
def calibrate_slots(image=None, config_out: str = "slot_config.json", ref_out: str = "test_images/empty_tray.jpg") -> dict:
    """Click TOP-LEFT then BOTTOM-RIGHT of every slot on an EMPTY tray. u = undo, q = done."""
    global _ref_img
    if image is None:
        print("[TRAY] Grabbing calibration frame from camera...")
        image = get_live_frame()
        if image is None:
            print("[TRAY ERROR] Cannot calibrate: no image available.")
            return {}

    state = {"clicks": [], "boxes": [], "img": image.copy()}
    COLOUR_PENDING, COLOUR_DONE, COLOUR_LABEL = (0, 200, 255), (0, 220, 60), (255, 255, 255)

    def _redraw(s):
        canvas = image.copy()
        for idx, box in enumerate(s["boxes"]):
            x1, y1, x2, y2 = box
            cv.rectangle(canvas, (x1, y1), (x2, y2), COLOUR_DONE, 2)
            cv.putText(canvas, f"slot_{idx + 1}", (x1, y1 - 6), cv.FONT_HERSHEY_SIMPLEX, 0.55, COLOUR_LABEL, 1)
        if len(s["clicks"]) == 1:
            cx, cy = s["clicks"][0]
            cv.circle(canvas, (cx, cy), 5, COLOUR_PENDING, -1)
            cv.putText(canvas, "click 2nd corner", (cx + 8, cy), cv.FONT_HERSHEY_SIMPLEX, 0.5, COLOUR_PENDING, 1)
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

    win = "Tray Calibration  |  click top-left then bottom-right per slot  |  u=undo  q=done"
    cv.namedWindow(win, cv.WINDOW_NORMAL)
    cv.setMouseCallback(win, _on_mouse, state)
    _redraw(state)
    print("[TRAY] Calibration window open. Click TOP-LEFT then BOTTOM-RIGHT for each slot; 'u' undo, 'q' when finished.")

    while True:
        cv.imshow(win, state["img"])
        key = cv.waitKey(20) & 0xFF
        if key == ord('q') or cv.getWindowProperty(win, cv.WND_PROP_VISIBLE) < 1:
            break
        if key == ord('u') and state["boxes"]:
            removed = state["boxes"].pop()
            print(f"  [CALIB] Removed slot_{len(state['boxes']) + 1} -> {removed}")
            state["clicks"].clear()
            _redraw(state)
    cv.destroyWindow(win)

    if not state["boxes"]:
        print("[TRAY] No slots defined - calibration cancelled.")
        return {}

    cfg = {f"slot_{i + 1}": box for i, box in enumerate(state["boxes"])}
    cfg_path = file_path(config_out)
    os.makedirs(os.path.dirname(cfg_path) or ".", exist_ok=True)
    with open(cfg_path, "w") as f:
        json.dump(cfg, f, indent=4)
    print(f"[TRAY] Saved {len(cfg)} slots to '{cfg_path}'.")

    ref_path = file_path(ref_out)
    os.makedirs(os.path.dirname(ref_path) or ".", exist_ok=True)
    cv.imwrite(ref_path, image)
    print(f"[TRAY] Reference image saved to '{ref_path}'.")

    with _lock:
        _ref_img = image.copy()          # use the new empty-tray photo right away
        _cfg_cache.clear()
        _slot_states.clear()
    _sync_positions(cfg)
    return cfg


# ===========================================================
# SLOT DETECTION
# ===========================================================
def _scan_camera(thresh: float) -> tuple[dict, object]:
    """Grab a frame and compare every slot box with the empty-tray reference."""
    ref = _load_reference()
    if ref is None:
        return {}, None
    frame = get_live_frame()
    if frame is None:
        return {}, None

    cfg = load_cfg("slot_config.json")
    display = frame.copy()
    results: dict[str, str] = {}

    if ref.shape != frame.shape:
        _warn("size", f"[TRAY ERROR] Reference image is {ref.shape[1]}x{ref.shape[0]} but the camera gives "
                      f"{frame.shape[1]}x{frame.shape[0]}. Run calibration again.")
        return {name: "unknown" for name in cfg}, display

    for name, box in cfg.items():
        try:
            x1, y1, x2, y2 = box
            fh, fw = frame.shape[:2]
            if not (0 <= x1 < x2 <= fw and 0 <= y1 < y2 <= fh):     # box partly outside the picture: never trust it
                results[name] = "unknown"
                continue
            roi_ref = ref[y1:y2, x1:x2]
            roi_cur = frame[y1:y2, x1:x2]

            mean_diff = float(np.mean(cv.absdiff(roi_ref, roi_cur)))
            is_free = mean_diff < thresh
            status = "free" if is_free else "occupied"
            results[name] = status
            if VERBOSE_SCAN:
                print(f"  [SCAN] {name}: mean_diff={mean_diff:.1f}  thresh={thresh}  -> {status}")

            color = (0, 255, 0) if is_free else (0, 0, 255)       # green = free, red = occupied
            cv.rectangle(display, (x1, y1), (x2, y2), color, 2)
            cv.putText(display, f"{name}: {status} ({mean_diff:.0f})", (x1, y1 - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        except Exception as e:
            print(f"[TRAY ERROR] Processing {name}: {e}")
    return results, display


def _annotate_from_state(states: dict):
    """Draw the known slot states on the last frame without grabbing a new one."""
    if _last_frame is None:
        return None
    frame = _last_frame.copy()
    for name, box in load_cfg("slot_config.json").items():
        try:
            x1, y1, x2, y2 = box
            status = states.get(name, "unknown")
            color = (0, 255, 0) if status == "free" else (0, 0, 255)
            cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv.putText(frame, f"{name}: {status}", (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        except Exception as e:
            print(f"[TRAY ERROR] Annotating {name}: {e}")
    return frame


def get_free_slots(thresh: float | None = None, rescan: bool = True) -> tuple[dict, object]:
    """
    Returns (states, annotated_frame). By default every call looks at the camera again (live view), which is what
    main_tray.py expects. rescan=False reuses the last scan and only redraws it.
    """
    with _lock:
        if rescan or not _slot_states:
            states, frame = _scan_camera(FREE_THRESH if thresh is None else thresh)
            if states:
                _slot_states.clear()
                _slot_states.update(states)
            return dict(states), frame
        return dict(_slot_states), _annotate_from_state(_slot_states)


def get_robot_coords_for_slot(name: str) -> list[float]:
    """[X, Y, Z] from slot_positions.json for the given slot."""
    if name not in POS:
        raise KeyError(f"Slot '{name}' not found in calibrated positions.")
    return list(POS[name])


def verify_placement(name: str, thresh: float | None = None) -> bool:
    """True if the slot now looks occupied (fresh camera scan)."""
    states, _ = get_free_slots(VERIFY_THRESH if thresh is None else thresh)
    return states.get(name) == "occupied"


def get_closest_free_slot(robot_xy: list[float] | None = None) -> tuple[str | None, list[float] | None]:
    """Free slot nearest to robot_xy (first free one if None). Marks it occupied in memory."""
    with _lock:
        if not _slot_states:
            get_free_slots(rescan=True)
        free = {n: POS[n] for n, s in _slot_states.items() if s == "free" and n in POS}
        if not free:
            print("[TRAY] No free slots available.")
            return None, None
        if robot_xy is None or len(robot_xy) < 2:
            chosen = next(iter(free))
        else:
            rx, ry = robot_xy[0], robot_xy[1]
            chosen = min(free, key=lambda n: math.hypot(free[n][0] - rx, free[n][1] - ry))
        coords = list(POS[chosen])
        _slot_states[chosen] = "occupied"
        print(f"[TRAY] Slot '{chosen}' selected @ {coords}  ->  marked occupied.")
        return chosen, coords


def mark_slot_free(name: str) -> None:
    with _lock:
        _slot_states[name] = "free"
    print(f"[TRAY] Slot '{name}' marked free.")


# ===========================================================
# STANDALONE: calibrate / live view
# ===========================================================
if __name__ == "__main__":
    print("=" * 60)
    print("  TRAY CAMERA - standalone")
    print("=" * 60)
    load_calibrated_positions()
    try:
        if input("\nRun interactive slot calibration (tray must be EMPTY)? (y/n): ").strip().lower() == "y":
            calibrate_slots()
        print("\nLive view: green = free, red = occupied (number = difference). Press q to quit.")
        while True:
            _states, img = get_free_slots()
            if img is not None:
                cv.imshow("Tray Test (press q to quit)", img)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        release_camera()
        cv.destroyAllWindows()
