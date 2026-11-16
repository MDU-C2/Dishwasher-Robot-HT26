"""import os
import sys
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")  # must be set before importing cv2
import json
import math
import time
import threading
import cv2 as cv
import numpy as np

USE_CAM = True
CAM_IDX = 0                      # index of the 2D tray camera (run find_cameras.py if unsure)
CAM_FALLBACK_INDICES: list[int] = []   # other indices to try if CAM_IDX fails (empty = don't guess)

# ---- detection tuning -------------------------------------------------------
DIFF_PIXEL_THRESH = 25     # per-pixel gray-level difference that counts as "changed"
OCCUPIED_FRACTION = 0.12   # fraction of changed pixels in a slot ROI => occupied
BLUR_KSIZE = (7, 7)        # blur before comparing, suppresses sensor noise

# ==================================================================================
#  HARD-CODED ROBOT COORDINATES FOR EACH SLOT  (edit this list)
#
#  Jog the arm to where the cup should be RELEASED in each slot, read X, Y, Z from
#  the FlexPendant (same work object / tool the RAPID program uses, in mm) and
#  write them here.
#
#  "slot_N" must be the N-th box you click during 2D calibration. The camera only
#  decides whether each slot is free or occupied; these numbers are what get sent
#  to RAPID. Slots without an entry here are never chosen.
# ==================================================================================
SLOT_ROBOT_COORDS: dict[str, list[float]] = {
    # "slot_1": [X, Y, Z],
    # "slot_2": [X, Y, Z],
}
# If this dict is empty, the coordinates in slot_positions.json are used instead.

POS: dict[str, list[float]] = {name: list(c) for name, c in SLOT_ROBOT_COORDS.items()}
_cap = None                         # persistent VideoCapture
_ref_img = None                     # reference image (empty tray)
_slot_states: dict[str, str] = {}   # slot_name -> "free" | "occupied"  (in-memory)
_last_frame = None                  # last captured frame - reused for display (no re-grab)
_reserved: set[str] = set()         # slots the robot was sent to (occupied until mark_slot_free)
_camera_states: dict[str, str] = {} # what the camera alone saw on the last scan
_camera_lock = threading.RLock()    # main loop and robot thread both use the camera


def set_camera_index(idx: int) -> None:
    global CAM_IDX, _cap
    CAM_IDX = idx
    if _cap is not None:
        _cap.release()
        _cap = None
    print(f"[TRAY] Camera index set to {CAM_IDX}.")


def file_path(p: str) -> str:

    if os.path.exists(p):
        return p
    d = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(d, p)


def _check_positions(coords: dict) -> None:

    for name, c in coords.items():
        if not (isinstance(c, (list, tuple)) and len(c) == 3
                and all(isinstance(v, (int, float)) for v in c)):
            raise ValueError(f"Slot '{name}' must be [X, Y, Z] numbers, got {c!r}")

    cfg_path = file_path("slot_config.json")
    if os.path.exists(cfg_path):
        cam_slots = set(load_cfg("slot_config.json").keys())
        no_coords = sorted(cam_slots - set(coords))
        no_camera = sorted(set(coords) - cam_slots)
        if no_coords:
            print(f"[TRAY WARNING] Camera slots without robot coordinates (never used): {no_coords}")
        if no_camera:
            print(f"[TRAY WARNING] Robot coordinates without a camera slot (never used): {no_camera}")


def load_calibrated_positions(path: str = "slot_positions.json") -> None:
  
    global POS
    if SLOT_ROBOT_COORDS:
        POS = {name: list(c) for name, c in SLOT_ROBOT_COORDS.items()}
        source = "hard-coded SLOT_ROBOT_COORDS"
    else:
        p = file_path(path)
        if not os.path.exists(p):
            print(f"[TRAY ERROR] SLOT_ROBOT_COORDS is empty and '{p}' not found - no robot coordinates.")
            POS = {}
            return
        with open(p, "r") as f:
            POS = json.load(f)
        source = f"'{p}'"
    _check_positions(POS)
    print(f"[TRAY] Using {len(POS)} slot coordinate(s) from {source}: {sorted(POS)}")


def _sync_positions(cfg: dict, pos_path: str = "slot_positions.json") -> None:

    load_calibrated_positions(pos_path)


def load_cfg(p: str = "slot_config.json") -> dict:
    path = file_path(p)
    if not os.path.exists(path):
        print(f"[TRAY ERROR] Config file not found: {path}")
        return {}
    with open(path, "r") as f:
        return json.load(f)


def _backend_candidates() -> list:
    
    if sys.platform.startswith("win"):
        return [cv.CAP_DSHOW, cv.CAP_MSMF]
    if sys.platform == "darwin":
        return [cv.CAP_AVFOUNDATION, cv.CAP_ANY]
    return [cv.CAP_ANY]


def _open_camera():

    indices = [CAM_IDX] + [i for i in CAM_FALLBACK_INDICES if i != CAM_IDX]
    for idx in indices:
        for backend in _backend_candidates():
            t0 = time.time()
            print(f"[TRAY] Opening camera index={idx} backend={backend} ...")
            cap = cv.VideoCapture(idx, backend)
            if not cap.isOpened():
                cap.release()
                continue
            if sys.platform != "darwin":
                cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))  # less USB bandwidth
            cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv.CAP_PROP_BUFFERSIZE, 1)
            for _ in range(15):
                ret, frame = cap.read()
                if ret and frame is not None:
                    h, w = frame.shape[:2]
                    print(f"[TRAY] Camera opened: index={idx} backend={backend} "
                          f"{w}x{h} in {time.time() - t0:.1f}s")
                    return cap
                time.sleep(0.1)
            cap.release()
    return None


def get_live_frame():
    with _camera_lock:
        return _get_live_frame_unlocked()


def _get_live_frame_unlocked():
    global _cap, _last_frame
    if not USE_CAM:
        img = cv.imread(file_path("test_images/current_state.jpg"))
        _last_frame = img
        return img

    # Re-open only when needed
    if _cap is None or not _cap.isOpened():
        cap = _open_camera()
        if cap is None:
            print(f"[TRAY ERROR] Could not open the 2D camera at index {CAM_IDX}. "
                  f"Run find_cameras.py to find the right index and set CAM_IDX.")
            return None
        _cap = cap

        # Flush warmup frames
        for _ in range(3):
            _cap.grab()
        time.sleep(0.2)

    ret, frame = _cap.read()
    if not ret or frame is None:
        print("[TRAY ERROR] Failed to grab frame from 2D camera.")
        return None
    _last_frame = frame
    return frame


def grab_settled_frame(n_warmup: int = 30, n_avg: int = 5):
    with _camera_lock:
        return _grab_settled_frame_unlocked(n_warmup, n_avg)


def _grab_settled_frame_unlocked(n_warmup: int, n_avg: int):
   
    global _last_frame
    first = get_live_frame()          # makes sure the camera is open
    if first is None:
        return None
    if not USE_CAM:
        return first

    for _ in range(n_warmup):
        _cap.read()
        time.sleep(0.03)

    frames = []
    for _ in range(n_avg):
        ret, f = _cap.read()
        if ret and f is not None:
            frames.append(f.astype(np.float32))
    if not frames:
        return None

    avg = np.mean(frames, axis=0).astype(np.uint8)
    _last_frame = avg
    return avg


def release_camera() -> None:
    global _cap
    stop_background_scan()
    if _cap is not None:
        _cap.release()
        _cap = None
        print("[TRAY] 2D camera released.")


def reset_slot_states() -> None:

    _slot_states.clear()
    _reserved.clear()
    _camera_states.clear()
    print("[TRAY] Slot states reset.")


def _slot_metric(roi_ref, roi_cur) -> tuple[float, float]:
   
    ref = cv.GaussianBlur(cv.cvtColor(roi_ref, cv.COLOR_BGR2GRAY), BLUR_KSIZE, 0).astype(np.float32)
    cur = cv.GaussianBlur(cv.cvtColor(roi_cur, cv.COLOR_BGR2GRAY), BLUR_KSIZE, 0).astype(np.float32)
    diff = np.abs((cur - cur.mean()) - (ref - ref.mean()))
    return float(diff.mean()), float(np.mean(diff > DIFF_PIXEL_THRESH))


# click 2 points per slot
def calibrate_slots(image=None,
                    config_out: str = "slot_config.json",
                    ref_out: str = "test_images/empty_tray.jpg") -> dict:
    global _ref_img

    if image is None:
        print("[TRAY] Grabbing calibration frame from camera (letting exposure settle)...")
        image = grab_settled_frame()
        if image is None:
            print("[TRAY ERROR] Cannot calibrate: no image available.")
            return {}

    state = {
        "clicks": [],   # raw (x,y) click points collected
        "boxes": [],    # completed [x1,y1,x2,y2] boxes
        "img": image.copy(),
    }

    COLOUR_PENDING = (0, 200, 255)
    COLOUR_DONE = (0, 220, 60)
    COLOUR_LABEL = (255, 255, 255)

    def _redraw(s):
        canvas = image.copy()
        for idx, box in enumerate(s["boxes"]):
            x1, y1, x2, y2 = box
            cv.rectangle(canvas, (x1, y1), (x2, y2), COLOUR_DONE, 2)
            cv.putText(canvas, f"slot_{idx + 1}", (x1, y1 - 6),
                       cv.FONT_HERSHEY_SIMPLEX, 0.55, COLOUR_LABEL, 1)
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
    cv.waitKey(1)

    if not state["boxes"]:
        print("[TRAY] No slots defined - calibration cancelled.")
        return {}

    cfg = {f"slot_{i + 1}": box for i, box in enumerate(state["boxes"])}

    cfg_path = file_path(config_out)
    os.makedirs(os.path.dirname(cfg_path) or ".", exist_ok=True)
    with open(cfg_path, "w") as f:
        json.dump(cfg, f, indent=4)
    print(f"[TRAY] Saved {len(cfg)} slots to '{cfg_path}'.")

    # Reference image: a fresh, settled frame taken AFTER clicking.
    # Tray must be empty and hands out of view.
    print("[TRAY] Capturing reference image - keep tray empty and hands out of view...")
    time.sleep(1.0)
    ref_frame = grab_settled_frame()
    if ref_frame is None:
        ref_frame = image
    ref_path = file_path(ref_out)
    os.makedirs(os.path.dirname(ref_path) or ".", exist_ok=True)
    if not cv.imwrite(ref_path, ref_frame):
        print(f"[TRAY ERROR] Could not write reference image to '{ref_path}'.")
    else:
        print(f"[TRAY] Reference image saved to '{ref_path}'.")

    _ref_img = ref_frame.copy()
    _slot_states.clear()          # old states belong to the old calibration
    _reserved.clear()
    _camera_states.clear()

    _sync_positions(cfg)
    return cfg


# detect slots
def _scan_camera(thresh: float = OCCUPIED_FRACTION, quiet: bool = False) -> tuple[dict, object]:
   
    global _ref_img

    if _ref_img is None:
        rf_path = file_path("test_images/empty_tray.jpg")
        _ref_img = cv.imread(rf_path)
        if _ref_img is None:
            print(f"[TRAY ERROR] Reference image missing: {rf_path}")
            return {}, None

    current_frame = grab_settled_frame(n_warmup=3, n_avg=3)
    if current_frame is None:
        return {}, None

    if current_frame.shape != _ref_img.shape:
        print(f"[TRAY ERROR] Frame size {current_frame.shape} != reference {_ref_img.shape}. "
              f"Recalibrate.")
        return {}, None

    cfg = load_cfg("slot_config.json")
    results: dict[str, str] = {}
    display_frame = current_frame.copy()

    for name, box in cfg.items():
        try:
            x1, y1, x2, y2 = box
            roi_ref = _ref_img[y1:y2, x1:x2]
            roi_cur = current_frame[y1:y2, x1:x2]

            if roi_ref.size == 0 or roi_ref.shape != roi_cur.shape:
                results[name] = "unknown"
                continue

            mean_diff, frac = _slot_metric(roi_ref, roi_cur)
            cam_status = "free" if frac < thresh else "occupied"
            _camera_states[name] = cam_status

            # occupied if the camera sees something OR the robot was already sent there
            reserved = name in _reserved
            status = "occupied" if (reserved or cam_status == "occupied") else "free"
            results[name] = status

            if not quiet:
                print(f"  [SCAN] {name}: mean_diff={mean_diff:.1f}  changed={frac:.2%}  "
                      f"thresh={thresh:.0%}  -> {status}{' (reserved)' if reserved else ''}")

            color = (0, 255, 0) if status == "free" else (0, 0, 255)
            cv.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
            cv.putText(display_frame, f"{name}: {status} ({frac:.0%})", (x1, y1 - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        except Exception as e:
            print(f"[TRAY ERROR] Processing {name}: {e}")

    return results, display_frame


def _annotate_from_state(states: dict) -> object:
    if _last_frame is None:
        return None
    frame = _last_frame.copy()
    cfg = load_cfg("slot_config.json")
    for name, box in cfg.items():
        try:
            x1, y1, x2, y2 = box
            status = states.get(name, "unknown")
            color = (0, 255, 0) if status == "free" else (0, 0, 255)
            cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv.putText(frame, f"{name}: {status}", (x1, y1 - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        except Exception as e:
            print(f"[TRAY ERROR] Annotating {name}: {e}")
    return frame


def get_free_slots(rescan: bool = False, thresh: float = OCCUPIED_FRACTION,
                   quiet: bool = False) -> tuple[dict, object]:
    global _slot_states

    if rescan or not _slot_states:
        with _camera_lock:
            states, frame = _scan_camera(thresh, quiet)
            _slot_states.update(states)
            return dict(_slot_states), frame

    # Fast path: just annotate current frame with known state
    frame = _annotate_from_state(_slot_states)
    return dict(_slot_states), frame


def get_closest_free_slot(robot_xy: list[float] | None = None) -> tuple[str | None, list[float] | None]:
    global _slot_states

    with _camera_lock:     # scan + choose + reserve as one step
        # Always rescan camera live when coordinates are requested
        get_free_slots(rescan=True)

        free = {name: POS[name] for name, state in _slot_states.items()
                if state == "free" and name in POS}

        if not free:
            print("[TRAY] No free slots available.")
            return None, None

        if robot_xy is None or len(robot_xy) < 2:
            chosen = next(iter(free))
        else:
            rx, ry = robot_xy[0], robot_xy[1]
            chosen = min(free, key=lambda n: math.hypot(free[n][0] - rx, free[n][1] - ry))

        coords = list(POS[chosen])

        # mark occupied every time we send coordinates to the robot
        _slot_states[chosen] = "occupied"
        _reserved.add(chosen)
        print(f"[TRAY] Slot '{chosen}' selected @ {coords}  ->  marked occupied.")

        return chosen, coords




def get_robot_coords_for_slot(name: str) -> list[float]:
    if name not in POS:
        raise KeyError(f"Slot '{name}' not found in calibrated positions.")
    return list(POS[name])


def mark_slot_free(name: str) -> None:
    global _slot_states
    _slot_states[name] = "free"
    _reserved.discard(name)
    print(f"[TRAY] Slot '{name}' marked free.")


def release_reservation(name: str) -> None:
  
    with _camera_lock:
        _reserved.discard(name)
        cam = _camera_states.get(name)
        if cam is not None:
            _slot_states[name] = cam


def verify_placement(name: str, thresh: float = OCCUPIED_FRACTION) -> bool:
   
    get_free_slots(rescan=True, thresh=thresh)
    cam_status = _camera_states.get(name)
    _reserved.discard(name)                      # reservation ends at the detection image
    if cam_status is not None:
        _slot_states[name] = cam_status
    return cam_status == "occupied"


# ---- background scanning: keeps the tray view fresh without ever blocking the main loop ----
_view_lock = threading.Lock()
_view_img = None
_bg_thread = None
_bg_stop = threading.Event()


def _bg_loop(period: float) -> None:
    global _view_img
    while not _bg_stop.is_set():
        try:
            _, frame = get_free_slots(rescan=True, quiet=True)
            if frame is not None:
                with _view_lock:
                    _view_img = frame
        except Exception as e:
            print(f"[TRAY ERROR] background scan: {e}")
        _bg_stop.wait(period)


def start_background_scan(period: float = 1.0) -> None:
    #Start scanning the tray in a background thread (opening the camera there too)
    global _bg_thread
    if _bg_thread is not None and _bg_thread.is_alive():
        return
    _bg_stop.clear()
    _bg_thread = threading.Thread(target=_bg_loop, args=(period,), daemon=True)
    _bg_thread.start()
    print(f"[TRAY] Background tray scan started (every {period:.1f}s).")


def stop_background_scan() -> None:
    _bg_stop.set()
    if _bg_thread is not None:
        _bg_thread.join(timeout=3)


def get_cached_view() -> tuple[dict, object]:

    with _view_lock:
        img = None if _view_img is None else _view_img.copy()
    return dict(_slot_states), img


if __name__ == "__main__":
    print("=" * 60)
    print("  TRAY CAMERA - standalone test")
    print("=" * 60)

    load_calibrated_positions()

    answer = input("\nRun interactive slot calibration? (y/n): ").strip().lower()
    if answer == "y":
        calibrate_slots()
        print("\n[TEST] Calibration complete.  Reference image saved (empty tray).")
    else:
        print("[TEST] Skipping calibration - using existing slot_config.json.")

    print("\n[TEST] Taking one snapshot to detect slot states...")
    slots, img = get_free_slots(rescan=True)
    print(f"[TEST] Detected states: {slots}")

    if img is not None:
        WIN = "Slot Detection Result  |  any key to continue"
        cv.namedWindow(WIN, cv.WINDOW_NORMAL)
        cv.imshow(WIN, img)
        print("[TEST] Showing detected slots - press any key to continue.")
        cv.waitKey(0)
        cv.destroyWindow(WIN)

    print("\n[TEST] Sending all free slots to robot - one by one ...\n")
    pick_count = 0
    robot_xy = [300.0, -120.0]

    while True:
        slot_name, coords = get_closest_free_slot(robot_xy)

        if slot_name is None:
            print("[TEST] No more free slots - tray is full.")
            break

        pick_count += 1
        print(f"[TEST] Pick #{pick_count}")
        print(f"[TEST]   -> Chosen slot : {slot_name}")
        print(f"[TEST]   -> Coords sent : {coords}")
        print(f"[TEST]   -> States now  : {_slot_states}")

        robot_xy = coords[:2]

        states, annotated = get_free_slots(rescan=False)
        if annotated is not None:
            free_count = sum(1 for s in states.values() if s == "free")
            win_title = (
                f"After pick #{pick_count}: {slot_name}  |  "
                f"{free_count} slot(s) remaining  |  any key to continue"
            )
            cv.namedWindow(win_title, cv.WINDOW_NORMAL)
            cv.imshow(win_title, annotated)
            print(f"[TEST]   -> {free_count} slot(s) remaining - press any key to continue.\n")
            cv.waitKey(0)
            cv.destroyWindow(win_title)

    cv.destroyAllWindows()
    release_camera()
    print(f"\n[TEST] Done - {pick_count} slot(s) dispatched.")"""


import os
import sys
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")
import json
import time
import threading
import cv2 as cv
import numpy as np

CAM = 1
PIX = 25
FRAC = 0.12
BLUR = (7, 7)
AGE = 3.0
DIR = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(DIR, "slot_config.json")
REF = os.path.join(DIR, "test_images", "empty_tray.jpg")

COORDS = {
    "slot_1": [350.0, -150.0, 45.0],
    "slot_2": [350.0, -250.0, 45.0],
    "slot_3": [350.0, -350.0, 45.0],
    "slot_4": [420.0, -150.0, 45.0],
    "slot_5": [420.0, -250.0, 45.0],
    "slot_6": [420.0, -350.0, 45.0],
    "slot_7": [480.0, -150.0, 45.0],
    "slot_8": [480.0, -250.0, 45.0],
    "slot_9": [480.0, -350.0, 45.0],
    "slot_10": [510.0, -150.0, 45.0],
}

_cap = None
_ref = None
_frame = None
_view = None
_new = False
_when = 0.0
_boxes = None
_res = set()
_seen = {}
_lock = threading.RLock()


def boxes():
    """Saved slot boxes."""
    global _boxes
    if _boxes is None:
        _boxes = {}
        if os.path.exists(CFG):
            with open(CFG) as f:
                _boxes = json.load(f)
    return _boxes


def check():
    """Report slot coordinate match."""
    b = boxes()
    print(f"[TRAY] {len(b)} camera slot(s), {len(COORDS)} robot coordinate(s).")
    miss = sorted(set(b) - set(COORDS))
    if miss:
        print(f"[TRAY WARNING] No robot coordinates for {miss}, unused.")


def _backends():
    """Platform capture backends."""
    if sys.platform.startswith("win"):
        return [cv.CAP_DSHOW, cv.CAP_MSMF]
    if sys.platform == "darwin":
        return [cv.CAP_AVFOUNDATION, cv.CAP_ANY]
    return [cv.CAP_ANY]


def _open():
    """Open the tray camera."""
    for be in _backends():
        t = time.time()
        print(f"[TRAY] Opening camera {CAM}...")
        cap = cv.VideoCapture(CAM, be)
        if not cap.isOpened():
            cap.release()
            continue
        if sys.platform != "darwin":
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(15):
            ok, f = cap.read()
            if ok and f is not None:
                print(f"[TRAY] Camera ready in {time.time() - t:.1f}s")
                return cap
            time.sleep(0.1)
        cap.release()
    return None


def _grab(wait=0.15, n=3):
    """Settled averaged frame."""
    global _cap, _frame
    if _cap is None or not _cap.isOpened():
        _cap = _open()
        if _cap is None:
            print(f"[TRAY ERROR] Cannot open camera {CAM}.")
            return None
    end = time.time() + wait
    while time.time() < end:
        _cap.read()
        time.sleep(0.03)
    fs = []
    for _ in range(n):
        ok, f = _cap.read()
        if ok and f is not None:
            fs.append(f.astype(np.float32))
    if not fs:
        print("[TRAY ERROR] No camera frame.")
        return None
    _frame = np.mean(fs, axis=0).astype(np.uint8)
    return _frame


def _diff(a, b):
    """Changed pixel fraction."""
    a, b = [cv.GaussianBlur(cv.cvtColor(x, cv.COLOR_BGR2GRAY), BLUR, 0).astype(np.float32) for x in (a, b)]
    return float(np.mean(np.abs((b - b.mean()) - (a - a.mean())) > PIX))


def _draw():
    """Render slot status image."""
    global _view, _new
    if _frame is None:
        return
    v = _frame.copy()
    for n, (x1, y1, x2, y2) in boxes().items():
        if n in _res:
            txt, col = "next", (0, 255, 255)
        elif _seen.get(n) == "free":
            txt, col = "free", (0, 255, 0)
        else:
            txt, col = "occupied", (0, 0, 255)
        cv.rectangle(v, (x1, y1), (x2, y2), col, 2)
        cv.putText(v, f"{n}: {txt}", (x1, max(y1 - 10, 12)), cv.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    _view, _new = v, True


def _scan():
    """Compare frame with reference."""
    global _ref, _when
    if _ref is None:
        _ref = cv.imread(REF)
        if _ref is None:
            print("[TRAY ERROR] No reference image. Calibrate.")
            return False
    f = _grab()
    if f is None:
        return False
    if f.shape != _ref.shape:
        print("[TRAY ERROR] Camera size changed. Calibrate.")
        return False
    out = []
    for n, (x1, y1, x2, y2) in boxes().items():
        r = _diff(_ref[y1:y2, x1:x2], f[y1:y2, x1:x2])
        _seen[n] = "free" if r < FRAC else "occupied"
        out.append(f"{n} {_seen[n]} ({r:.0%})")
    print("[SCAN] " + ", ".join(out))
    _when = time.time()
    _draw()
    return True


def reserve():
    """Reserve first free slot."""
    with _lock:
        if time.time() - _when > AGE and not _scan():
            return None
        for n in boxes():
            if _seen.get(n) == "free" and n not in _res and n in COORDS:
                _res.add(n)
                _draw()
                print(f"[TRAY] Slot '{n}' initialized @ {COORDS[n]}")
                return n, [float(v) for v in COORDS[n]]
        print("[TRAY] No free slot.")
        return None


def verify(n):
    """Check slot holds cup."""
    with _lock:
        ok = _scan()
        _res.discard(n)
        _draw()
        return _seen.get(n) == "occupied" if ok else None


def drop(n):
    """Release slot reservation."""
    with _lock:
        _res.discard(n)
        _draw()


def view():
    """New tray image, else None."""
    global _new
    if not _new:
        return None
    _new = False
    return _view


def close():
    """Release the camera."""
    global _cap
    with _lock:
        if _cap is not None:
            _cap.release()
            _cap = None


def _click(img):
    """Click two corners per slot."""
    bx, pts = [], []
    win = "Calibrate | 2 clicks per slot | u undo | q done"

    def mouse(ev, x, y, fl, pr):
        if ev != cv.EVENT_LBUTTONDOWN:
            return
        pts.append((x, y))
        if len(pts) == 2:
            (a, b), (c, d) = pts
            pts.clear()
            if a != c and b != d:
                bx.append([min(a, c), min(b, d), max(a, c), max(b, d)])
                print(f"  slot_{len(bx)} -> {bx[-1]}")

    def draw():
        v = img.copy()
        for i, (x1, y1, x2, y2) in enumerate(bx):
            cv.rectangle(v, (x1, y1), (x2, y2), (0, 220, 60), 2)
            cv.putText(v, f"slot_{i + 1}", (x1, max(y1 - 6, 12)), cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        for p in pts:
            cv.circle(v, p, 5, (0, 200, 255), -1)
        return v

    cv.namedWindow(win, cv.WINDOW_NORMAL)
    cv.setMouseCallback(win, mouse)
    while True:
        cv.imshow(win, draw())
        k = cv.waitKey(20) & 0xFF
        if k == ord("q") or cv.getWindowProperty(win, cv.WND_PROP_VISIBLE) < 1:
            break
        if k == ord("u") and bx:
            bx.pop()
            pts.clear()
    cv.destroyWindow(win)
    cv.waitKey(1)
    return bx


def calibrate():
    """Click slots, save reference."""
    global _boxes, _ref, _when
    print("[TRAY] Keep tray empty and arm out of view.")
    img = _grab(1.5, 5)
    if img is None:
        return False
    bx = _click(img)
    if not bx:
        print("[TRAY] No slots drawn. Nothing saved.")
        return False
    print("[TRAY] Capturing empty-tray reference...")
    ref = _grab(1.5, 5)
    if ref is None:
        return False
    os.makedirs(os.path.dirname(REF), exist_ok=True)
    if not cv.imwrite(REF, ref):
        print("[TRAY ERROR] Cannot save reference image.")
        return False
    cfg = {f"slot_{i + 1}": b for i, b in enumerate(bx)}
    with open(CFG, "w") as f:
        json.dump(cfg, f, indent=4)
    with _lock:
        _boxes, _ref, _when = cfg, ref, 0.0
        _res.clear()
        _seen.clear()
    print(f"[TRAY] Saved {len(cfg)} slot(s).")
    return True