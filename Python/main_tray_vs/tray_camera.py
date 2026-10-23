"""
tray_camera.py - 2D tray camera: which dishwasher slots are free, and where the robot should place a mug.

Detection: both pictures are turned into blurred grayscale and each is shifted to zero mean brightness, so a change in
room lighting does not look like a mug. A slot is OCCUPIED when more than FRAC of its pixels still differ by PIX.
A background thread keeps only the newest camera frame, so old frames can never pile up.

Two calling styles are supported (use either):

  Style A - main_tray.py / main_trayOriginal.py (current):
      load_calibrated_positions("slot_positions.json")
      get_free_slots()                 -> (states, annotated_image)  states = {"slot_1": "free" | "occupied" | "unknown"}
      get_robot_coords_for_slot(name)  -> [x, y, z]
      verify_placement(name)           -> True / False (keep checking) / None (gave up)
      release_camera()

  Style B - original main_tray.py with setup():
      boxes()  check()  calibrate()  reserve() -> (slot, [x, y, z]) | None
      verify(slot)  drop(slot)  view() -> new image | None  close()

Run `python tray_camera.py` to calibrate or to watch the live view.

Robot coordinates of a slot: if slot_transform.json exists (see below) they are computed from the MIDDLE of the slot's
pixel box in slot_config.json, so moving or redrawing a box moves the target. Otherwise they come from
slot_positions.json (or COORDS). Create the camera -> robot mapping once with
    python tray_camera.py --calibrate-transform   click 4+ points on the tray, type each point's robot X Y Z (pendant)
    python tray_camera.py --fit-from-slots        or: fit it from slots whose robot x, y, z you already measured
Redo it if the 2D camera or the robot base is ever moved.

Files (next to this script): slot_config.json (pixel boxes), test_images/empty_tray.jpg (empty-tray reference),
slot_positions.json (robot x, y, z per slot), slot_transform.json (camera -> robot mapping).
"""
import os
import sys
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")   # before cv2 is imported: fast webcam start on Windows
import json
import re
import time
import threading

import cv2 as cv
import numpy as np

# ===========================================================
# CONFIGURATION
# ===========================================================
USE_CAM = True              # False = read test_images/current_state.jpg instead of a camera (for testing)
CAM = 1                     # camera index of the tray camera (1 = USB / DroidCam / Iriun on Windows)
PIX = 25                    # a pixel "differs" when its brightness-normalised difference is above this (0-255)
FRAC = 0.06                 # slot is OCCUPIED when more than this fraction of its pixels differ
BLUR = (7, 7)
AGE = 3.0                   # reserve() rescans when its last scan is older than this (seconds)
TRIES = 10                  # verify() gives up after this many "not seen" answers in a row
LIVE_AGE = 1.0              # get_free_slots() reuses the last scan if it is younger than this (seconds)
FRAME_MAX_AGE = 1.0         # a camera frame older than this counts as "no frame" (frozen / unplugged camera)
OPEN_RETRY_SECONDS = 5.0    # after a failed camera open, wait this long before trying again
DIR = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(DIR, "slot_config.json")
REF = os.path.join(DIR, "test_images", "empty_tray.jpg")

#Change before running (optional: slot_positions.json is used too and wins over these)
COORDS = {
 #   "slot_1": [335.0, -402.0, 160.0],
 #  "slot_2": [441.0, -402.0, 160.0],
 #  "slot_3": [506.0, -166.0, 170.0],
}

# Robot coordinates of a slot:
#   "auto" = from the middle of the slot's pixel box via slot_transform.json when that file exists, else slot_positions.json
#   "json" = always slot_positions.json / COORDS (the older behaviour)
COORD_SOURCE = "auto"
SLOT_Z = None                # optional fixed release height in mm; overrides the z stored in slot_transform.json
TRANSFORM = os.path.join(DIR, "slot_transform.json")
_tf = None
_tf_tried = False

POS: dict[str, list[float]] = {}     # robot coordinates loaded from slot_positions.json
_pos_tried = False
_cap = None
_ref = None
_frame = None
_view = None
_new = False
_when = 0.0
_boxes = None
_res = set()
_seen = {}
_miss = {}
_lock = threading.RLock()

# --- background capture thread state ---
_cap_thread = None
_cap_running = False
_cap_lost = False
_latest_frame = None
_latest_time = 0.0
_frame_lock = threading.Lock()
_next_open_try = 0.0
_last_warn: dict[str, float] = {}


def _warn(key, msg, every=10.0):
    """Print at most once per `every` seconds (the display loop calls in here many times per second)."""
    now = time.time()
    if now - _last_warn.get(key, 0.0) >= every:
        _last_warn[key] = now
        print(msg)


# ===========================================================
# FILES AND ROBOT COORDINATES
# ===========================================================
def file_path(p: str) -> str:
    if os.path.exists(p):
        return p
    d = os.path.dirname(os.path.abspath(__file__))
    a1 = os.path.join(d, p)
    if os.path.exists(a1):
        return a1
    a2 = os.path.join(d, "..", "..", p)
    if os.path.exists(a2):
        return a2
    return p


def set_slot_positions(positions: dict[str, list[float]]):
    global POS
    POS.clear()
    POS.update(positions)


def load_calibrated_positions(path: str = "slot_positions.json") -> None:
    """Robot x, y, z for every slot name. Raises FileNotFoundError if the file is missing."""
    p = file_path(path)
    if not os.path.exists(p):
        raise FileNotFoundError(f"No calibration file: {p}")
    with open(p, "r") as f:
        data = json.load(f)
    set_slot_positions(data)
    print(f"[TRAY] Loaded {len(POS)} slot coordinates from '{p}'.")


def _json_coords() -> dict:
    """slot -> [x, y, z] from slot_positions.json (loaded on first use) over the COORDS defaults in this file."""
    global _pos_tried
    if not POS and not _pos_tried:
        _pos_tried = True
        try:
            load_calibrated_positions()
        except Exception:
            pass
    merged = {k: list(v) for k, v in COORDS.items()}
    merged.update({k: list(v) for k, v in POS.items()})
    return merged


def _coords() -> dict:
    """slot -> [x, y, z] used for placing: the middle of each slot box through slot_transform.json when available."""
    merged = _json_coords()
    if COORD_SOURCE == "auto" and _load_transform() is not None:
        for name in boxes():
            merged[name] = _transform_coords(name)
    return merged


def get_robot_coords_for_slot(name: str) -> list[float]:
    c = _coords()
    if name not in c:
        raise KeyError(f"Unknown slot '{name}'. Ensure slot_positions.json is loaded.")
    return [float(v) for v in c[name]]


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
    c = _coords()
    tf = _load_transform() if COORD_SOURCE == "auto" else None
    if tf is not None:
        print(f"[TRAY] Robot coordinates: middle of each slot box -> robot via slot_transform.json "
              f"({tf['kind']}, fit error {tf['rms']} mm, z={SLOT_Z if SLOT_Z is not None else tf['z']}).")
    else:
        print("[TRAY] Robot coordinates: from slot_positions.json / COORDS (no slot_transform.json).")
    print(f"[TRAY] {len(b)} camera slot(s), {len(c)} robot coordinate(s).")
    miss = sorted(set(b) - set(c))
    if miss:
        print(f"[TRAY WARNING] No robot coordinates for {miss}, unused.")


# ===========================================================
# SLOT BOX MIDDLE -> ROBOT COORDINATES
# ===========================================================
def _load_transform():
    """slot_transform.json -> {"H": 3x3, "z", "kind", "rms"} or None. Cached."""
    global _tf, _tf_tried
    if not _tf_tried:
        _tf_tried = True
        if os.path.exists(TRANSFORM):
            try:
                with open(TRANSFORM) as f:
                    d = json.load(f)
                _tf = {"H": np.array(d["matrix"], dtype=np.float64), "z": float(d["z"]),
                       "kind": d.get("kind", "homography"), "rms": d.get("rms_mm")}
            except Exception as e:
                print(f"[TRAY ERROR] Cannot read {TRANSFORM}: {e}")
    return _tf


def slot_center(name: str) -> tuple[float, float]:
    """Pixel middle of a slot box from slot_config.json."""
    x1, y1, x2, y2 = boxes()[name]
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def _apply(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def pixel_to_robot(u: float, v: float) -> tuple[float, float]:
    """Robot x, y (mm) of a pixel on the tray plane."""
    tf = _load_transform()
    if tf is None:
        raise RuntimeError("No slot_transform.json - run: python tray_camera.py --calibrate-transform")
    return _apply(tf["H"], u, v)


def _transform_coords(name: str) -> list[float]:
    tf = _load_transform()
    x, y = pixel_to_robot(*slot_center(name))
    z = SLOT_Z if SLOT_Z is not None else tf["z"]
    return [round(x, 1), round(y, 1), round(float(z), 1)]


def fit_transform(points) -> tuple[float, float]:
    """
    points = [((pixel_u, pixel_v), (robot_x, robot_y, robot_z)), ...]  -> saves slot_transform.json.
    3 points give an exact affine map, 4 or more a least-squares homography. Returns (rms_mm, max_mm).
    """
    global _tf, _tf_tried
    if len(points) < 3:
        raise ValueError("Need at least 3 reference points (4 or more recommended).")
    src = np.array([p[0] for p in points], dtype=np.float64)
    xyz = np.array([p[1] for p in points], dtype=np.float64)
    dst = xyz[:, :2]
    sv = np.linalg.svd(src - src.mean(axis=0), compute_uv=False)
    if sv[-1] < 1e-6 or sv[-1] / sv[0] < 0.05:
        raise ValueError("The reference points are (almost) in a line - spread them over the tray.")
    if len(points) == 3:
        M = cv.getAffineTransform(src.astype(np.float32), dst.astype(np.float32))
        H, kind = np.vstack([M, [0.0, 0.0, 1.0]]), "affine"
    else:
        H, _ = cv.findHomography(src, dst, 0)
        kind = "homography"
        if H is None:
            raise ValueError("Could not fit a transform to these points.")
    err = np.array([np.hypot(*(np.array(_apply(H, u, v)) - d)) for (u, v), d in zip(src, dst)])
    rms, worst = float(np.sqrt(np.mean(err ** 2))), float(err.max())
    z_spread = float(np.ptp(xyz[:, 2]))
    data = {"kind": kind, "matrix": H.tolist(), "z": float(xyz[:, 2].mean()), "z_spread_mm": z_spread,
            "rms_mm": round(rms, 2), "max_mm": round(worst, 2),
            "points": [{"pixel": list(map(float, p[0])), "robot": list(map(float, p[1]))} for p in points],
            "saved": time.strftime("%Y-%m-%d %H:%M:%S")}
    with open(TRANSFORM, "w") as f:
        json.dump(data, f, indent=4)
    _tf, _tf_tried = None, False                    # reload on next use
    print(f"[TRAY] Saved {kind} map from {len(points)} points: error rms {rms:.1f} mm, worst {worst:.1f} mm, "
          f"z = {data['z']:.1f} mm.")
    if worst > 5:
        print("[TRAY WARNING] Worst error is over 5 mm - re-measure the points or add more of them.")
    if z_spread > 5:
        print(f"[TRAY WARNING] The reference z values differ by {z_spread:.1f} mm; the mean is used as release height.")
    return rms, worst


def fit_from_slots() -> bool:
    """Fit the mapping from slots whose robot x, y, z you already measured (slot_positions.json / COORDS)."""
    known = _json_coords()
    pts = [(slot_center(n), known[n]) for n in boxes() if n in known]
    print(f"[TRAY] {len(pts)} slot(s) have both a pixel box and measured robot coordinates.")
    try:
        fit_transform(pts)
    except ValueError as e:
        print(f"[TRAY ERROR] {e}")
        return False
    return True


def calibrate_transform() -> bool:
    """
    Click 4+ points on the tray (3 is the minimum), and for each type the robot X Y Z in mm (jog the gripper to the
    point, read the pendant). Best points: the middles of the slots, spread out, not all in one line.
    u = undo, q = fit and save. Needs a console (it asks questions), so run it by hand with vision stopped.
    """
    frame = _grab(1.0, 3)
    if frame is None:
        return False
    pts: list = []                                    # [((u, v), (x, y, z))]
    clicks: list = []
    win = "Camera -> robot | click a point, then type its robot X Y Z | u undo | q done"

    def mouse(ev, x, y, fl, pr):
        if ev == cv.EVENT_LBUTTONDOWN:
            clicks.append((x, y))

    def draw():
        v = frame.copy()
        for n, (x1, y1, x2, y2) in boxes().items():                   # slot middles as a click guide
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            cv.rectangle(v, (x1, y1), (x2, y2), (120, 120, 120), 1)
            cv.drawMarker(v, (cx, cy), (0, 200, 255), cv.MARKER_CROSS, 14, 1)
        for i, ((u, w), _) in enumerate(pts):
            cv.circle(v, (int(u), int(w)), 6, (0, 220, 60), -1)
            cv.putText(v, str(i + 1), (int(u) + 8, int(w) - 8), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return v

    cv.namedWindow(win, cv.WINDOW_NORMAL)
    cv.setMouseCallback(win, mouse)
    print("[TRAY] Click a point, then type its robot X Y Z (mm). Cross marks = slot middles. u = undo, q = done.")
    while True:
        cv.imshow(win, draw())
        k = cv.waitKey(20) & 0xFF
        while clicks:
            u, w = clicks.pop(0)
            raw = input(f"  Point {len(pts) + 1} at pixel ({u}, {w}) - robot X Y Z in mm (empty = skip): ")
            nums = re.findall(r"-?\d+(?:\.\d+)?", raw)
            if len(nums) == 3:
                pts.append(((float(u), float(w)), tuple(float(x) for x in nums)))
            else:
                print("  skipped (need exactly three numbers)")
        if k == ord("u") and pts:
            pts.pop()
            print(f"  removed - {len(pts)} point(s) left")
        if k == ord("q") or cv.getWindowProperty(win, cv.WND_PROP_VISIBLE) < 1:
            break
    cv.destroyWindow(win)
    cv.waitKey(1)
    try:
        fit_transform(pts)
    except ValueError as e:
        print(f"[TRAY ERROR] {e} Nothing saved.")
        return False
    check()
    return True


# ===========================================================
# CAMERA
# ===========================================================
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


def _capture_loop():
    """Continuously drain the camera so no backlog can build up.

    Runs in its own thread for the lifetime of the camera connection. Always overwrites _latest_frame, so only the
    single newest frame is kept. Does not take _lock (a scan holding it waits for frames from this thread).
    """
    global _latest_frame, _latest_time, _cap_lost
    fails = 0
    while _cap_running:
        cap = _cap
        if cap is None or not cap.isOpened():
            time.sleep(0.05)
            continue
        ok, f = cap.read()
        if ok and f is not None:
            fails = 0
            with _frame_lock:
                _latest_frame, _latest_time = f, time.time()
        else:
            fails += 1
            time.sleep(0.02)
            if fails > 100:                 # ~2 s without a frame: camera unplugged / frozen -> reopen on next use
                _cap_lost = True
                fails = 0
        # no sleep on success on purpose - drain as fast as the camera delivers


def _ensure_capture_thread():
    global _cap, _cap_thread, _cap_running, _cap_lost, _next_open_try
    if _cap is not None and _cap_lost:
        print("[TRAY] Camera stopped delivering frames - reopening.")
        _cap.release()
        _cap, _cap_lost = None, False
    if _cap is None or not _cap.isOpened():
        if time.time() < _next_open_try:        # don't stall the vision loop retrying on every frame
            return False
        _cap = _open()
        if _cap is None:
            _next_open_try = time.time() + OPEN_RETRY_SECONDS
            _warn("open", f"[TRAY ERROR] Cannot open camera {CAM}.")
            return False
    if _cap_thread is None or not _cap_thread.is_alive():
        _cap_running = True
        _cap_thread = threading.Thread(target=_capture_loop, daemon=True)
        _cap_thread.start()
    return True


def _grab(wait=0.15, n=3):
    """Settled averaged frame, drawn from the always-fresh background thread."""
    global _frame
    if not USE_CAM:
        f = cv.imread(file_path("test_images/current_state.jpg"))
        if f is None:
            _warn("static", "[TRAY ERROR] test_images/current_state.jpg not found.")
            return None
        _frame = f
        return f

    if not _ensure_capture_thread():
        return None

    # brief settle time (e.g. after the camera just opened, or lighting flicker); we read the live background-thread
    # frame, not a backlog, so this does not compound over repeated calls
    end = time.time() + wait
    while time.time() < end:
        time.sleep(0.02)

    fs = []
    deadline = time.time() + 1.0
    last_seen = None
    while len(fs) < n and time.time() < deadline:
        with _frame_lock:
            f, t = _latest_frame, _latest_time
        if f is not None and time.time() - t < FRAME_MAX_AGE:
            if last_seen is None or not np.array_equal(f, last_seen):
                fs.append(f.astype(np.float32))
                last_seen = f
                continue
        time.sleep(0.01)

    if not fs:
        _warn("noframe", "[TRAY ERROR] No camera frame.")
        return None
    _frame = np.mean(fs, axis=0).astype(np.uint8)
    return _frame


# ===========================================================
# DETECTION
# ===========================================================
def _diff(a, b):
    """Changed pixel fraction (brightness-normalised)."""
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


def _scan(n=3, wait=0.15, quiet=False, frac=None):
    """Compare a fresh frame with the reference. quiet=True prints only when a slot changes state."""
    global _ref, _when
    if _ref is None:
        _ref = cv.imread(REF)
        if _ref is None:
            _warn("ref", "[TRAY ERROR] No reference image. Calibrate.")
            return False
    f = _grab(wait, n)
    if f is None:
        return False
    if f.shape != _ref.shape:
        _warn("size", "[TRAY ERROR] Camera size changed. Calibrate.")
        return False
    limit = FRAC if frac is None else frac
    fh, fw = f.shape[:2]
    prev = dict(_seen)
    out = []
    for name, (x1, y1, x2, y2) in boxes().items():
        if not (0 <= x1 < x2 <= fw and 0 <= y1 < y2 <= fh):      # box sticks out of the picture: never trust it
            _seen[name] = "unknown"
            out.append(f"{name} unknown")
            continue
        r = _diff(_ref[y1:y2, x1:x2], f[y1:y2, x1:x2])
        _seen[name] = "free" if r < limit else "occupied"
        out.append(f"{name} {_seen[name]} ({r:.0%})")
    if not quiet or any(prev.get(k) != v for k, v in _seen.items()):
        print("[SCAN] " + ", ".join(out))
    _when = time.time()
    _draw()
    return True


# ===========================================================
# STYLE A: main_tray.py calls
# ===========================================================
def get_free_slots(thresh=None, rescan=False, max_age=None):
    """
    Returns (states, annotated_image). The last scan is reused while it is younger than LIVE_AGE seconds (so the
    display loop is cheap); rescan=True always takes a fresh, frame-averaged scan (use it when choosing a slot).
    `thresh` is only honoured as a fraction between 0 and 1; the older pixel-difference values (20, 50) are ignored.
    """
    frac = thresh if (thresh is not None and 0 < thresh <= 1) else None
    age = LIVE_AGE if max_age is None else max_age
    with _lock:
        if rescan or frac is not None:
            ok = _scan(frac=frac)
        elif time.time() - _when > age:
            ok = _scan(n=1, wait=0.0, quiet=True)
        else:
            ok = True
        if not ok:
            return {}, None
        return {n: _seen.get(n, "unknown") for n in boxes()}, _view


def verify_placement(name: str, thresh=None) -> bool:
    """True when the slot holds a mug, False while it does not, None after TRIES misses in a row (gave up)."""
    return verify(name)


# ===========================================================
# STYLE B: original main_tray.py calls
# ===========================================================
def reserve():
    """Reserve first free slot."""
    with _lock:
        if time.time() - _when > AGE and not _scan():
            return None
        c = _coords()
        for n in boxes():
            if _seen.get(n) == "free" and n not in _res and n in c:
                _res.add(n)
                _draw()
                print(f"[TRAY] Slot '{n}' initialized @ {c[n]}")
                return n, [float(v) for v in c[n]]
        print("[TRAY] No free slot.")
        return None


def verify(n):
    """Check slot holds cup."""
    with _lock:
        ok = _scan()
        _res.discard(n)
        _draw()
        if not ok:
            _miss.pop(n, None)
            return None
        if _seen.get(n) == "occupied":
            _miss.pop(n, None)
            return True
        _miss[n] = _miss.get(n, 0) + 1
        if _miss[n] < TRIES:
            return False
        _miss.pop(n)
        print(f"[TRAY] Cup not seen in '{n}'. Giving up.")
        return None


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
    """Release the camera and stop the background capture thread."""
    global _cap, _cap_running, _cap_thread
    with _lock:
        _cap_running = False
        if _cap_thread is not None:
            _cap_thread.join(timeout=1.0)
            _cap_thread = None
        if _cap is not None:
            _cap.release()
            _cap = None


release_camera = close


# ===========================================================
# CALIBRATION
# ===========================================================
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
        _miss.clear()
    print(f"[TRAY] Saved {len(cfg)} slot(s).")
    if os.path.exists(TRANSFORM):
        print("[TRAY] slot_transform.json is still used for robot coordinates. If the 2D camera was moved, "
              "redo it: python tray_camera.py --calibrate-transform")
    check()
    return True


calibrate_slots = calibrate


# ===========================================================
# STANDALONE: calibrate / live view
# ===========================================================
if __name__ == "__main__":
    if "--calibrate" in sys.argv:          # used by the YuMi panel's "Calibrate 2D" button: calibrate, then quit
        try:
            done = calibrate()
        finally:
            close()
            cv.destroyAllWindows()
        sys.exit(0 if done else 1)
    if "--calibrate-transform" in sys.argv:   # camera pixels -> robot mm (needs a console, vision stopped)
        try:
            done = calibrate_transform()
        finally:
            close()
            cv.destroyAllWindows()
        sys.exit(0 if done else 1)
    if "--fit-from-slots" in sys.argv:
        sys.exit(0 if fit_from_slots() else 1)
    print("=" * 60)
    print("  TRAY CAMERA - standalone")
    print("=" * 60)
    try:
        load_calibrated_positions()
    except FileNotFoundError as e:
        print(f"[INFO] {e}")
    check()
    try:
        if input("\nCalibrate slots now (tray EMPTY, arm out of view)? (y/n): ").strip().lower() == "y":
            calibrate()
        print("\nLive view: green = free, red = occupied. Press q in the window to quit.")
        while True:
            _states, img = get_free_slots()
            if img is not None:
                cv.imshow("Tray Test (press q to quit)", img)
            if cv.waitKey(30) & 0xFF == ord("q"):
                break
    finally:
        close()
        cv.destroyAllWindows()
