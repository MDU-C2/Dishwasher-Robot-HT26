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
FRAC = 0.06
BLUR = (7, 7)
AGE = 3.0
TRIES = 10
DIR = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(DIR, "slot_config.json")
REF = os.path.join(DIR, "test_images", "empty_tray.jpg")


#Change before running
COORDS = {
 #   "slot_1": [335.0, -402.0, 160.0],
 #  "slot_2": [441.0, -402.0, 160.0],
 #  "slot_3": [506.0, -166.0, 170.0],
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
_miss = {}
_lock = threading.RLock()

# --- background capture thread state ---
_cap_thread = None
_cap_running = False
_latest_frame = None
_frame_lock = threading.Lock()


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


def _capture_loop():
    """Continuously drain the camera so no backlog can build up.

    Runs in its own thread for the lifetime of the camera connection.
    Always overwrites _latest_frame, so only the single newest frame
    is ever kept - old frames are discarded as fast as the camera
    produces them instead of piling up in an OS/driver buffer.
    """
    global _cap, _latest_frame, _cap_running
    while _cap_running:
        if _cap is None or not _cap.isOpened():
            time.sleep(0.05)
            continue
        ok, f = _cap.read()
        if ok and f is not None:
            with _frame_lock:
                _latest_frame = f
        # no sleep here on purpose - we want to drain as fast as the
        # camera delivers frames, so nothing ever queues up


def _ensure_capture_thread():
    global _cap, _cap_thread, _cap_running
    if _cap is None or not _cap.isOpened():
        _cap = _open()
        if _cap is None:
            print(f"[TRAY ERROR] Cannot open camera {CAM}.")
            return False
    if _cap_thread is None or not _cap_thread.is_alive():
        _cap_running = True
        _cap_thread = threading.Thread(target=_capture_loop, daemon=True)
        _cap_thread.start()
    return True


def _grab(wait=0.15, n=3):
    """Settled averaged frame, drawn from the always-fresh background thread."""
    global _frame
    if not _ensure_capture_thread():
        return None

    # brief settle time (e.g. after camera just opened, or lighting flicker),
    # but we are reading the live background-thread frame, not draining a
    # backlog, so this no longer compounds over repeated calls
    end = time.time() + wait
    while time.time() < end:
        time.sleep(0.02)

    fs = []
    deadline = time.time() + 1.0
    last_seen = None
    while len(fs) < n and time.time() < deadline:
        with _frame_lock:
            f = _latest_frame
        if f is not None and (last_seen is None or not np.array_equal(f, last_seen)):
            fs.append(f.astype(np.float32))
            last_seen = f
        else:
            time.sleep(0.01)

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
    return True
