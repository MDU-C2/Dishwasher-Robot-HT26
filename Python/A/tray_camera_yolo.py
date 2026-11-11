import os
import sys
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")
os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/ultralytics")
import json
import time
import threading
import cv2 as cv
import numpy as np
CAM = int(os.environ.get("TRAY_CAM", 1))
YOLO_CONF = 0.35                                             
OVERLAP_THRESH = 0.30                                                                      
PIX = 40                                           
FRAC = 0.15                                       
BLUR = (7, 7)
TRIES = 10
DELAY = 1.0
GAP = 0.5
MAXAGE = 1.0
RETRY = 5.0
SCAN_DELAY = 1.5
SCAN_FRAMES = 3
REF_COUNTDOWN = 5
REF_FRAMES = 10
DIR = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(DIR, "slot_config.json")
POS_CFG = os.path.join(DIR, "slot_positions.json")
REF = os.path.join(DIR, "test_images", "empty_tray.jpg")
BEST_PT = os.path.join(DIR, "best.pt")
_X_VALS = [340.0, 420.0, 500.0, 580.0, 660.0]
def _build_default_coords():
    coords = {}
    slot_idx = 1
    for row, y in enumerate([-420.0, -280.0, -100.0]):
        for x in _X_VALS:
            coords[f"slot_{slot_idx}"] = [x, y, 180.0]
            slot_idx += 1
    return coords
def load_robot_coords():
    coords = _build_default_coords()
    if os.path.exists(POS_CFG):
        try:
            with open(POS_CFG, "r") as f:
                saved = json.load(f)
                if isinstance(saved, dict) and saved:
                    coords.update(saved)
        except Exception as e:
            print(f"[TRAY WARNING] Error reading {POS_CFG}: {e}")
    return coords
COORDS = load_robot_coords()
_yolo_model = None
_yolo_failed = False
def get_yolo_model():
    global _yolo_model, _yolo_failed
    if _yolo_model is not None:
        return _yolo_model
    if _yolo_failed:
        return None
    if not os.path.exists(BEST_PT):
        print(f"[TRAY YOLO WARNING] Model file not found at: {BEST_PT}. Falling back to image diff.")
        _yolo_failed = True
        return None
    try:
        from ultralytics import YOLO
        print(f"[TRAY YOLO] Loading YOLO model from {BEST_PT}...")
        model = YOLO(BEST_PT)
        print(f"[TRAY YOLO] Model loaded successfully: classes={model.names}")
        _yolo_model = model
        return _yolo_model
    except Exception as exc:
        print(f"[TRAY YOLO ERROR] Could not initialize YOLO: {exc}. Falling back to image diff.")
        _yolo_failed = True
        return None
_cap = None
_ref = None
_frame = None
_view = None
_boxes = None
_st = {}                                                    
_cam = {}                                                    
_detected_cups = []                                                          
_lock = threading.RLock()
_ul = threading.Lock()
_fl = threading.Lock()
_thr = None
_scan_thr = None
_pending_check = {}
_go = False
_lost = False
_last = None
_t = 0.0
_next = 0.0
_warned = {}
def _warn(key, msg, every=10.0):
    now = time.time()
    if now - _warned.get(key, 0.0) >= every:
        _warned[key] = now
        print(msg)
def boxes():
    global _boxes
    if _boxes is None:
        _boxes = {}
        if os.path.exists(CFG):
            try:
                with open(CFG) as f:
                    _boxes = json.load(f)
            except Exception as e:
                print(f"[TRAY ERROR] Cannot load {CFG}: {e}")
    return _boxes
def report():
    b = boxes()
    coords = load_robot_coords()
    print(f"[TRAY] {len(b)} camera slot(s), {len(coords)} robot coordinate(s).")
    miss = sorted(set(b) - set(coords))
    if miss:
        print(f"[TRAY WARNING] No robot coordinates for {miss}, unused.")
check = report                                              
def _backends():
    if sys.platform.startswith("win"):
        return [cv.CAP_DSHOW, cv.CAP_MSMF]
    if sys.platform == "darwin":
        return [cv.CAP_AVFOUNDATION, cv.CAP_ANY]
    return [cv.CAP_ANY]
def _open():
    search_indices = [CAM] + [i for i in [0, 1, 2] if i != CAM]
    for idx in search_indices:
        for be in _backends():
            t = time.time()
            cap = cv.VideoCapture(idx, be)
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
                if ok and f is not None and f.size > 0:
                    print(f"[TRAY] Camera {idx} ready in {time.time() - t:.1f}s")
                    return cap
                time.sleep(0.1)
            cap.release()
    return None
def _loop():
    global _last, _t, _lost
    fails = 0
    while _go:
        cap = _cap
        if cap is None or not cap.isOpened():
            time.sleep(0.05)
            continue
        ok, f = cap.read()
        if ok and f is not None:
            fails = 0
            with _fl:
                _last, _t = f, time.time()
        else:
            fails += 1
            time.sleep(0.02)
            if fails > 100:
                _lost = True
                fails = 0
def _up():
    global _cap, _thr, _scan_thr, _go, _lost, _next
    with _ul:
        if _cap is not None and _lost:
            print("[TRAY] Camera stopped delivering frames, reopening.")
            _cap.release()
            _cap, _lost = None, False
        if _cap is None or not _cap.isOpened():
            if time.time() < _next:
                return False
            _cap = _open()
            if _cap is None:
                _next = time.time() + RETRY
                _warn("open", f"[TRAY ERROR] Cannot open camera {CAM}.")
                return False
        if _thr is None or not _thr.is_alive():
            _go = True
            _thr = threading.Thread(target=_loop, daemon=True)
            _thr.start()
        if _scan_thr is None or not _scan_thr.is_alive():
            _scan_thr = threading.Thread(target=_scanner_loop, daemon=True)
            _scan_thr.start()
        return True
def _grab(wait=0.05, n=1):
    global _frame
    if not _up():
        return None
    if wait > 0:
        time.sleep(wait)
    deadline = time.time() + 1.0
    seen_ts = set()
    fs = []
    while len(fs) < n and time.time() < deadline:
        with _fl:
            f, t = _last, _t
        if f is not None and time.time() - t < MAXAGE and t not in seen_ts:
            seen_ts.add(t)
            fs.append(f.astype(np.float32))
        else:
            time.sleep(0.01)
    if not fs:
        _warn("noframe", "[TRAY ERROR] No camera frame.")
        return None
    _frame = (np.mean(fs, axis=0).astype(np.uint8) if len(fs) > 1 else fs[0].astype(np.uint8))
    return _frame
def _diff(a, b):
    a, b = [cv.GaussianBlur(cv.cvtColor(x, cv.COLOR_BGR2GRAY), BLUR, 0).astype(np.float32) for x in (a, b)]
    return float(np.mean(np.abs((b - b.mean()) - (a - a.mean())) > PIX))
def _box_overlap_check(slot_box, cup_box):
\
\
\
    sx1, sy1, sx2, sy2 = slot_box
    cx1, cy1, cx2, cy2 = cup_box
    cup_mid_x = (cx1 + cx2) / 2.0
    cup_mid_y = (cy1 + cy2) / 2.0
    if sx1 <= cup_mid_x <= sx2 and sy1 <= cup_mid_y <= sy2:
        return True, 1.0
    inter_x1 = max(sx1, cx1)
    inter_y1 = max(sy1, cy1)
    inter_x2 = min(sx2, cx2)
    inter_y2 = min(sy2, cy2)
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    if inter_area <= 0:
        return False, 0.0
    cup_area = max(1, (cx2 - cx1) * (cy2 - cy1))
    ratio = inter_area / float(cup_area)
    return (ratio >= OVERLAP_THRESH), ratio
def _nxt():
    coords = load_robot_coords()
    for n in boxes():
        if _st.get(n) == "free" and n in coords:
            return n
    return None
def _draw():
    global _view
    with _fl:
        f = _last
    if f is None:
        return
    v = f.copy()
    coords = load_robot_coords()
    nx = _nxt()
    with _lock:
        cups_to_draw = list(_detected_cups)
    for (cx1, cy1, cx2, cy2, conf, label) in cups_to_draw:
        cv.rectangle(v, (cx1, cy1), (cx2, cy2), (255, 120, 0), 2)
        cv.putText(v, f"cup {conf:.0%}", (cx1, max(cy1 - 5, 12)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 120, 0), 1)
    for n, (x1, y1, x2, y2) in boxes().items():
        s = _st.get(n, "free")
        if s == "sent":
            txt, col = "sent",      (0, 255, 255)
        elif s == "full":
            txt, col = "full",      (0, 0, 255)
        elif s == "bad":
            txt, col = "check",     (255, 0, 255)
        elif n not in coords:
            txt, col = "no coords", (160, 160, 160)
        elif n == nx:
            txt, col = "next",      (255, 255, 0)
        else:
            txt, col = "free",      (0, 255, 0)
        cv.rectangle(v, (x1, y1), (x2, y2), col, 2)
        cv.putText(v, f"{n}: {txt}", (x1, max(y1 - 10, 12)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    status_text = f"[TRAY YOLO] Model: best.pt | Cups: {len(cups_to_draw)} | Next: {nx or 'None'}"
    cv.putText(v, status_text, (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    _view = v
def _scan(n=1, wait=0.05, quiet=False):
\
\
\
\
    global _detected_cups, _ref
    f = _grab(wait, n)
    if f is None:
        return False
    fh, fw = f.shape[:2]
    model = get_yolo_model()
    results = {}
    detected_cups = []
    out = []
    if model is not None:
        try:
            pred = model.predict(f, conf=YOLO_CONF, verbose=False)[0]
            boxes_data = pred.boxes
            if boxes_data is not None and len(boxes_data) > 0:
                for b in boxes_data:
                    xyxy = b.xyxy[0].cpu().numpy()
                    conf = float(b.conf[0].cpu().numpy())
                    cls_id = int(b.cls[0].cpu().numpy())
                    label = model.names.get(cls_id, "cup")
                    detected_cups.append((int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]), conf, label))
            for k, (x1, y1, x2, y2) in boxes().items():
                if not (0 <= x1 < x2 <= fw and 0 <= y1 < y2 <= fh):
                    results[k] = "unknown"
                    out.append(f"{k} unknown")
                    continue
                cup_found = False
                matched_conf = 0.0
                for (cx1, cy1, cx2, cy2, conf, label) in detected_cups:
                    match, ratio = _box_overlap_check((x1, y1, x2, y2), (cx1, cy1, cx2, cy2))
                    if match:
                        cup_found = True
                        matched_conf = max(matched_conf, conf)
                if cup_found:
                    results[k] = "occupied"
                    out.append(f"{k} occupied ({matched_conf:.0%})")
                else:
                    results[k] = "free"
                    out.append(f"{k} free")
        except Exception as exc:
            print(f"[TRAY YOLO ERROR] Inference failed: {exc}. Falling back to diff.")
            model = None                               
    if model is None:
        with _lock:
            if _ref is None and os.path.exists(REF):
                _ref = cv.imread(REF)
            ref = _ref
        if ref is None:
            _warn("ref", "[TRAY ERROR] No reference image available for fallback. Calibrate.")
            return False
        if f.shape != ref.shape:
            _warn("size", "[TRAY ERROR] Camera size changed. Calibrate.")
            return False
        for k, (x1, y1, x2, y2) in boxes().items():
            if not (0 <= x1 < x2 <= fw and 0 <= y1 < y2 <= fh):
                results[k] = "unknown"
                out.append(f"{k} unknown")
                continue
            r = _diff(ref[y1:y2, x1:x2], f[y1:y2, x1:x2])
            results[k] = "free" if r < FRAC else "occupied"
            out.append(f"{k} {results[k]} ({r:.0%})")
    with _lock:
        prev = dict(_cam)
        _cam.update(results)
        _detected_cups = detected_cups
        if not quiet or prev != _cam:
            print("[SCAN-YOLO] " + ", ".join(out))
    return True
def rescan():
    if not _scan():
        return False
    with _lock:
        for n in boxes():
            if _st.get(n) != "sent":
                _st[n] = "free" if _cam.get(n) == "free" else "full"
        _draw()
    return True
def start():
    with _lock:
        _st.clear()
    return rescan()
def take():
\
\
\
    if SCAN_DELAY > 0:
        time.sleep(SCAN_DELAY)
    if not _scan(n=SCAN_FRAMES, wait=0.05):
        print("[TRAY] Scan failed — cannot confirm slot occupancy.")
        return None
    coords = load_robot_coords()
    with _lock:
        for n in boxes():
            if _st.get(n) not in (None, "free"):
                continue
            if _cam.get(n) == "occupied":
                continue
            if n not in coords:
                continue
            _st[n] = "sent"
            _draw()
            print(f"[TRAY] Slot '{n}' reserved @ {coords[n]}  |  next free: {_nxt()}")
            return n, [float(v) for v in coords[n]]
    print("[TRAY] No free slot available.")
    return None
reserve = take                                              
def _scanner_loop():
    while _go:
        with _lock:
            pending = dict(_pending_check)
        if pending:
            if _scan(n=1, wait=0.05, quiet=True):
                now = time.time()
                missed_slots = []
                with _lock:
                    for n, deadline in list(_pending_check.items()):
                        cam_state = _cam.get(n)
                        if cam_state == "occupied":
                            _st[n] = "full"
                            del _pending_check[n]
                            _draw()
                            print(f"[TRAY] ✓ Cup confirmed by YOLO in '{n}'.")
                        elif now > deadline:
                            _st[n] = "free"
                            del _pending_check[n]
                            missed_slots.append(n)
                            print(
                                f"[TRAY] ✗ Cup not seen in '{n}' — "
                                f"slot freed for reuse. Rescanning all slots..."
                            )
                if missed_slots:
                    time.sleep(SCAN_DELAY)
                    if _scan(n=SCAN_FRAMES, wait=0.2, quiet=False):
                        with _lock:
                            for slot in boxes():
                                if _st.get(slot) == "sent":
                                    continue
                                new_state = "free" if _cam.get(slot) == "free" else "full"
                                _st[slot] = new_state
                            _draw()
        time.sleep(GAP)
def done(n):
    deadline = time.time() + DELAY + TRIES * GAP
    with _lock:
        _pending_check[n] = deadline
    print(f"[TRAY] Queued '{n}' for YOLO cup-placement verification (deadline in {TRIES * GAP:.0f}s).")
def free(n):
    with _lock:
        if _st.get(n) == "sent":
            _st[n] = "free"
            _draw()
            print(f"[TRAY] Slot '{n}' returned to free (motion aborted).")
drop = free                                              
def verify(n):
    with _lock:
        ok = _scan(n=SCAN_FRAMES, wait=0.05)
        _draw()
        if not ok:
            return None
        if _cam.get(n) == "occupied":
            _st[n] = "full"
            return True
        return False
def view():
    _draw()
    return _view
def close():
    global _cap, _go, _thr, _scan_thr
    with _ul:
        _go = False
        if _thr is not None:
            _thr.join(timeout=1.0)
            _thr = None
        if _scan_thr is not None:
            _scan_thr.join(timeout=1.0)
            _scan_thr = None
        if _cap is not None:
            _cap.release()
            _cap = None
def _click(img):
    bx, pts = [], []
    win = "Calibrate Slots | 2 clicks per slot | u undo | q done"
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
            cv.putText(v, f"slot_{i + 1}", (x1, max(y1 - 6, 12)),
                       cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
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
    global _boxes, _ref
    print("[TRAY] Keep tray empty and arm out of view.")
    img = _grab(1.5, 5)
    if img is None:
        return False
    bx = _click(img)
    if not bx:
        print("[TRAY] No slots drawn. Nothing saved.")
        return False
    print(f"[TRAY] Reference capture in {REF_COUNTDOWN}s — keep tray empty.")
    win_ref = "Reference preview — clear tray then wait"
    cv.namedWindow(win_ref, cv.WINDOW_NORMAL)
    deadline = time.time() + REF_COUNTDOWN
    while time.time() < deadline:
        remaining = int(deadline - time.time()) + 1
        with _fl:
            live = _last
        if live is not None:
            preview = live.copy()
            for i, (x1, y1, x2, y2) in enumerate(bx):
                cv.rectangle(preview, (x1, y1), (x2, y2), (0, 220, 60), 2)
                cv.putText(preview, f"slot_{i+1}", (x1, max(y1 - 6, 12)),
                           cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv.putText(preview, f"Capturing reference in {remaining}s ...",
                       (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 60, 255), 2)
            cv.imshow(win_ref, preview)
        cv.waitKey(100)
    cv.destroyWindow(win_ref)
    cv.waitKey(1)
    print(f"[TRAY] Capturing reference ({REF_FRAMES} frames averaged)...")
    ref = _grab(1.5, REF_FRAMES)
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
        _boxes, _ref = cfg, ref
        _st.clear()
        _cam.clear()
    print(f"[TRAY] Saved {len(cfg)} slot(s) + reference image.")
    return True
