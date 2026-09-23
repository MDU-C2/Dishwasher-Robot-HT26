import os, json, time
import cv2 as cv
import numpy as np

USE_CAM = True
CAM_IDX = 1

POS: dict[str, list[float]] = {}
# measure robot coordinates 
def set_slot_positions(positions: dict[str, list[float]]): 
    global POS
    POS.clear()
    POS.update(positions)

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

def load_calibrated_positions(path: str = "slot_positions.json") -> None:
    p = file_path(path)
    if not os.path.exists(p):
        raise FileNotFoundError(f"No calibration file: {p}")
    with open(p, "r") as f:
        data = json.load(f)
    set_slot_positions(data)

def get_frame(warmup_frames: int = 10) -> np.ndarray:
    if USE_CAM:
        for idx in [CAM_IDX]:
            cap = cv.VideoCapture(idx) #catch one image
            if cap.isOpened():
                time.sleep(0.5) #delay för fokus

                for _ in range(warmup_frames):
                    cap.grab()
                ret, f = cap.read()
                cap.release()
                if ret and f is not None:
                    return f
            cap.release()
        raise RuntimeError("Could not take picture")
    p = file_path("test_images/current_state.jpg")
    f = cv.imread(p)
    if f is None:
        raise FileNotFoundError(f"Missing {p}")
    return f

def load_cfg(p: str = "slot_config.json") -> dict[str, list[int]]:
    path = file_path(p)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing {path}")
    with open(path, "r") as f:
        c = json.load(f)
    if not c:
        raise ValueError("Empty config")
    return c

def get_free_slots(thresh: float = 20, cfg_path: str = "slot_config.json", ref_path: str = "test_images/empty_tray.jpg") -> dict[str, str]:
    fp = file_path(ref_path)
    ref = cv.imread(fp)
    if ref is None:
        raise FileNotFoundError(f"Missing {fp}")

    cur = get_frame()
    cfg = load_cfg(cfg_path)
    res = {}

    for name, b in cfg.items():
        x1, y1, x2, y2 = b
        rc = ref[y1:y2, x1:x2]
        cc = cur[y1:y2, x1:x2]

        if rc.shape != cc.shape:
            res[name] = "unknown"
            continue

        diff = cv.absdiff(rc, cc)
        m = float(np.mean(diff))

        print(f"[diff mean: {m:.2f} (threshold: {thresh})")

        res[name] = "occupied" if m > thresh else "free"
    return res

def get_robot_coords_for_slot(name: str) -> list[float]:
    if name not in POS:
        raise KeyError(f"Unkown slot '{name}'. Ensure slot_positions.json is loaded.")
    return list(POS[name])

def verify_placement(name: str, thresh: float = 20, cfg_path: str = "slot_config.json", ref_path: str = "test_images/empty_tray.jpg") -> bool:
    res = get_free_slots(thresh, cfg_path, ref_path)
    return res.get(name) == "occupied"

if __name__ == "__main__":
    try:
        load_calibrated_positions("slot_positions.json")
        print("[INFO] Loaded slot_positions.json.")
    except FileNotFoundError:
        print("[INFO] Running without slot_positions.json loaded "
              "(get_robot_coords_for_slot will fail until it exists).")

    status = get_free_slots()
    print("Slots:", status)

    if "slot_1" in status:
        print("Verify slot_1:", verify_placement("slot_1"))
        if POS:
            try:
                print("Coords slot_1:", get_robot_coords_for_slot("slot_1"))
            except KeyError as e:
                print(e)