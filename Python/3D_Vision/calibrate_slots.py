import os
import json
import cv2 as cv

s: list[tuple[int, int, int, int]] = []
pts: list[tuple[int, int]] = []
img = None
win = "Tray Slot Calibration"

COLORS = [
    (0, 255, 0),
    (0, 165, 255),
    (255, 0, 0),
    (0, 0, 255),
    (255, 0, 255),
    (0, 255, 255),
]


def clr(idx: int) -> tuple:
    return COLORS[idx % len(COLORS)]


def update_title():
    n = len(s) + 1
    msg = f"Slot {n} - click FIRST corner" if len(pts) == 0 else f"Slot {n} - click OPPOSITE corner"
    cv.setWindowTitle(win, msg)


def on_click(evt, x, y, flags, param):
    if evt != cv.EVENT_LBUTTONDOWN:
        return
    pts.append((x, y))
    c = clr(len(s))
    cv.circle(img, (x, y), 8, c, -1)
    cv.circle(img, (x, y), 9, (255, 255, 255), 2)
    cv.imshow(win, img)

    if len(pts) == 2:
        (x1, y1), (x2, y2) = pts
        x_min, x_max = min(x1, x2), max(x1, x2)
        y_min, y_max = min(y1, y2), max(y1, y2)
        s.append((x_min, y_min, x_max, y_max))
        cv.rectangle(img, (x_min, y_min), (x_max, y_max), c, 3)
        lbl = f"slot_{len(s)}"
        (tw, th), _ = cv.getTextSize(lbl, cv.FONT_HERSHEY_SIMPLEX, 1.2, 2)
        cv.rectangle(img, (x_min, y_min - th - 10), (x_min + tw + 8, y_min), c, -1)
        cv.putText(img, lbl, (x_min + 4, y_min - 6), cv.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        cv.imshow(win, img)
        pts.clear()

    update_title()


def resolve_path(p: str) -> str:
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


if __name__ == "__main__":
    from tray_camera import USE_CAM, get_frame

    if USE_CAM:
        src = get_frame()
        ref_p = resolve_path("test_images/empty_tray.jpg")
        cv.imwrite(ref_p, src)
    else:
        p = resolve_path("test_images/empty_tray.jpg")
        src = cv.imread(p)
    if src is None:
        raise SystemExit(1)

    h, w = src.shape[:2]
    scale = min(1.0, 1600 / w)
    img = cv.resize(src, (int(w * scale), int(h * scale))) if scale < 1.0 else src.copy()

    cv.namedWindow(win, cv.WINDOW_NORMAL)
    cv.imshow(win, img)
    update_title()
    cv.setMouseCallback(win, on_click)
    cv.waitKey(0)
    cv.destroyAllWindows()

    if scale < 1.0:
        s[:] = [(int(x1 / scale), int(y1 / scale), int(x2 / scale), int(y2 / scale)) for x1, y1, x2, y2 in s]

    if s:
        cfg = {f"slot_{i}": list(b) for i, b in enumerate(s, 1)}
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "slot_config.json")
        with open(out, "w") as f:
            json.dump(cfg, f, indent=4)
        print(f"Saved {len(cfg)} slot(s) to {out}")