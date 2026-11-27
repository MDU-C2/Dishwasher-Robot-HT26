"""Render pure real-device 2x3 comparison grids (Ground Truth + 5 models' raw OAK-D Pro outputs).

No synthetic FN boxes and no CLS:/FP: diagnostic prefixes are drawn on the 5 model panels.
Each model panel shows ONLY what that model actually detected on the OAK-D Pro above --conf,
using a consistent per-class color palette shared with the Ground Truth panel.
"""

from __future__ import annotations

import argparse
import html
import json
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_metrics as em  # noqa: E402

PAPER5_ORDER = [
    ("marc_v5s_640", "1. MARC-v5s-640"),
    ("ft_marc_v5s_416", "2. FT-MARC-v5s-416"),
    ("yolov5s_416", "3. YOLOv5s-416"),
    ("yolo11n_416", "4. YOLO11n-416"),
    ("yolo11n_640", "5. YOLO11n-640"),
]

# Consistent per-class BGR color palette (shared by GT and all 5 models)
CLASS_COLORS_BGR: dict[str, tuple[int, int, int]] = {
    "Back": (60, 140, 255),        # orange
    "Front": (255, 190, 40),       # sky blue / cyan-blue
    "left_side": (235, 95, 220),   # magenta-pink
    "right_side": (60, 225, 240),  # yellow
    "upright": (60, 215, 70),      # green
    "upside_down": (85, 85, 255),  # red-coral
    "Gripper": (180, 180, 180),    # light gray (legacy auxiliary)
    "handle": (210, 160, 120),     # muted slate (legacy auxiliary)
}


def draw_label(img: np.ndarray, text: str, x: int, y: int, color: tuple[int, int, int]) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.44
    (tw, th), baseline = cv2.getTextSize(text, font, scale, 1)
    y_top = max(th + 4, y - 4)
    cv2.rectangle(img, (x, y_top - th - 3), (min(img.shape[1] - 1, x + tw + 4), y_top + 2), (18, 20, 24), -1)
    cv2.putText(img, text, (x + 2, y_top - 1), font, scale, color, 1, cv2.LINE_AA)


def render_gt_panel(
    bgr: np.ndarray,
    gt_boxes: np.ndarray,
    gt_classes: np.ndarray,
    subset: str,
    panel_w: int = 640,
    panel_h: int = 360,
) -> np.ndarray:
    oh, ow = bgr.shape[:2]
    sx, sy = panel_w / ow, panel_h / oh
    img = cv2.resize(bgr, (panel_w, panel_h), interpolation=cv2.INTER_AREA)
    for box, cid in zip(gt_boxes, gt_classes):
        lbl = em.CANONICAL_LABELS[int(cid)]
        col = CLASS_COLORS_BGR.get(lbl, (80, 220, 80))
        x1, y1, x2, y2 = int(box[0] * sx), int(box[1] * sy), int(box[2] * sx), int(box[3] * sy)
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        draw_label(img, lbl, x1, y1, col)
    hdr = np.full((34, panel_w, 3), (28, 34, 44), dtype=np.uint8)
    cv2.putText(hdr, f"0. Ground Truth ({subset})", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (250, 250, 250), 1, cv2.LINE_AA)
    cv2.putText(hdr, f"GT: {len(gt_boxes)}", (panel_w - 95, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 235, 190), 1, cv2.LINE_AA)
    return np.vstack([hdr, img])


def render_raw_model_panel(
    bgr: np.ndarray,
    title: str,
    dets: list[dict[str, Any]],
    conf_thresh: float,
    panel_w: int = 640,
    panel_h: int = 360,
) -> np.ndarray:
    oh, ow = bgr.shape[:2]
    sx, sy = panel_w / ow, panel_h / oh
    img = cv2.resize(bgr, (panel_w, panel_h), interpolation=cv2.INTER_AREA)
    shown = [d for d in dets if float(d["conf"]) >= conf_thresh]
    shown.sort(key=lambda d: float(d["conf"]))
    for d in shown:
        lbl = str(d["label"])
        col = CLASS_COLORS_BGR.get(lbl, (220, 220, 220))
        x1, y1, x2, y2 = [
            int(d["xyxy"][0] * sx),
            int(d["xyxy"][1] * sy),
            int(d["xyxy"][2] * sx),
            int(d["xyxy"][3] * sy),
        ]
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        draw_label(img, f"{lbl} {float(d['conf']):.2f}", x1, y1, col)
    hdr = np.full((34, panel_w, 3), (32, 34, 40), dtype=np.uint8)
    cv2.putText(hdr, title, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (245, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(hdr, f"Det: {len(shown)}", (panel_w - 95, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (190, 220, 255), 1, cv2.LINE_AA)
    return np.vstack([hdr, img])


def render_all(dataset: Path, preds_dir: Path, out_dir: Path, preprocess: str, conf_thresh: float) -> None:
    gt_by_image, _ = em.load_ground_truth(dataset)
    preds_by_model: dict[str, dict[str, list[dict[str, Any]]]] = {}
    for mid, _ in PAPER5_ORDER:
        _, pbi, _ = em.load_predictions(preds_dir / f"{mid}_oak_{preprocess}.jsonl")
        preds_by_model[mid] = pbi

    overlays_dir = out_dir / "overlays"
    overlays_dir.mkdir(parents=True, exist_ok=True)

    # Order cleanly: negative (8), multi (38), single (54), sorted by timestamp within each
    ordered = []
    for img_name, gt in sorted(gt_by_image.items()):
        n = len(gt["boxes"])
        sub = "negative" if n == 0 else ("single" if n == 1 else "multi")
        pri = 0 if sub == "negative" else (1 if sub == "multi" else 2)
        ordered.append((pri, img_name, sub, n))
    ordered.sort()

    cards = []
    for idx, (_, img_name, sub, gt_n) in enumerate(ordered, 1):
        bgr = cv2.imread(str(dataset / "images" / img_name))
        if bgr is None:
            continue
        gt = gt_by_image[img_name]
        panels = [render_gt_panel(bgr, gt["boxes"], gt["classes"], sub)]
        counts_txt = []
        for mid, title in PAPER5_ORDER:
            dets = preds_by_model[mid].get(img_name, [])
            n_det = sum(1 for d in dets if float(d["conf"]) >= conf_thresh)
            counts_txt.append(f"{title.split(' ', 1)[1]}:{n_det}")
            panels.append(render_raw_model_panel(bgr, title, dets, conf_thresh))
        grid = np.vstack([np.hstack(panels[0:3]), np.hstack(panels[3:6])])
        stem_short = img_name.split("_jpg.rf.")[0]
        fname = f"{idx:03d}_{sub}_{stem_short}.jpg"
        cv2.imwrite(str(overlays_dir / fname), grid, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        cards.append(
            f"<div class='grid-card' data-subset='{sub}'>"
            f"<div class='meta'><b>#{idx:03d} {html.escape(stem_short)}</b> | "
            f"<span class='tag {sub}'>{sub}</span> | GT={gt_n} | {' · '.join(counts_txt)}</div>"
            f"<a href='overlays/{fname}' target='_blank'><img src='overlays/{fname}' loading='lazy'></a>"
            f"</div>"
        )

    legend_items = []
    for k, (b, g, r) in CLASS_COLORS_BGR.items():
        legend_items.append(f"<span class='leg'><i style='background:rgb({r},{g},{b})'></i>{k}</span>")

    page = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>OAK-D Pro Raw On-Device Detection Comparison ({preprocess}, conf={conf_thresh:.2f})</title>
<style>
  body {{ font-family: "Segoe UI", system-ui, sans-serif; margin: 22px auto; max-width: 1560px; background: #f6f8fa; color: #1f2328; }}
  .bar {{ background: #fff; border: 1px solid #d0d7de; border-radius: 10px; padding: 14px 18px; margin-bottom: 18px; }}
  .leg {{ display: inline-flex; align-items: center; margin-right: 14px; font-size: 13px; }}
  .leg i {{ display: inline-block; width: 12px; height: 12px; border-radius: 3px; margin-right: 5px; }}
  .filters button {{ margin-right: 8px; padding: 6px 14px; border: 1px solid #c9d1d9; border-radius: 6px; background: #fff; cursor: pointer; }}
  .filters button.active {{ background: #1f6feb; color: #fff; border-color: #1f6feb; }}
  .grid-card {{ background: #fff; border: 1px solid #d0d7de; border-radius: 8px; padding: 12px; margin-bottom: 16px; }}
  .meta {{ font-size: 13px; color: #444; margin-bottom: 8px; }}
  img {{ max-width: 100%; border-radius: 4px; }}
  .tag {{ display: inline-block; padding: 2px 7px; border-radius: 10px; font-size: 12px; font-weight: 600; }}
  .tag.single {{ background: #ddf4ff; color: #0969da; }}
  .tag.multi {{ background: #fff8c5; color: #9a6700; }}
  .tag.negative {{ background: #ffebe9; color: #cf222e; }}
</style>
</head>
<body>
<div class="bar">
  <h2 style="margin:0 0 8px 0;">OAK-D Pro 100-Image Test Set — Raw On-Device Detection Comparison (<code>{preprocess}</code>, <code>conf &ge; {conf_thresh:.2f}</code>)</h2>
  <p style="margin:0 0 10px 0;font-size:13px;color:#555;">Panel 0 shows manual Ground Truth; Panels 1–5 <b>render only the raw bounding boxes and confidence scores emitted on OAK-D Pro</b> (no FN overlays or CLS/FP diagnostic tags).</p>
  <div style="margin-bottom:10px;">{"".join(legend_items)}</div>
  <div class="filters">
    <button class="active" onclick="flt('all', this)">All (100)</button>
    <button onclick="flt('negative', this)">Pure Negative (8)</button>
    <button onclick="flt('multi', this)">Multi-Cup (38)</button>
    <button onclick="flt('single', this)">Single-Cup (54)</button>
  </div>
</div>
{"".join(cards)}
<script>
function flt(mode, btn) {{
  document.querySelectorAll('.filters button').forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
  document.querySelectorAll('.grid-card').forEach(el => {{
    el.style.display = (mode === 'all' || el.dataset.subset === mode) ? '' : 'none';
  }});
}}
</script>
</body>
</html>
"""
    (out_dir / "report_raw.html").write_text(page, encoding="utf-8")
    print(f"[SAVED] {len(cards)} pure real-device grids -> {out_dir / 'report_raw.html'}")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--preds-dir", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--preprocess", default="stretch")
    p.add_argument("--conf", type=float, default=0.50)
    args = p.parse_args()
    render_all(args.dataset, args.preds_dir, args.out_dir, args.preprocess, args.conf)


if __name__ == "__main__":
    main()
