"""Unified 6-class canonical detection evaluator for Dishwasher 3D YOLO models.

Pure NumPy + OpenCV implementation (runs directly inside the project's .venv
without requiring PyTorch, Ultralytics, or Matplotlib).

Inputs:
  - registry.json (for canonical label mapping & model metadata)
  - Ground-truth dataset directory:
      <dataset_dir>/images/*.(png|jpg|jpeg)
      <dataset_dir>/labels/<stem>.txt   (YOLO format: class cx cy w h; 0-byte = negative image)
      Optional: <dataset_dir>/metadata/<stem>.json (from independent-vision-test/capture_rgb.py)
  - Predictions JSONL (one JSON line per image, produced by eval_oak_blob.py or eval_pt_onnx.py):
      {
        "model_id": "...",
        "runtime": "oak_blob_fp16" | "pt_fp32" | "onnx",
        "preprocess": "letterbox" | "stretch" | "center_crop",
        "image": "scene_001.png",
        "width": 1920,
        "height": 1080,
        "latency_ms": 18.4,
        "detections": [
          {"class_id": 0, "label": "Back", "conf": 0.92, "xyxy": [x1, y1, x2, y2]}
        ]
      }

Outputs:
  - Per-model metrics JSON (mAP50, mAP50-95, per-class AP/P/R/F1, empty-scene FPIR,
    auxiliary-class audit counts, latency summary, optional subgroup metrics)
  - Confusion matrix CSV & PNG (6 canonical cup classes + background)
  - Multi-model comparison CSV, Markdown, and LaTeX (booktabs) tables
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from collections import defaultdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np

CANONICAL_LABELS = [
    "Back",
    "Front",
    "left_side",
    "right_side",
    "upright",
    "upside_down",
]
IOU_THRESHOLDS = np.linspace(0.50, 0.95, 10)
IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}


def box_iou_matrix(boxes_a: np.ndarray, boxes_b: np.ndarray) -> np.ndarray:
    """Compute pairwise IoU between boxes_a (N,4) and boxes_b (M,4) in xyxy."""
    if len(boxes_a) == 0 or len(boxes_b) == 0:
        return np.zeros((len(boxes_a), len(boxes_b)), dtype=np.float64)
    ax1, ay1, ax2, ay2 = boxes_a[:, 0:1], boxes_a[:, 1:2], boxes_a[:, 2:3], boxes_a[:, 3:4]
    bx1, by1, bx2, by2 = boxes_b[:, 0], boxes_b[:, 1], boxes_b[:, 2], boxes_b[:, 3]
    inter_w = np.maximum(0.0, np.minimum(ax2, bx2) - np.maximum(ax1, bx1))
    inter_h = np.maximum(0.0, np.minimum(ay2, by2) - np.maximum(ay1, by1))
    inter = inter_w * inter_h
    area_a = np.maximum(0.0, (ax2 - ax1) * (ay2 - ay1))
    area_b = np.maximum(0.0, (bx2 - bx1) * (by2 - by1))
    union = area_a + area_b - inter
    return np.where(union > 0.0, inter / union, 0.0)


def compute_ap_101(recalls: np.ndarray, precisions: np.ndarray) -> float:
    """Exact COCO pycocotools 101-point interpolated AP."""
    if len(recalls) == 0:
        return 0.0
    # Precision envelope: p_env[k] = max_{j >= k} precisions[j]
    p_env = np.flip(np.maximum.accumulate(np.flip(precisions)))
    r_thresholds = np.linspace(0.0, 1.0, 101)
    idxs = np.searchsorted(recalls, r_thresholds, side="left")
    ap_vals = np.where(idxs < len(p_env), p_env[np.minimum(idxs, len(p_env) - 1)], 0.0)
    return float(np.mean(ap_vals))


def load_ground_truth(dataset_dir: Path) -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    """Load images, YOLO labels, and optional metadata from dataset_dir."""
    img_dir = dataset_dir / "images"
    lbl_dir = dataset_dir / "labels"
    meta_dir = dataset_dir / "metadata"
    if not img_dir.is_dir():
        raise FileNotFoundError(f"Expected images/ directory inside {dataset_dir}")

    gt_by_image: dict[str, dict[str, Any]] = {}
    meta_by_image: dict[str, dict[str, Any]] = {}

    for img_path in sorted(img_dir.iterdir()):
        if img_path.suffix.lower() not in IMAGE_EXTS:
            continue
        im = cv2.imread(str(img_path))
        if im is None:
            raise RuntimeError(f"Failed to read image: {img_path}")
        h, w = im.shape[:2]
        lbl_path = lbl_dir / f"{img_path.stem}.txt"
        if not lbl_path.exists():
            raise FileNotFoundError(
                f"Missing label file {lbl_path}. Negative images must have an explicit 0-byte .txt file."
            )
        boxes = []
        classes = []
        with open(lbl_path, "r", encoding="utf-8") as f:
            for line_no, raw in enumerate(f, 1):
                line = raw.strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) < 5:
                    raise ValueError(f"Malformed YOLO label {lbl_path}:{line_no}: {raw!r}")
                cid = int(float(parts[0]))
                cx, cy, bw, bh = map(float, parts[1:5])
                if cid < 0 or cid >= len(CANONICAL_LABELS):
                    raise ValueError(f"Out-of-range canonical class id {cid} in {lbl_path}:{line_no}")
                x1 = (cx - bw / 2.0) * w
                y1 = (cy - bh / 2.0) * h
                x2 = (cx + bw / 2.0) * w
                y2 = (cy + bh / 2.0) * h
                boxes.append([x1, y1, x2, y2])
                classes.append(cid)

        gt_by_image[img_path.name] = {
            "width": w,
            "height": h,
            "boxes": np.array(boxes, dtype=np.float64).reshape(-1, 4),
            "classes": np.array(classes, dtype=np.int64),
        }

        mpath = meta_dir / f"{img_path.stem}.json"
        if mpath.exists():
            with open(mpath, "r", encoding="utf-8") as mf:
                meta_by_image[img_path.name] = json.load(mf)

    return gt_by_image, meta_by_image


def load_predictions(pred_jsonl: Path) -> tuple[dict[str, Any], dict[str, list[dict[str, Any]]], list[float]]:
    """Load prediction records from a JSONL file."""
    preds_by_image: dict[str, list[dict[str, Any]]] = {}
    latencies: list[float] = []
    header: dict[str, Any] = {}
    with open(pred_jsonl, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if not header:
                header = {
                    "model_id": rec.get("model_id", pred_jsonl.stem),
                    "runtime": rec.get("runtime", "unknown"),
                    "preprocess": rec.get("preprocess", "unknown"),
                }
            img_name = os.path.basename(rec["image"])
            preds_by_image[img_name] = rec.get("detections", [])
            if rec.get("latency_ms") is not None:
                latencies.append(float(rec["latency_ms"]))
    return header, preds_by_image, latencies


def map_detection_class(
    det: dict[str, Any],
    id_map: dict[str, int | None],
    source_labels: list[str],
) -> tuple[int | None, str]:
    """Map raw model class_id/label to canonical 6-class ID or None if auxiliary."""
    raw_id = det.get("class_id")
    if raw_id is not None and str(raw_id) in id_map:
        cid = id_map[str(raw_id)]
        raw_name = source_labels[int(raw_id)] if 0 <= int(raw_id) < len(source_labels) else str(raw_id)
        return cid, raw_name
    label = str(det.get("label", ""))
    if label in CANONICAL_LABELS:
        return CANONICAL_LABELS.index(label), label
    return None, label


def evaluate_subset(
    image_names: list[str],
    gt_by_image: dict[str, dict[str, Any]],
    preds_by_image: dict[str, list[dict[str, Any]]],
    id_map: dict[str, int | None],
    source_labels: list[str],
    conf_op: float = 0.25,
    iou_op: float = 0.50,
) -> dict[str, Any]:
    """Evaluate predictions against ground truth on a list of images."""
    n_cls = len(CANONICAL_LABELS)
    # Per-class predictions list: (conf, img_name, xyxy)
    cls_preds: dict[int, list[tuple[float, str, np.ndarray]]] = defaultdict(list)
    # Per-class GT dict: img_name -> (boxes, matched_flags)
    cls_gts: dict[int, dict[str, np.ndarray]] = {c: {} for c in range(n_cls)}
    gt_counts = np.zeros(n_cls, dtype=np.int64)

    aux_audit = defaultdict(int)
    empty_images = 0
    empty_fp_images = 0
    empty_fp_total = 0

    # Confusion matrix: rows = GT (0..5, 6=background), cols = Pred (0..5, 6=background)
    cm = np.zeros((n_cls + 1, n_cls + 1), dtype=np.int64)

    for img_name in image_names:
        gt = gt_by_image[img_name]
        gt_boxes = gt["boxes"]
        gt_classes = gt["classes"]

        for c in range(n_cls):
            mask = gt_classes == c
            if np.any(mask):
                cls_gts[c][img_name] = gt_boxes[mask]
                gt_counts[c] += int(np.sum(mask))

        raw_dets = preds_by_image.get(img_name, [])
        mapped_dets = []
        for d in raw_dets:
            conf = float(d["conf"])
            cid, raw_name = map_detection_class(d, id_map, source_labels)
            if cid is None:
                if conf >= conf_op:
                    aux_audit[raw_name] += 1
                continue
            xyxy = np.array(d["xyxy"], dtype=np.float64)
            cls_preds[cid].append((conf, img_name, xyxy))
            if conf >= conf_op:
                mapped_dets.append((conf, cid, xyxy))

        # Empty-scene false positive metrics at operating point
        if len(gt_classes) == 0:
            empty_images += 1
            if mapped_dets:
                empty_fp_images += 1
                empty_fp_total += len(mapped_dets)

        # Confusion matrix at operating point (greedy IoU matching sorted by conf)
        mapped_dets.sort(key=lambda x: x[0], reverse=True)
        matched_gt = np.zeros(len(gt_classes), dtype=bool)
        if len(gt_boxes) > 0 and len(mapped_dets) > 0:
            pred_boxes = np.stack([d[2] for d in mapped_dets], axis=0)
            ious = box_iou_matrix(pred_boxes, gt_boxes)
            for p_idx, (_, p_cls, _) in enumerate(mapped_dets):
                best_g = -1
                best_iou = iou_op
                for g_idx in range(len(gt_classes)):
                    if matched_gt[g_idx]:
                        continue
                    if ious[p_idx, g_idx] >= best_iou:
                        best_iou = ious[p_idx, g_idx]
                        best_g = g_idx
                if best_g >= 0:
                    matched_gt[best_g] = True
                    cm[gt_classes[best_g], p_cls] += 1
                else:
                    cm[n_cls, p_cls] += 1
        else:
            for _, p_cls, _ in mapped_dets:
                cm[n_cls, p_cls] += 1

        for g_idx, g_cls in enumerate(gt_classes):
            if not matched_gt[g_idx]:
                cm[g_cls, n_cls] += 1

    # Compute AP50 and AP50:95 per class
    per_class: dict[str, dict[str, Any]] = {}
    ap50_list = []
    ap5095_list = []
    p_list = []
    r_list = []
    f1_list = []

    for c, cname in enumerate(CANONICAL_LABELS):
        n_gt = int(gt_counts[c])
        preds_c = sorted(cls_preds[c], key=lambda x: x[0], reverse=True)
        n_pred = len(preds_c)

        # Operating point P/R/F1 from confusion matrix diagonal vs row/col sums
        tp_op = int(cm[c, c])
        fp_op = int(np.sum(cm[:, c]) - tp_op)
        fn_op = int(np.sum(cm[c, :]) - tp_op)
        prec_op = tp_op / (tp_op + fp_op) if (tp_op + fp_op) > 0 else 0.0
        rec_op = tp_op / (tp_op + fn_op) if (tp_op + fn_op) > 0 else 0.0
        f1_op = 2 * prec_op * rec_op / (prec_op + rec_op) if (prec_op + rec_op) > 0 else 0.0

        aps = []
        if n_gt > 0 and n_pred > 0:
            for thr in IOU_THRESHOLDS:
                used = {img: np.zeros(len(bxs), dtype=bool) for img, bxs in cls_gts[c].items()}
                tp = np.zeros(n_pred, dtype=np.float64)
                fp = np.zeros(n_pred, dtype=np.float64)
                for idx, (_, img_name, pbox) in enumerate(preds_c):
                    gboxes = cls_gts[c].get(img_name)
                    if gboxes is None or len(gboxes) == 0:
                        fp[idx] = 1.0
                        continue
                    ious = box_iou_matrix(pbox.reshape(1, 4), gboxes)[0]
                    best_j = int(np.argmax(ious))
                    if ious[best_j] >= thr and not used[img_name][best_j]:
                        used[img_name][best_j] = True
                        tp[idx] = 1.0
                    else:
                        fp[idx] = 1.0
                tp_cum = np.cumsum(tp)
                fp_cum = np.cumsum(fp)
                rec = tp_cum / n_gt
                prec = tp_cum / np.maximum(tp_cum + fp_cum, 1e-12)
                aps.append(compute_ap_101(rec, prec))
            ap50 = aps[0]
            ap5095 = float(np.mean(aps))
        else:
            ap50 = 0.0
            ap5095 = 0.0

        per_class[cname] = {
            "gt_instances": n_gt,
            "pred_total": n_pred,
            "tp_at_op": tp_op,
            "fp_at_op": fp_op,
            "fn_at_op": fn_op,
            "precision": round(prec_op, 4),
            "recall": round(rec_op, 4),
            "f1": round(f1_op, 4),
            "ap50": round(ap50, 4),
            "ap50_95": round(ap5095, 4),
        }
        if n_gt > 0:
            ap50_list.append(ap50)
            ap5095_list.append(ap5095)
            p_list.append(prec_op)
            r_list.append(rec_op)
            f1_list.append(f1_op)

    return {
        "num_images": len(image_names),
        "total_gt_instances": int(np.sum(gt_counts)),
        "operating_point": {"conf": conf_op, "iou": iou_op},
        "macro": {
            "precision": round(float(np.mean(p_list)) if p_list else 0.0, 4),
            "recall": round(float(np.mean(r_list)) if r_list else 0.0, 4),
            "f1": round(float(np.mean(f1_list)) if f1_list else 0.0, 4),
            "mAP50": round(float(np.mean(ap50_list)) if ap50_list else 0.0, 4),
            "mAP50_95": round(float(np.mean(ap5095_list)) if ap5095_list else 0.0, 4),
        },
        "empty_scenes": {
            "num_empty_images": empty_images,
            "fp_images": empty_fp_images,
            "fp_image_rate": round(empty_fp_images / empty_images, 4) if empty_images > 0 else None,
            "fp_total": empty_fp_total,
            "fp_per_empty_image": round(empty_fp_total / empty_images, 4) if empty_images > 0 else None,
        },
        "auxiliary_class_audit": dict(aux_audit),
        "per_class": per_class,
        "confusion_matrix": cm.tolist(),
    }


def save_confusion_matrix_png(cm: list[list[int]], title: str, out_png: Path) -> None:
    """Render a clean 7x7 confusion matrix PNG using OpenCV (no matplotlib required)."""
    labels = CANONICAL_LABELS + ["bg"]
    n = len(labels)
    cell = 78
    margin_left = 140
    margin_top = 110
    margin_right = 40
    margin_bottom = 80
    w = margin_left + n * cell + margin_right
    h = margin_top + n * cell + margin_bottom
    canvas = np.full((h, w, 3), 255, dtype=np.uint8)

    cv2.putText(canvas, title, (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(canvas, "Rows: Ground Truth | Cols: Predicted", (20, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (90, 90, 90), 1, cv2.LINE_AA)

    arr = np.array(cm, dtype=np.int64)
    row_sums = np.maximum(arr.sum(axis=1, keepdims=True), 1)
    norm = arr / row_sums

    for r in range(n):
        for c in range(n):
            x1 = margin_left + c * cell
            y1 = margin_top + r * cell
            x2 = x1 + cell
            y2 = y1 + cell
            val = int(arr[r, c])
            ratio = float(norm[r, c])
            # Blue heat shading
            blue = int(255 - 15 * ratio)
            green = int(255 - 140 * ratio)
            red = int(255 - 190 * ratio)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (blue, green, red), -1)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (210, 210, 210), 1)
            txt_color = (255, 255, 255) if ratio > 0.55 else (25, 25, 25)
            txt = str(val)
            (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.putText(canvas, txt, (x1 + (cell - tw) // 2, y1 + (cell + th) // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, txt_color, 2, cv2.LINE_AA)

        # Row label
        cv2.putText(canvas, labels[r], (14, margin_top + r * cell + cell // 2 + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (30, 30, 30), 1, cv2.LINE_AA)

    for c in range(n):
        lbl = labels[c][:10]
        cv2.putText(canvas, lbl, (margin_left + c * cell + 4, margin_top - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 30, 30), 1, cv2.LINE_AA)

    out_png.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_png), canvas)


def write_comparison_tables(results: list[dict[str, Any]], out_dir: Path) -> None:
    """Write CSV, Markdown, and LaTeX booktabs comparison tables."""
    out_dir.mkdir(parents=True, exist_ok=True)

    csv_path = out_dir / "eval_comparison.csv"
    fields = [
        "model_id",
        "runtime",
        "preprocess",
        "num_images",
        "precision",
        "recall",
        "f1",
        "mAP50",
        "mAP50_95",
        "empty_fp_rate",
        "mean_latency_ms",
        "fps",
    ]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in results:
            ov = r["overall"]
            lat = r.get("latency_summary", {})
            dname = r.get("display_name") or r["model_id"]
            w.writerow(
                {
                    "model_id": dname,
                    "runtime": r["runtime"],
                    "preprocess": r["preprocess"],
                    "num_images": ov["num_images"],
                    "precision": ov["macro"]["precision"],
                    "recall": ov["macro"]["recall"],
                    "f1": ov["macro"]["f1"],
                    "mAP50": ov["macro"]["mAP50"],
                    "mAP50_95": ov["macro"]["mAP50_95"],
                    "empty_fp_rate": ov["empty_scenes"]["fp_image_rate"],
                    "mean_latency_ms": lat.get("mean_ms"),
                    "fps": lat.get("fps"),
                }
            )

    md_lines = [
        "# Unified 6-Class Evaluation Comparison",
        "",
        "| Model | Runtime | Preprocess | P | R | F1 | mAP@50 | mAP@50:95 | Empty FP Rate | Latency (ms) |",
        "| :--- | :--- | :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    tex_lines = [
        r"\begin{table}[t]",
        r"  \centering",
        r"  \caption{Unified 6-class cup orientation detection comparison on the evaluation dataset.}",
        r"  \label{tab:yolo-comparison}",
        r"  \begin{tabular}{lllcccccc}",
        r"    \toprule",
        r"    Model & Runtime & Prep. & P & R & F1 & mAP@50 & mAP@50:95 & Lat. (ms) \\",
        r"    \midrule",
    ]

    for r in results:
        ov = r["overall"]
        m = ov["macro"]
        efp = ov["empty_scenes"]["fp_image_rate"]
        efp_s = f"{efp:.3f}" if efp is not None else "--"
        lat = r.get("latency_summary", {}).get("mean_ms")
        lat_s = f"{lat:.1f}" if lat is not None else "--"
        dname = r.get("display_name") or r["model_id"]
        md_lines.append(
            f"| `{dname}` | {r['runtime']} | {r['preprocess']} | "
            f"{m['precision']:.4f} | {m['recall']:.4f} | {m['f1']:.4f} | "
            f"{m['mAP50']:.4f} | {m['mAP50_95']:.4f} | {efp_s} | {lat_s} |"
        )
        mid_tex = dname.replace("_", r"\_")
        rt_tex = r["runtime"].replace("_", r"\_")
        pp_tex = r["preprocess"].replace("_", r"\_")
        tex_lines.append(
            f"    {mid_tex} & {rt_tex} & {pp_tex} & "
            f"{m['precision']:.3f} & {m['recall']:.3f} & {m['f1']:.3f} & "
            f"{m['mAP50']:.3f} & {m['mAP50_95']:.3f} & {lat_s} \\\\"
        )

    tex_lines.extend([r"    \bottomrule", r"  \end{tabular}", r"\end{table}"])
    (out_dir / "eval_comparison.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    (out_dir / "eval_comparison.tex").write_text("\n".join(tex_lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate YOLO predictions on canonical 6-class labels.")
    parser.add_argument("--registry", type=Path, default=Path(__file__).resolve().parents[1] / "registry.json")
    parser.add_argument("--dataset", type=Path, required=True, help="Dataset dir with images/ and labels/")
    parser.add_argument("--preds", type=Path, nargs="+", required=True, help="One or more prediction JSONL files")
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "reports" / "eval")
    parser.add_argument("--conf", type=float, default=0.25, help="Locked operating confidence threshold for P/R/CM")
    parser.add_argument("--iou", type=float, default=0.50, help="Operating IoU threshold for P/R/CM")
    args = parser.parse_args()

    reg = json.loads(args.registry.read_text(encoding="utf-8"))
    models_by_id = {m["model_id"]: m for m in reg["models"]}
    gt_by_image, meta_by_image = load_ground_truth(args.dataset)
    all_images = sorted(gt_by_image.keys())

    all_results = []
    args.out.mkdir(parents=True, exist_ok=True)

    for pred_path in args.preds:
        header, preds_by_image, latencies = load_predictions(pred_path)
        mid = header["model_id"]
        if mid in models_by_id:
            id_map = models_by_id[mid]["source_id_to_canonical"]
            source_labels = models_by_id[mid]["source_labels"]
        else:
            id_map = {str(i): i for i in range(len(CANONICAL_LABELS))}
            source_labels = list(CANONICAL_LABELS)

        overall = evaluate_subset(all_images, gt_by_image, preds_by_image, id_map, source_labels, args.conf, args.iou)

        # Optional subgroup breakdown (distance_band / scene_group from capture_rgb.py metadata)
        subgroups: dict[str, Any] = {}
        if meta_by_image:
            by_band: dict[str, list[str]] = defaultdict(list)
            by_group: dict[str, list[str]] = defaultdict(list)
            for img_name in all_images:
                m = meta_by_image.get(img_name, {})
                plan = m.get("scene_plan", m)
                if plan.get("distance_band"):
                    by_band[str(plan["distance_band"])].append(img_name)
                if plan.get("scene_group"):
                    by_group[str(plan["scene_group"])].append(img_name)
            for k, imgs in sorted(by_band.items()):
                subgroups[f"distance_band:{k}"] = evaluate_subset(
                    imgs, gt_by_image, preds_by_image, id_map, source_labels, args.conf, args.iou
                )["macro"]
            for k, imgs in sorted(by_group.items()):
                subgroups[f"scene_group:{k}"] = evaluate_subset(
                    imgs, gt_by_image, preds_by_image, id_map, source_labels, args.conf, args.iou
                )["macro"]

        lat_summary = {}
        if latencies:
            mean_ms = float(np.mean(latencies))
            lat_summary = {
                "count": len(latencies),
                "mean_ms": round(mean_ms, 2),
                "p50_ms": round(float(np.percentile(latencies, 50)), 2),
                "p95_ms": round(float(np.percentile(latencies, 95)), 2),
                "fps": round(1000.0 / mean_ms, 2) if mean_ms > 0 else None,
            }

        dname = models_by_id[mid].get("display_name", mid) if mid in models_by_id else mid
        res = {
            "model_id": mid,
            "display_name": dname,
            "runtime": header["runtime"],
            "preprocess": header["preprocess"],
            "predictions_file": str(pred_path),
            "dataset_dir": str(args.dataset),
            "latency_summary": lat_summary,
            "overall": overall,
            "subgroups": subgroups,
        }
        all_results.append(res)

        stem = f"{mid}_{header['runtime']}_{header['preprocess']}"
        (args.out / f"{stem}_metrics.json").write_text(
            json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        save_confusion_matrix_png(
            overall["confusion_matrix"],
            f"{dname} ({header['runtime']}, {header['preprocess']})",
            args.out / f"{stem}_cm.png",
        )
        print(
            f"[OK] {dname:22s} mAP50={overall['macro']['mAP50']:.4f} "
            f"mAP50-95={overall['macro']['mAP50_95']:.4f} "
            f"P={overall['macro']['precision']:.4f} R={overall['macro']['recall']:.4f}"
        )

    write_comparison_tables(all_results, args.out)
    print(f"\nSaved comparison reports to: {args.out}")


if __name__ == "__main__":
    main()
