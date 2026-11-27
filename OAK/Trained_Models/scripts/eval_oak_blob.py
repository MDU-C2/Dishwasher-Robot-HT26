"""Offline-image replay & inference runner for OAK-D Pro (.blob) models in registry.json.

Feeds RGB images from a dataset directory into an OAK-D Pro device over XLinkIn
using DepthAI 2.30.0.0 YoloDetectionNetwork, maps detections back to original
image pixel coordinates, and writes standardized JSONL for eval_metrics.py.

Supports:
  --preprocess letterbox    Aspect-ratio preserving pad (value 114), unified offline benchmark
  --preprocess stretch      Direct resize (matches A1119/camera_setup.py preview stretch)
  --preprocess center_crop  Center square crop + resize (matches legacy v5 720p crop)
  --check-only              Build pipeline and validate blob/config without connecting OAK
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import cv2
import depthai as dai
import numpy as np

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}


def preprocess_image(
    bgr: np.ndarray, input_w: int, input_h: int, mode: str
) -> tuple[np.ndarray, dict[str, float]]:
    """Preprocess BGR image into (input_h, input_w, 3) uint8 + inverse transform metadata."""
    h, w = bgr.shape[:2]
    if mode == "letterbox":
        scale = min(input_w / w, input_h / h)
        new_w = int(round(w * scale))
        new_h = int(round(h * scale))
        resized = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        pad_x = (input_w - new_w) // 2
        pad_y = (input_h - new_h) // 2
        canvas = np.full((input_h, input_w, 3), 114, dtype=np.uint8)
        canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized
        return canvas, {
            "mode": mode,
            "orig_w": float(w),
            "orig_h": float(h),
            "scale": float(scale),
            "pad_x": float(pad_x),
            "pad_y": float(pad_y),
        }
    elif mode == "stretch":
        resized = cv2.resize(bgr, (input_w, input_h), interpolation=cv2.INTER_LINEAR)
        return resized, {"mode": mode, "orig_w": float(w), "orig_h": float(h)}
    elif mode == "center_crop":
        side = min(w, h)
        x0 = (w - side) // 2
        y0 = (h - side) // 2
        cropped = bgr[y0 : y0 + side, x0 : x0 + side]
        resized = cv2.resize(cropped, (input_w, input_h), interpolation=cv2.INTER_LINEAR)
        return resized, {
            "mode": mode,
            "orig_w": float(w),
            "orig_h": float(h),
            "crop_x0": float(x0),
            "crop_y0": float(y0),
            "crop_side": float(side),
        }
    else:
        raise ValueError(f"Unsupported preprocess mode: {mode}")


def unmap_box(
    xmin: float, ymin: float, xmax: float, ymax: float, input_w: int, input_h: int, info: dict[str, Any]
) -> list[float]:
    """Convert normalized [0,1] network box back to original image pixel xyxy."""
    w = info["orig_w"]
    h = info["orig_h"]
    mode = info["mode"]
    if mode == "letterbox":
        px1 = (xmin * input_w - info["pad_x"]) / info["scale"]
        py1 = (ymin * input_h - info["pad_y"]) / info["scale"]
        px2 = (xmax * input_w - info["pad_x"]) / info["scale"]
        py2 = (ymax * input_h - info["pad_y"]) / info["scale"]
    elif mode == "stretch":
        px1 = xmin * w
        py1 = ymin * h
        px2 = xmax * w
        py2 = ymax * h
    elif mode == "center_crop":
        side = info["crop_side"]
        px1 = info["crop_x0"] + xmin * side
        py1 = info["crop_y0"] + ymin * side
        px2 = info["crop_x0"] + xmax * side
        py2 = info["crop_y0"] + ymax * side
    else:
        raise ValueError(mode)
    return [
        round(float(np.clip(px1, 0.0, w)), 2),
        round(float(np.clip(py1, 0.0, h)), 2),
        round(float(np.clip(px2, 0.0, w)), 2),
        round(float(np.clip(py2, 0.0, h)), 2),
    ]


def build_oak_pipeline(
    blob_path: Path,
    nn_meta: dict[str, Any],
    conf_thresh: float,
    iou_thresh: float,
) -> dai.Pipeline:
    """Build XLinkIn -> YoloDetectionNetwork -> XLinkOut pipeline."""
    pipeline = dai.Pipeline()
    xin = pipeline.create(dai.node.XLinkIn)
    xin.setStreamName("in_frame")

    nn = pipeline.create(dai.node.YoloDetectionNetwork)
    nn.setBlobPath(str(blob_path))
    nn.setConfidenceThreshold(float(conf_thresh))
    nn.setIouThreshold(float(iou_thresh))
    nn.setNumClasses(int(nn_meta["classes"]))
    nn.setCoordinateSize(int(nn_meta.get("coordinates", 4)))
    nn.setAnchors(nn_meta.get("anchors", []))
    nn.setAnchorMasks(nn_meta.get("anchor_masks", {}))
    nn.setNumInferenceThreads(2)
    nn.input.setBlocking(True)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("detections")

    xin.out.link(nn.input)
    nn.out.link(xout.input)
    return pipeline


def extract_nn_metadata(model_entry: dict[str, Any], root_dir: Path) -> tuple[Path, dict[str, Any], int, int]:
    """Resolve blob path and DepthAI YOLO metadata for a registered model."""
    blob_rel = model_entry.get("artifacts", {}).get("blob")
    if not blob_rel:
        raise ValueError(f"Model {model_entry['model_id']} has no compiled .blob artifact.")
    blob_path = root_dir / blob_rel

    # Prefer depthai_config JSON if available; otherwise derive from registry metadata
    cfg_rel = model_entry.get("artifacts", {}).get("depthai_config")
    nn_meta: dict[str, Any] = {
        "classes": model_entry["num_classes"],
        "coordinates": 4,
        "anchors": [],
        "anchor_masks": {},
        "iou_threshold": 0.5,
        "confidence_threshold": 0.25,
    }
    if cfg_rel:
        cfg_data = json.loads((root_dir / cfg_rel).read_text(encoding="utf-8"))
        if "nn_config" in cfg_data:
            nn_meta.update(cfg_data["nn_config"].get("NN_specific_metadata", {}))
        elif "anchors" in cfg_data:
            nn_meta["anchors"] = cfg_data.get("anchors", [])
            nn_meta["anchor_masks"] = cfg_data.get("anchor_masks", {})
            nn_meta["iou_threshold"] = cfg_data.get("iou", 0.5)
            nn_meta["confidence_threshold"] = cfg_data.get("confidence", 0.25)

    w, h = model_entry["input_size"]
    return blob_path, nn_meta, int(w), int(h)


def run_model(
    model_entry: dict[str, Any],
    root_dir: Path,
    dataset_dir: Path | None,
    out_jsonl: Path | None,
    preprocess: str,
    conf: float,
    iou: float,
    check_only: bool,
) -> None:
    mid = model_entry["model_id"]
    blob_path, nn_meta, in_w, in_h = extract_nn_metadata(model_entry, root_dir)
    pipeline = build_oak_pipeline(blob_path, nn_meta, conf_thresh=conf, iou_thresh=iou)
    blob_info = dai.OpenVINO.Blob(str(blob_path))
    print(
        f"[PIPELINE OK] {mid} | input={in_w}x{in_h} | shaves={blob_info.numShaves} | "
        f"classes={nn_meta['classes']} | anchors={'yes' if nn_meta['anchors'] else 'anchor-free'}"
    )
    if check_only:
        return

    assert dataset_dir is not None and out_jsonl is not None
    img_dir = dataset_dir / "images" if (dataset_dir / "images").is_dir() else dataset_dir
    img_paths = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    if not img_paths:
        raise RuntimeError(f"No images found in {img_dir}")

    labels = model_entry["source_labels"]
    out_jsonl.parent.mkdir(parents=True, exist_ok=True)

    with dai.Device(pipeline) as device:
        q_in = device.getInputQueue("in_frame", maxSize=4, blocking=True)
        q_det = device.getOutputQueue("detections", maxSize=4, blocking=True)

        with open(out_jsonl, "w", encoding="utf-8") as fout:
            for idx, img_path in enumerate(img_paths, 1):
                bgr = cv2.imread(str(img_path))
                if bgr is None:
                    raise RuntimeError(f"Cannot read {img_path}")
                oh, ow = bgr.shape[:2]
                inp, info = preprocess_image(bgr, in_w, in_h, preprocess)
                planar = inp.transpose(2, 0, 1).flatten()

                frame = dai.ImgFrame()
                frame.setType(dai.ImgFrame.Type.BGR888p)
                frame.setWidth(in_w)
                frame.setHeight(in_h)
                frame.setData(planar)

                t0 = time.perf_counter()
                q_in.send(frame)
                in_det = q_det.get()
                dt_ms = (time.perf_counter() - t0) * 1000.0

                dets = []
                for d in in_det.detections:
                    cid = int(d.label)
                    dets.append(
                        {
                            "class_id": cid,
                            "label": labels[cid] if 0 <= cid < len(labels) else str(cid),
                            "conf": round(float(d.confidence), 6),
                            "xyxy": unmap_box(d.xmin, d.ymin, d.xmax, d.ymax, in_w, in_h, info),
                        }
                    )

                rec = {
                    "model_id": mid,
                    "runtime": "oak_blob_fp16",
                    "preprocess": preprocess,
                    "image": img_path.name,
                    "width": ow,
                    "height": oh,
                    "latency_ms": round(dt_ms, 3),
                    "detections": dets,
                }
                fout.write(json.dumps(rec, ensure_ascii=False) + "\n")
                if idx % 20 == 0 or idx == len(img_paths):
                    print(f"  [{mid}] {idx}/{len(img_paths)} ({dt_ms:.1f} ms)")

    print(f"[SAVED] {out_jsonl}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run OAK-D Pro (.blob) inference for registered models.")
    parser.add_argument("--registry", type=Path, default=Path(__file__).resolve().parents[1] / "registry.json")
    parser.add_argument("--models", nargs="+", default=["paper5"], help="Model IDs, preset ('paper5', 'marc', 'simon', 'ours'), or 'all'")
    parser.add_argument("--dataset", type=Path, help="Dataset dir (or images dir)")
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parents[1] / "reports" / "preds")
    parser.add_argument("--preprocess", choices=["letterbox", "stretch", "center_crop"], default="letterbox")
    parser.add_argument("--conf", type=float, default=0.001, help="Low conf for full AP ranking (default 0.001)")
    parser.add_argument("--iou", type=float, default=0.50, help="NMS IoU threshold")
    parser.add_argument("--check-only", action="store_true", help="Verify pipelines offline without OAK hardware")
    args = parser.parse_args()

    root_dir = args.registry.resolve().parent
    reg = json.loads(args.registry.read_text(encoding="utf-8"))
    models = [m for m in reg["models"] if m.get("artifacts", {}).get("blob")]
    presets = reg.get("presets", {})
    if args.models != ["all"]:
        wanted_list: list[str] = []
        for token in args.models:
            if token in presets:
                wanted_list.extend(presets[token])
            else:
                wanted_list.append(token)
        wanted = set(wanted_list)
        by_id = {m["model_id"]: m for m in models}
        missing = wanted - set(by_id.keys())
        if missing:
            raise ValueError(f"Unknown or non-blob model_ids: {sorted(missing)}")
        # Preserve preset / CLI order
        seen = set()
        ordered = []
        for mid in wanted_list:
            if mid not in seen:
                seen.add(mid)
                ordered.append(by_id[mid])
        models = ordered

    if not args.check_only and not args.dataset:
        parser.error("--dataset is required unless --check-only is set.")

    for m in models:
        out_jsonl = None if args.check_only else args.out_dir / f"{m['model_id']}_oak_{args.preprocess}.jsonl"
        run_model(m, root_dir, args.dataset, out_jsonl, args.preprocess, args.conf, args.iou, args.check_only)


if __name__ == "__main__":
    main()
