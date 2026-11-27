"""Offline PyTorch (.pt) inference runner for registered 3D YOLO models.

Produces the EXACT same prediction JSONL schema as eval_oak_blob.py so that
eval_metrics.py can compare FP32 (.pt) and OAK FP16 (.blob) models side-by-side
on the canonical 6-class label space.

Requirements (PC with GPU/CPU or Google Colab; NOT required in OAK .venv):
  - For YOLO11n / YOLOv8n .pt: `pip install ultralytics`
  - For YOLOv5s .pt (legacy YOLOv5 repo checkpoints): requires local yolov5 repo
    or torch.hub (`--yolov5-repo` path to local yolov5 directory)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}


def run_ultralytics_pt(
    pt_path: Path,
    img_paths: list[Path],
    imgsz: int,
    conf: float,
    iou: float,
    device: str,
) -> list[dict[str, Any]]:
    from ultralytics import YOLO

    model = YOLO(str(pt_path))
    names = model.names
    records = []
    for p in img_paths:
        t0 = time.perf_counter()
        res = model.predict(
            source=str(p),
            imgsz=imgsz,
            conf=conf,
            iou=iou,
            device=device,
            verbose=False,
        )[0]
        dt_ms = (time.perf_counter() - t0) * 1000.0
        h, w = res.orig_shape
        dets = []
        if res.boxes is not None and len(res.boxes) > 0:
            xyxys = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            clss = res.boxes.cls.cpu().numpy().astype(int)
            for box, sc, cid in zip(xyxys, confs, clss):
                lbl = names[int(cid)] if isinstance(names, (dict, list)) else str(cid)
                dets.append(
                    {
                        "class_id": int(cid),
                        "label": str(lbl),
                        "conf": round(float(sc), 6),
                        "xyxy": [round(float(x), 2) for x in box.tolist()],
                    }
                )
        records.append(
            {
                "image": p.name,
                "width": int(w),
                "height": int(h),
                "latency_ms": round(dt_ms, 3),
                "detections": dets,
            }
        )
    return records


def run_yolov5_pt(
    pt_path: Path,
    img_paths: list[Path],
    imgsz: int,
    conf: float,
    iou: float,
    device: str,
    yolov5_repo: Path,
) -> list[dict[str, Any]]:
    import cv2
    import numpy as np
    import torch

    repo_str = str(yolov5_repo.resolve())
    if repo_str not in sys.path:
        sys.path.insert(0, repo_str)

    from models.common import DetectMultiBackend
    from utils.augmentations import letterbox
    from utils.general import non_max_suppression, scale_boxes
    from utils.torch_utils import select_device

    dev = select_device(device)
    model = DetectMultiBackend(str(pt_path), device=dev, fp16=False)
    stride, names = model.stride, model.names

    records = []
    for p in img_paths:
        im0 = cv2.imread(str(p))
        if im0 is None:
            raise RuntimeError(f"Failed to read {p}")
        h, w = im0.shape[:2]
        im = letterbox(im0, imgsz, stride=stride, auto=False)[0]
        im = im.transpose((2, 0, 1))[::-1]
        im = np.ascontiguousarray(im)
        tensor = torch.from_numpy(im).to(dev).float() / 255.0
        if tensor.ndim == 3:
            tensor = tensor.unsqueeze(0)

        t0 = time.perf_counter()
        with torch.no_grad():
            pred = model(tensor)
            pred = non_max_suppression(pred, conf_thres=conf, iou_thres=iou, max_det=300)
        dt_ms = (time.perf_counter() - t0) * 1000.0

        dets = []
        det = pred[0]
        if len(det):
            det[:, :4] = scale_boxes(tensor.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, sc, cid in det.cpu().numpy().tolist():
                ci = int(cid)
                lbl = names[ci] if ci in names or (isinstance(names, list) and ci < len(names)) else str(ci)
                dets.append(
                    {
                        "class_id": ci,
                        "label": str(lbl),
                        "conf": round(float(sc), 6),
                        "xyxy": [round(float(x), 2) for x in xyxy],
                    }
                )
        records.append(
            {
                "image": p.name,
                "width": int(w),
                "height": int(h),
                "latency_ms": round(dt_ms, 3),
                "detections": dets,
            }
        )
    return records


def main() -> None:
    parser = argparse.ArgumentParser(description="Run PyTorch (.pt) inference for registered models.")
    parser.add_argument("--registry", type=Path, default=Path(__file__).resolve().parents[1] / "registry.json")
    parser.add_argument("--models", nargs="+", default=["paper5"], help="Model IDs, preset ('paper5', 'ours', ...), or 'all'")
    parser.add_argument("--dataset", type=Path, required=True, help="Dataset dir (with images/) or images dir")
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parents[1] / "reports" / "preds")
    parser.add_argument("--conf", type=float, default=0.001, help="Low conf for full AP ranking")
    parser.add_argument("--iou", type=float, default=0.50, help="NMS IoU threshold")
    parser.add_argument("--device", type=str, default="cpu", help="Torch device: cpu, 0, ...")
    parser.add_argument(
        "--yolov5-repo",
        type=Path,
        default=Path("yolov5"),
        help="Path to local yolov5 repo for legacy YOLOv5s .pt checkpoints",
    )
    args = parser.parse_args()

    root_dir = args.registry.resolve().parent
    reg = json.loads(args.registry.read_text(encoding="utf-8"))
    models = [m for m in reg["models"] if m.get("artifacts", {}).get("pt")]
    presets = reg.get("presets", {})
    if args.models != ["all"]:
        wanted: set[str] = set()
        for token in args.models:
            if token in presets:
                wanted.update(presets[token])
            else:
                wanted.add(token)
        models = [m for m in models if m["model_id"] in wanted]

    img_dir = args.dataset / "images" if (args.dataset / "images").is_dir() else args.dataset
    img_paths = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    for m in models:
        mid = m["model_id"]
        pt_path = root_dir / m["artifacts"]["pt"]
        imgsz = int(m["input_size"][0])
        arch = m["architecture"].lower()
        print(f"[RUN] {mid} ({arch}, imgsz={imgsz}) ...")
        if arch.startswith("yolov5"):
            recs = run_yolov5_pt(pt_path, img_paths, imgsz, args.conf, args.iou, args.device, args.yolov5_repo)
        else:
            recs = run_ultralytics_pt(pt_path, img_paths, imgsz, args.conf, args.iou, args.device)

        out_jsonl = args.out_dir / f"{mid}_pt_letterbox.jsonl"
        with open(out_jsonl, "w", encoding="utf-8") as fout:
            for r in recs:
                r["model_id"] = mid
                r["runtime"] = "pt_fp32"
                r["preprocess"] = "letterbox"
                fout.write(json.dumps(r, ensure_ascii=False) + "\n")
        print(f"[SAVED] {out_jsonl} ({len(recs)} images)")


if __name__ == "__main__":
    main()
