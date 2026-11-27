"""Convert registered .pt models to OAK-D Pro RVC2 .blob via Luxonis tools API.

Uploads a model's .pt checkpoint to https://tools.luxonis.com/upload, extracts
the returned .blob / -simplified.onnx / .json bundle, verifies tensor shapes,
SHAVES, class order, and DepthAI 2.30.0.0 pipeline build, and updates the
model's meta.json and top-level registry.json.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import uuid
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import depthai as dai

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

LUXONIS_URL = "https://tools.luxonis.com/upload"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for ch in iter(lambda: f.read(1 << 20), b""):
            h.update(ch)
    return h.hexdigest()


def luxonis_version_tag(arch: str) -> str:
    low = arch.lower()
    if low.startswith("yolov5"):
        return "v5"
    if low.startswith("yolov8"):
        return "v8"
    if low.startswith("yolo11"):
        return "v11"
    raise ValueError(f"Unsupported architecture for Luxonis converter: {arch}")


def expected_channels_per_head(arch: str, num_classes: int) -> int:
    if arch.lower().startswith("yolov5"):
        return 3 * (5 + num_classes)
    return 5 + num_classes


def convert_single_model(
    root_dir: Path,
    entry: dict[str, Any],
    shaves: int = 5,
    timeout_sec: int = 420,
) -> dict[str, Any]:
    mid = entry["model_id"]
    pt_rel = entry.get("artifacts", {}).get("pt")
    if not pt_rel:
        raise ValueError(f"{mid} has no .pt weight file.")
    pt_path = root_dir / pt_rel
    in_w, in_h = entry["input_size"]
    ver = luxonis_version_tag(entry["architecture"])
    conv_id = str(uuid.uuid4())

    print(f"\n=== Converting {mid} ({entry['architecture']} -> {ver}, {in_w}x{in_h}, {shaves} shaves) ===", flush=True)
    with tempfile.TemporaryDirectory(prefix=f"oak_conv_{mid}_") as td:
        tdir = Path(td)
        out_zip = tdir / f"{mid}_response.zip"
        err_hdr = tdir / "headers.txt"
        cmd = [
            "curl.exe",
            "-sS",
            "--max-time",
            str(timeout_sec),
            "-D",
            str(err_hdr),
            "-o",
            str(out_zip),
            "-w",
            "%{http_code}",
            "-X",
            "POST",
            LUXONIS_URL,
            "-F",
            f"id={conv_id}",
            "-F",
            f"version={ver}",
            "-F",
            f"inputshape={in_w} {in_h}",
            "-F",
            f"nShaves={shaves}",
            "-F",
            "useLegacyFrontend=true",
            "-F",
            "useRVC2=true",
            "-F",
            f"file=@{pt_path}",
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        http_code = proc.stdout.strip()
        if proc.returncode != 0 or http_code != "200":
            body_preview = ""
            if out_zip.exists():
                body_preview = out_zip.read_bytes()[:400].decode("utf-8", errors="ignore")
            raise RuntimeError(
                f"Luxonis conversion failed for {mid}: rc={proc.returncode} http={http_code} "
                f"stderr={proc.stderr.strip()} body={body_preview!r}"
            )

        if not zipfile.is_zipfile(out_zip):
            body_preview = out_zip.read_bytes()[:400].decode("utf-8", errors="ignore")
            raise RuntimeError(f"Response for {mid} is not a valid ZIP: {body_preview!r}")

        ext_dir = tdir / "extracted"
        ext_dir.mkdir()
        with zipfile.ZipFile(out_zip) as zf:
            for member in zf.namelist():
                target = (ext_dir / member).resolve()
                if not target.is_relative_to(ext_dir.resolve()):
                    raise ValueError(f"Unsafe zip member: {member}")
            zf.extractall(ext_dir)

        blobs = list(ext_dir.rglob("*.blob"))
        configs = list(ext_dir.rglob("*.json"))
        onnxs = list(ext_dir.rglob("*.onnx"))
        if len(blobs) != 1 or len(configs) != 1:
            raise RuntimeError(f"Expected 1 blob and 1 json in response, got blobs={blobs}, configs={configs}")

        src_blob = blobs[0]
        src_cfg = configs[0]
        src_onnx = onnxs[0] if onnxs else None

        # Verify config labels and input size
        cfg_data = json.loads(src_cfg.read_text(encoding="utf-8"))
        labels = cfg_data.get("mappings", {}).get("labels", [])
        if labels != entry["source_labels"]:
            raise ValueError(f"{mid}: converted labels {labels} != expected {entry['source_labels']}")
        expected_size_str = f"{in_w}x{in_h}"
        if cfg_data.get("nn_config", {}).get("input_size") != expected_size_str:
            raise ValueError(
                f"{mid}: converted input_size {cfg_data.get('nn_config', {}).get('input_size')} != {expected_size_str}"
            )

        # Verify DepthAI OpenVINO blob & pipeline construction
        blob_obj = dai.OpenVINO.Blob(str(src_blob))
        inputs = {k: list(v.dims) for k, v in blob_obj.networkInputs.items()}
        outputs = {k: list(v.dims) for k, v in blob_obj.networkOutputs.items()}
        in_dims = next(iter(inputs.values()))
        if in_dims[0] != in_w or in_dims[1] != in_h:
            raise ValueError(f"{mid}: blob input {in_dims} != {in_w}x{in_h}")
        if blob_obj.numShaves != shaves:
            raise ValueError(f"{mid}: blob shaves={blob_obj.numShaves} != {shaves}")
        exp_ch = expected_channels_per_head(entry["architecture"], entry["num_classes"])
        for oname, odims in outputs.items():
            if odims[2] != exp_ch:
                raise ValueError(f"{mid}: output {oname} channels={odims[2]} != expected {exp_ch}")

        nn_meta = cfg_data["nn_config"]["NN_specific_metadata"]
        pipeline = dai.Pipeline()
        nn = pipeline.create(dai.node.YoloDetectionNetwork)
        nn.setBlobPath(str(src_blob))
        nn.setNumClasses(int(nn_meta["classes"]))
        nn.setCoordinateSize(int(nn_meta["coordinates"]))
        nn.setAnchors(nn_meta.get("anchors", []))
        nn.setAnchorMasks(nn_meta.get("anchor_masks", {}))
        nn.setIouThreshold(float(nn_meta["iou_threshold"]))
        nn.setConfidenceThreshold(float(nn_meta["confidence_threshold"]))
        nn.setNumInferenceThreads(2)

        # Install into models_integ/models/<mid>/
        mdir = root_dir / "models" / mid
        wdir = mdir / "weights"
        cdir = mdir / "config"
        wdir.mkdir(parents=True, exist_ok=True)
        cdir.mkdir(parents=True, exist_ok=True)

        dst_blob = wdir / f"{mid}_{shaves}shave.blob"
        shutil.copy2(src_blob, dst_blob)
        entry["artifacts"]["blob"] = dst_blob.relative_to(root_dir).as_posix()
        entry["sha256"]["blob"] = sha256_file(dst_blob)
        entry["shaves"] = shaves

        if src_onnx is not None:
            dst_onnx = wdir / f"{mid}.onnx"
            shutil.copy2(src_onnx, dst_onnx)
            entry["artifacts"]["onnx"] = dst_onnx.relative_to(root_dir).as_posix()
            entry["sha256"]["onnx"] = sha256_file(dst_onnx)

        dst_cfg = cdir / "depthai_config.json"
        shutil.copy2(src_cfg, dst_cfg)
        entry["artifacts"]["depthai_config"] = dst_cfg.relative_to(root_dir).as_posix()
        entry["sha256"]["depthai_config"] = sha256_file(dst_cfg)

        verif = {
            "converted_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "service": LUXONIS_URL,
            "conversion_id": conv_id,
            "luxonis_version": ver,
            "source_pt": entry["artifacts"]["pt"],
            "source_pt_sha256": entry["sha256"]["pt"],
            "blob": entry["artifacts"]["blob"],
            "blob_sha256": entry["sha256"]["blob"],
            "onnx": entry["artifacts"].get("onnx"),
            "onnx_sha256": entry["sha256"].get("onnx"),
            "depthai_config": entry["artifacts"]["depthai_config"],
            "depthai_version": dai.__version__,
            "openvino": str(blob_obj.version),
            "shaves": blob_obj.numShaves,
            "cmx_slices": blob_obj.numSlices,
            "inputs": inputs,
            "outputs": outputs,
            "labels": labels,
            "host_pipeline_check": "passed",
            "hardware_inference_tested": False,
        }
        verif_path = cdir / "conversion_verification.json"
        verif_path.write_text(json.dumps(verif, indent=2, ensure_ascii=False), encoding="utf-8")
        if "conversion_verification.json" not in entry.get("extra_configs", []):
            entry.setdefault("extra_configs", []).append("conversion_verification.json")
        entry["deploy_preprocess_note"] = (
            f"Compiled to {shaves}-SHAVE RVC2 OpenVINO 2022.1 FP16 blob via Luxonis tools; hardware inference pending"
        )
        (mdir / "meta.json").write_text(json.dumps(entry, indent=2, ensure_ascii=False), encoding="utf-8")

        print(
            f"[OK] {mid} -> {dst_blob.name} ({dst_blob.stat().st_size / 1024 / 1024:.2f} MB, "
            f"shaves={blob_obj.numShaves}, sha256={entry['sha256']['blob'][:12]})",
            flush=True,
        )
        return entry


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert registered .pt models to OAK RVC2 .blob.")
    parser.add_argument("--registry", type=Path, default=Path(__file__).resolve().parents[1] / "registry.json")
    parser.add_argument(
        "--models",
        nargs="+",
        default=["missing"],
        help="Model IDs to convert, or 'missing' for all registered models with .pt but no .blob",
    )
    parser.add_argument("--shaves", type=int, default=5, help="Number of SHAVES (default: 5)")
    args = parser.parse_args()

    root_dir = args.registry.resolve().parent
    reg = json.loads(args.registry.read_text(encoding="utf-8"))

    targets = []
    for m in reg["models"]:
        if args.models == ["missing"]:
            if m.get("artifacts", {}).get("pt") and not m.get("artifacts", {}).get("blob"):
                targets.append(m)
        elif m["model_id"] in set(args.models):
            targets.append(m)

    if not targets:
        print("No models need conversion.")
        return

    print(f"Converting {len(targets)} model(s): {[m['model_id'] for m in targets]}", flush=True)
    failed = []
    for m in targets:
        try:
            convert_single_model(root_dir, m, shaves=args.shaves)
            # Save registry incrementally after each successful conversion
            reg["generated_at_utc"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
            args.registry.write_text(json.dumps(reg, indent=2, ensure_ascii=False), encoding="utf-8")
        except Exception as exc:
            print(f"[FAILED] {m['model_id']}: {exc}", flush=True)
            failed.append((m["model_id"], str(exc)))

    if failed:
        print(f"\nCompleted with {len(failed)} failure(s):")
        for mid, msg in failed:
            print(f"  - {mid}: {msg}")
        raise SystemExit(1)
    print("\nAll requested conversions completed and registry updated!")


if __name__ == "__main__":
    main()
