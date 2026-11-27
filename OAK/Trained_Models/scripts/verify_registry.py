"""Verify integrity, SHA256 hashes, DepthAI blob metadata, and canonical class maps."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def expected_channels_per_head(arch: str, num_classes: int) -> int:
    if arch.lower().startswith("yolov5"):
        return 3 * (5 + num_classes)
    # YOLOv8 / YOLO11 RVC2 yolov6r2 3-head split: 4 bbox + 1 obj/dummy + nc
    return 5 + num_classes


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify models_integ registry and blob compatibility.")
    parser.add_argument("--registry", type=Path, default=Path(__file__).resolve().parents[1] / "registry.json")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "reports" / "registry_verification.json",
    )
    args = parser.parse_args()

    root_dir = args.registry.resolve().parent
    reg = json.loads(args.registry.read_text(encoding="utf-8"))

    try:
        import depthai as dai
    except ImportError:
        dai = None

    canonical = reg["canonical_labels"]
    checks: list[dict[str, Any]] = []
    errors: list[str] = []

    for m in reg["models"]:
        mid = m["model_id"]
        rec: dict[str, Any] = {"model_id": mid, "status": "ok", "checks": {}}

        # 1. Verify canonical mapping
        src_labels = m["source_labels"]
        id_map = m["source_id_to_canonical"]
        if len(src_labels) != m["num_classes"]:
            errors.append(f"{mid}: len(source_labels)={len(src_labels)} != num_classes={m['num_classes']}")
        mapped_targets = set()
        for sid_str, cid in id_map.items():
            sid = int(sid_str)
            if cid is not None:
                if src_labels[sid] != canonical[cid]:
                    errors.append(
                        f"{mid}: mapping mismatch sid={sid} ({src_labels[sid]}) != canonical[{cid}] ({canonical[cid]})"
                    )
                mapped_targets.add(cid)
        if mapped_targets != set(range(len(canonical))):
            errors.append(f"{mid}: canonical targets incomplete: {sorted(mapped_targets)}")
        rec["checks"]["label_map"] = "ok"

        # 2. Verify every declared artifact exists and matches recorded SHA256
        hashes = m.get("sha256", {})
        for key, rel in m.get("artifacts", {}).items():
            if not rel:
                continue
            full = root_dir / rel
            if not full.exists():
                errors.append(f"{mid}: missing artifact {key}: {rel}")
                rec["checks"][key] = "MISSING"
                continue
            actual = sha256_file(full)
            expected = hashes.get(key)
            if expected and actual != expected:
                errors.append(f"{mid}: SHA256 mismatch for {key}: expected {expected[:12]}, got {actual[:12]}")
                rec["checks"][key] = "SHA256_MISMATCH"
            else:
                rec["checks"][key] = f"ok ({actual[:12]})"

        # 3. Verify DepthAI blob shapes and pipeline construction
        blob_rel = m.get("artifacts", {}).get("blob")
        if blob_rel and dai is not None:
            blob_path = root_dir / blob_rel
            b = dai.OpenVINO.Blob(str(blob_path))
            in_dims = list(next(iter(b.networkInputs.values())).dims)
            out_dims = {k: list(v.dims) for k, v in b.networkOutputs.items()}
            exp_w, exp_h = m["input_size"]
            if in_dims[0] != exp_w or in_dims[1] != exp_h:
                errors.append(f"{mid}: blob input {in_dims[:2]} != registry {[exp_w, exp_h]}")
            exp_ch = expected_channels_per_head(m["architecture"], m["num_classes"])
            for oname, odims in out_dims.items():
                if odims[2] != exp_ch:
                    errors.append(f"{mid}: blob output {oname} channels={odims[2]} != expected {exp_ch}")
            rec["checks"]["depthai_blob"] = {
                "openvino": str(b.version),
                "shaves": b.numShaves,
                "cmx_slices": b.numSlices,
                "inputs": in_dims,
                "outputs": out_dims,
            }

        if any(e.startswith(f"{mid}:") for e in errors):
            rec["status"] = "FAIL"
        checks.append(rec)

    summary = {
        "total_models": len(checks),
        "passed": sum(1 for c in checks if c["status"] == "ok"),
        "failed": sum(1 for c in checks if c["status"] != "ok"),
        "depthai_checked": dai is not None,
        "errors": errors,
        "models": checks,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"Verified {summary['total_models']} models: {summary['passed']} passed, {summary['failed']} failed.")
    if errors:
        for e in errors:
            print("  ERROR:", e)
        raise SystemExit(1)
    print(f"Report written to: {args.out}")


if __name__ == "__main__":
    main()
