"""Generate paper-ready model catalog CSV, Markdown lineage report, and LaTeX booktabs tables."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


def fmt_metric(val: Any, digits: int = 4) -> str:
    if val is None or val == "":
        return "--"
    try:
        return f"{float(val):.{digits}f}"
    except (TypeError, ValueError):
        return str(val)


def build_md_table(models: list[dict[str, Any]]) -> list[str]:
    lines = [
        "| Model ID | Group | Arch | Cls | Input | SHAVE | Dataset | Ep (done/plan) | Artifacts | Val P | Val R | Val mAP@50 | Val mAP@50:95 | Notes |",
        "| :--- | :---: | :--- | :---: | :---: | :---: | :--- | :---: | :--- | ---: | ---: | ---: | ---: | :--- |",
    ]
    for m in models:
        tr = m.get("training", {})
        bm = tr.get("best_val_metrics", {})
        art = m.get("artifacts", {})
        avail = []
        if art.get("pt"):
            avail.append("pt")
        if art.get("onnx"):
            avail.append("onnx")
        if art.get("blob"):
            avail.append("blob")
        if art.get("results_csv"):
            avail.append("csv")
        ep_done = tr.get("epochs_completed")
        ep_plan = tr.get("epochs_planned")
        ep_str = f"{ep_done}/{ep_plan}" if ep_done is not None and ep_plan is not None else (str(ep_done or ep_plan or "--"))
        shave_str = str(m["shaves"]) if m.get("shaves") else "--"
        dname = m.get("display_name") or m["model_id"]
        lines.append(
            f"| `{dname}` | {m.get('source_group', m['generation'])} | {m['architecture']} | {m['num_classes']} | "
            f"{m['input_size'][0]}×{m['input_size'][1]} | {shave_str} | {tr.get('dataset', '--')} | "
            f"{ep_str} | {', '.join(avail)} | {fmt_metric(bm.get('precision'))} | {fmt_metric(bm.get('recall'))} | "
            f"{fmt_metric(bm.get('mAP50'))} | {fmt_metric(bm.get('mAP50_95'))} | {m.get('notes', '')} |"
        )
    return lines


def build_latex_table(models: list[dict[str, Any]], caption: str, label: str) -> str:
    tex = [
        r"\begin{table*}[t]",
        r"  \centering",
        f"  \\caption{{{caption}}}",
        f"  \\label{{{label}}}",
        r"  \small",
        r"  \begin{tabular}{llccclcccc}",
        r"    \toprule",
        r"    Model ID & Arch. & Cls & Input & SHAVE & Dataset & Ep. & Val P & Val R & Val mAP@50 / 50:95 \\",
        r"    \midrule",
    ]
    for m in models:
        tr = m.get("training", {})
        bm = tr.get("best_val_metrics", {})
        dname = m.get("display_name") or m["model_id"]
        mid_tex = dname.replace("_", r"\_")
        ds_tex = str(tr.get("dataset", "--")).replace("_", r"\_")
        ep_done = tr.get("epochs_completed")
        ep_s = str(ep_done) if ep_done is not None else "--"
        shave_s = str(m["shaves"]) if m.get("shaves") else "--"
        m50 = fmt_metric(bm.get("mAP50"), 3)
        m5095 = fmt_metric(bm.get("mAP50_95"), 3)
        map_pair = f"{m50} / {m5095}" if m50 != "--" else "--"
        tex.append(
            f"    {mid_tex} & {m['architecture']} & {m['num_classes']} & "
            f"{m['input_size'][0]} & {shave_s} & {ds_tex} & {ep_s} & "
            f"{fmt_metric(bm.get('precision'), 3)} & {fmt_metric(bm.get('recall'), 3)} & {map_pair} \\\\"
        )
    tex.extend([r"    \bottomrule", r"  \end{tabular}", r"\end{table*}"])
    return "\n".join(tex) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Build paper summary tables from registry.json.")
    parser.add_argument("--registry", type=Path, default=Path(__file__).resolve().parents[1] / "registry.json")
    args = parser.parse_args()

    root_dir = args.registry.resolve().parent
    reg = json.loads(args.registry.read_text(encoding="utf-8"))
    models = reg["models"]
    paper5_models = [m for m in models if m.get("paper_core")]
    archive_models = [m for m in models if not m.get("paper_core")]

    reports_dir = root_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    # 1. model_catalog.csv
    csv_fields = [
        "model_id",
        "paper_core",
        "source_group",
        "architecture",
        "num_classes",
        "input_size",
        "shaves",
        "dataset",
        "init_weights",
        "epochs_completed",
        "epochs_planned",
        "batch_size",
        "trained_date",
        "has_pt",
        "has_onnx",
        "has_blob",
        "has_training_logs",
        "val_best_epoch",
        "val_precision",
        "val_recall",
        "val_mAP50",
        "val_mAP50_95",
        "source_origin",
        "notes",
    ]
    csv_path = root_dir / "model_catalog.csv"
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=csv_fields)
        writer.writeheader()
        for m in models:
            tr = m.get("training", {})
            bm = tr.get("best_val_metrics", {})
            art = m.get("artifacts", {})
            writer.writerow(
                {
                    "model_id": m["model_id"],
                    "paper_core": bool(m.get("paper_core")),
                    "source_group": m.get("source_group", ""),
                    "architecture": m["architecture"],
                    "num_classes": m["num_classes"],
                    "input_size": f"{m['input_size'][0]}x{m['input_size'][1]}",
                    "shaves": m.get("shaves") or "",
                    "dataset": tr.get("dataset", ""),
                    "init_weights": tr.get("init_weights", ""),
                    "epochs_completed": tr.get("epochs_completed", ""),
                    "epochs_planned": tr.get("epochs_planned", ""),
                    "batch_size": tr.get("batch_size", ""),
                    "trained_date": tr.get("trained_date", ""),
                    "has_pt": bool(art.get("pt")),
                    "has_onnx": bool(art.get("onnx")),
                    "has_blob": bool(art.get("blob")),
                    "has_training_logs": bool(art.get("results_csv")),
                    "val_best_epoch": bm.get("epoch", ""),
                    "val_precision": bm.get("precision", ""),
                    "val_recall": bm.get("recall", ""),
                    "val_mAP50": bm.get("mAP50", ""),
                    "val_mAP50_95": bm.get("mAP50_95", ""),
                    "source_origin": m.get("source_origin", ""),
                    "notes": m.get("notes", ""),
                }
            )

    # 2. Markdown lineage & training metrics report
    md = [
        "# 3D Cup-Orientation YOLO Models — Consolidated Lineage & Training Metrics",
        "",
        f"- **Canonical 6-class task**: `{', '.join(reg['canonical_labels'])}`",
        f"- **Paper core lineup (`paper5`)**: {len(paper5_models)} models (`MARC-v5s-640`, `FT-MARC-v5s-416`, `YOLOv5s-416`, `YOLO11n-416`, `YOLO11n-640`)",
        f"- **Archived historical models**: {len(archive_models)} models (retained for traceability/ablation reference)",
        "- **Caution for paper writing**: Validation metrics below come from **each run's own validation split** (`complete_v3` vs `2026_GreG-2/3/4`). Use `eval_oak_blob.py --models paper5` + `eval_metrics.py` on a single frozen independent test set for final thesis comparison.",
        "",
        "## 1. Paper Core Lineup (`--models paper5`)",
        "",
        *build_md_table(paper5_models),
        "",
        "## 2. Archived Historical Models",
        "",
        *build_md_table(archive_models),
        "",
        "## 3. Deployment Preprocessing & SHAVES Notes (for Method / Threats to Validity)",
        "",
        "- **Class schema evolution**:",
        "  - **MARC Baseline (`MARC-v5s-640`)**: 8 classes (`Back, Front, Gripper, handle, left_side, right_side, upright, upside_down`).",
        "  - **Fine-Tuned MARC (`FT-MARC-v5s-416`)**: 7 classes (`handle` removed; `Gripper` kept).",
        "  - **Custom 6-Class Models (`YOLOv5s-416`, `YOLO11n-416`, `YOLO11n-640`)**: 6 canonical cup-orientation classes (`Gripper` removed; robot-arm/joint background hard negatives added).",
        "- **OAK-D Pro deployment uniformity**: All 5 `paper5` models are compiled to **5 SHAVES** (OpenVINO 2022.1 FP16), enabling fair on-device latency comparison.",
    ]
    (reports_dir / "model_lineage_and_metrics.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    # 3. LaTeX booktabs tables: dedicated paper5 + full archive
    (reports_dir / "table_paper5_overview.tex").write_text(
        build_latex_table(
            paper5_models,
            r"Core 5-model 3D cup-orientation YOLO lineup selected for thesis evaluation (all compiled to 5-SHAVE OAK-D Pro FP16 blobs). Validation metrics reflect each run's native validation split.",
            "tab:paper5-models",
        ),
        encoding="utf-8",
    )
    (reports_dir / "table_models_overview.tex").write_text(
        build_latex_table(
            models,
            r"Unified inventory of the 5 core 3D cup-orientation YOLO models (MARC, FT-MARC, and custom 2026_GreG-4 runs).",
            "tab:yolo-models-overview",
        ),
        encoding="utf-8",
    )

    print(
        f"Generated:\n  - {csv_path}\n  - {reports_dir / 'model_lineage_and_metrics.md'}\n"
        f"  - {reports_dir / 'table_paper5_overview.tex'}\n  - {reports_dir / 'table_models_overview.tex'}"
    )


if __name__ == "__main__":
    main()
