"""Generate publication-quality SVG vector charts of training curves & validation metrics.

Pure Python standard library (zero matplotlib dependency). Produces crisp vector
SVGs suitable for Overleaf/LaTeX (convert to PDF with Inkscape/rsvg-convert or
embed directly in reports/HTML):
  1. reports/fig_greg4_ablation_curves.svg  -- 2026_GreG-4 controlled 4-model curves (mAP@50 & mAP@50:95)
  2. reports/fig_g3_val_comparison.svg      -- Grouped bar chart of best validation P / R / mAP@50 / mAP@50:95 across all G3 runs
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


def read_curve(csv_path: Path) -> list[dict[str, float]]:
    with open(csv_path, "r", encoding="utf-8") as f:
        rows = [{k.strip(): v.strip() for k, v in r.items()} for r in csv.DictReader(f)]
    if not rows:
        return []
    cols = list(rows[0].keys())
    c_p = next((c for c in cols if "precision" in c.lower()), None)
    c_r = next((c for c in cols if "recall" in c.lower()), None)
    c_50 = next(
        (c for c in cols if ("map50" in c.lower() or "map_0.5" in c.lower()) and "95" not in c.lower()),
        None,
    )
    c_95 = next((c for c in cols if "95" in c.lower()), None)
    out = []
    for idx, r in enumerate(rows, 1):
        ep = int(float(r.get("epoch", idx)))
        # Normalize 0-based YOLOv5 epochs to 1-based for clean comparison with 1-based YOLO11
        if idx == 1 and ep == 0:
            shift = 1
        elif idx == 1:
            shift = 0
        out.append(
            {
                "epoch": float(ep + shift),
                "precision": float(r[c_p]) if c_p and r.get(c_p) else 0.0,
                "recall": float(r[c_r]) if c_r and r.get(c_r) else 0.0,
                "mAP50": float(r[c_50]) if c_50 and r.get(c_50) else 0.0,
                "mAP50_95": float(r[c_95]) if c_95 and r.get(c_95) else 0.0,
            }
        )
    return out


def esc(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def build_greg4_ablation_svg(root_dir: Path, out_svg: Path) -> None:
    series_defs = [
        ("yolov5s_416", "YOLOv5s-416 (120 ep)", "#1f77b4", "none"),
        ("yolo11n_416", "YOLO11n-416 (120 ep)", "#d62728", "none"),
        ("yolo11n_640", "YOLO11n-640 (120 ep)", "#2ca02c", "none"),
    ]
    curves: list[tuple[str, str, str, list[dict[str, float]]]] = []
    for mid, label, color, dash in series_defs:
        cp = root_dir / "models" / mid / "training" / "results.csv"
        if cp.exists():
            curves.append((label, color, dash, read_curve(cp)))

    w, h = 1120, 500
    panels = [
        ("mAP50", "Validation mAP@50 (2026_GreG-4)", 72, 72, 460, 300, 0.65, 1.00, 7),
        ("mAP50_95", "Validation mAP@50:95 (2026_GreG-4)", 610, 72, 460, 300, 0.40, 0.85, 9),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
        "<style>",
        '  .title { font: 600 16px "Segoe UI", Arial, sans-serif; fill: #1a1a1a; }',
        '  .subtitle { font: 600 13.5px "Segoe UI", Arial, sans-serif; fill: #222; }',
        '  .axis { font: 11px "Segoe UI", Arial, sans-serif; fill: #555; }',
        '  .legend { font: 12px "Segoe UI", Arial, sans-serif; fill: #222; }',
        "</style>",
        f'<rect width="{w}" height="{h}" fill="#ffffff"/>',
        '<text x="560" y="30" text-anchor="middle" class="title">Controlled 2026_GreG-4 Training Curves: Architecture, Resolution &amp; Early-Stopping Ablation</text>',
    ]

    max_ep = 120.0
    for metric, ptitle, px, py, pw, ph, ymin, ymax, n_yticks in panels:
        svg.append(f'<text x="{px + pw/2:.1f}" y="{py - 14}" text-anchor="middle" class="subtitle">{esc(ptitle)}</text>')
        svg.append(f'<rect x="{px}" y="{py}" width="{pw}" height="{ph}" fill="#fafafa" stroke="#d0d0d0" stroke-width="1"/>')

        # Y gridlines
        for i in range(n_yticks + 1):
            yv = ymin + (ymax - ymin) * i / n_yticks
            yy = py + ph - (yv - ymin) / (ymax - ymin) * ph
            svg.append(f'<line x1="{px}" y1="{yy:.1f}" x2="{px + pw}" y2="{yy:.1f}" stroke="#e8e8e8" stroke-width="1"/>')
            svg.append(f'<text x="{px - 8}" y="{yy + 4:.1f}" text-anchor="end" class="axis">{yv:.2f}</text>')

        # X gridlines
        for ep in (0, 20, 40, 60, 80, 100, 120):
            xx = px + (ep / max_ep) * pw
            svg.append(f'<line x1="{xx:.1f}" y1="{py}" x2="{xx:.1f}" y2="{py + ph}" stroke="#eeeeee" stroke-width="1"/>')
            svg.append(f'<text x="{xx:.1f}" y="{py + ph + 18}" text-anchor="middle" class="axis">{ep}</text>')

        svg.append(f'<text x="{px + pw/2:.1f}" y="{py + ph + 38}" text-anchor="middle" class="axis">Epoch</text>')

        # Plot curves
        for label, color, dash, pts in curves:
            coords = []
            best_pt = None
            for pt in pts:
                x = px + min(pt["epoch"], max_ep) / max_ep * pw
                yval = max(ymin, min(ymax, pt[metric]))
                y = py + ph - (yval - ymin) / (ymax - ymin) * ph
                coords.append(f"{x:.1f},{y:.1f}")
                if best_pt is None or pt[metric] > best_pt[2]:
                    best_pt = (x, y, pt[metric])
            dash_attr = f' stroke-dasharray="{dash}"' if dash != "none" else ""
            svg.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="2.0" stroke-linejoin="round"{dash_attr} points="{" ".join(coords)}"/>'
            )
            if best_pt is not None:
                svg.append(f'<circle cx="{best_pt[0]:.1f}" cy="{best_pt[1]:.1f}" r="3.6" fill="{color}" stroke="#fff" stroke-width="1"/>')

    # Clean 2x2 legend at bottom (zero clipping)
    col_x = [140, 620]
    row_y = [446, 472]
    for idx, (label, color, dash, pts) in enumerate(curves):
        lx = col_x[idx % 2]
        ly = row_y[idx // 2]
        best_50 = max(p["mAP50"] for p in pts)
        best_95 = max(p["mAP50_95"] for p in pts)
        dash_attr = f' stroke-dasharray="{dash}"' if dash != "none" else ""
        svg.append(f'<line x1="{lx}" y1="{ly - 4}" x2="{lx + 28}" y2="{ly - 4}" stroke="{color}" stroke-width="2.5"{dash_attr}/>')
        txt = f"{label}  —  peak mAP@50: {best_50:.3f} | mAP@50:95: {best_95:.3f}"
        svg.append(f'<text x="{lx + 36}" y="{ly}" class="legend">{esc(txt)}</text>')

    svg.append("</svg>")
    out_svg.parent.mkdir(parents=True, exist_ok=True)
    out_svg.write_text("\n".join(svg) + "\n", encoding="utf-8")


def build_g3_bar_svg(reg: dict[str, Any], out_svg: Path) -> None:
    g3_models = [m for m in reg["models"] if m.get("source_group") in ("ours", "custom")]
    w, h = 1120, 480
    px, py, pw, ph = 75, 70, 1000, 275
    ymin, ymax = 0.60, 1.00

    metrics = [
        ("precision", "Precision", "#4e79a7"),
        ("recall", "Recall", "#f28e2b"),
        ("mAP50", "mAP@50", "#59a14f"),
        ("mAP50_95", "mAP@50:95", "#e15759"),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
        "<style>",
        '  .title { font: 600 16px "Segoe UI", Arial, sans-serif; fill: #1a1a1a; }',
        '  .note { font: 11px "Segoe UI", Arial, sans-serif; fill: #666; }',
        '  .axis { font: 11px "Segoe UI", Arial, sans-serif; fill: #444; }',
        '  .val { font: 9.5px "Segoe UI", Arial, sans-serif; fill: #222; }',
        '  .legend { font: 12px "Segoe UI", Arial, sans-serif; fill: #222; }',
        "</style>",
        f'<rect width="{w}" height="{h}" fill="#ffffff"/>',
        '<text x="560" y="28" text-anchor="middle" class="title">Generation-3 (6-Class) Models — Best Validation Metrics Across Runs</text>',
        '<text x="560" y="48" text-anchor="middle" class="note">Note: Validation splits differ between 2026_GreG-2, GreG-3, and GreG-4; use unified frozen test set for final thesis ranking.</text>',
        f'<rect x="{px}" y="{py}" width="{pw}" height="{ph}" fill="#fafafa" stroke="#d0d0d0"/>',
    ]

    for i in range(9):
        yv = ymin + (ymax - ymin) * i / 8
        yy = py + ph - (yv - ymin) / (ymax - ymin) * ph
        svg.append(f'<line x1="{px}" y1="{yy:.1f}" x2="{px + pw}" y2="{yy:.1f}" stroke="#e8e8e8"/>')
        svg.append(f'<text x="{px - 8}" y="{yy + 4:.1f}" text-anchor="end" class="axis">{yv:.2f}</text>')

    group_w = pw / len(g3_models)
    bar_w = group_w * 0.17
    for gi, m in enumerate(g3_models):
        bm = m.get("training", {}).get("best_val_metrics", {})
        gx = px + gi * group_w + group_w * 0.14
        for mi, (mk, _, color) in enumerate(metrics):
            val = float(bm.get(mk) or 0.0)
            clamped = max(ymin, min(ymax, val))
            bh = (clamped - ymin) / (ymax - ymin) * ph
            bx = gx + mi * (bar_w + 2)
            by = py + ph - bh
            svg.append(f'<rect x="{bx:.1f}" y="{by:.1f}" width="{bar_w:.1f}" height="{bh:.1f}" fill="{color}" rx="2"/>')
            svg.append(f'<text x="{bx + bar_w/2:.1f}" y="{by - 4:.1f}" text-anchor="middle" class="val">{val:.2f}</text>')

        parts = m["model_id"].replace("ours_", "").split("_", 1)
        cx = px + (gi + 0.5) * group_w
        svg.append(f'<text x="{cx:.1f}" y="{py + ph + 18}" text-anchor="middle" class="axis">{esc(parts[0])}</text>')
        if len(parts) > 1:
            svg.append(f'<text x="{cx:.1f}" y="{py + ph + 33}" text-anchor="middle" class="axis">{esc(parts[1])}</text>')

    # Legend
    lx = 340
    ly = 430
    for _, lbl, color in metrics:
        svg.append(f'<rect x="{lx}" y="{ly - 11}" width="14" height="14" fill="{color}" rx="2"/>')
        svg.append(f'<text x="{lx + 20}" y="{ly}" class="legend">{esc(lbl)}</text>')
        lx += 120

    svg.append("</svg>")
    out_svg.parent.mkdir(parents=True, exist_ok=True)
    out_svg.write_text("\n".join(svg) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot G3 training curves and validation metric comparisons as SVG.")
    parser.add_argument("--registry", type=Path, default=Path(__file__).resolve().parents[1] / "registry.json")
    args = parser.parse_args()

    root_dir = args.registry.resolve().parent
    reg = json.loads(args.registry.read_text(encoding="utf-8"))
    out1 = root_dir / "reports" / "fig_greg4_ablation_curves.svg"
    out2 = root_dir / "reports" / "fig_g3_val_comparison.svg"
    build_greg4_ablation_svg(root_dir, out1)
    build_g3_bar_svg(reg, out2)
    print(f"Generated:\n  - {out1}\n  - {out2}")


if __name__ == "__main__":
    main()
