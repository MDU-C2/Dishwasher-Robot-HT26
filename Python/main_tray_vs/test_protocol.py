import os
import csv
from datetime import datetime
from statistics import mean, median, stdev
import numpy as np


# ==============================================================================
# LEGACY FUNCTIONS (Kept for backward compatibility)
# ==============================================================================
def create_today_textfile():
    today_str = datetime.now().strftime("%Y-%m-%d_%H.%M")
    folder = "Test_Protocols"
    os.makedirs(folder, exist_ok=True)
    filename = os.path.join(folder, f"{today_str}.txt")
    if not os.path.exists(filename):
        with open(filename, "w", encoding="utf-8") as f:
            f.write(f"File created on {today_str}\n")
    return filename


def fill_meta_data(file_path):
    with open(file_path, "a", encoding="utf-8") as f:
        f.write("---------- Meta Data ----------\n")
        f.write(f"Number of table verification cup positions: {len(REFERENCE_CUP_PAIRS)}\n")


def ask_user(file_path, start_end):
    if start_end == "start":
        response = input("What variable was tested? ")
        with open(file_path, "a", encoding="utf-8") as f:
            f.write("---------- Test Information ----------\n")
            f.write(f"Tested Variable: {response}\n")
    else:
        response = input("How did the test go? ")
        with open(file_path, "a", encoding="utf-8") as f:
            f.write("---------- Test Conclusion ----------\n")
            f.write(f"Test Conclusion: {response}\n")


def cup_information(file_path, count, cam_coords, process_time, confidence=None, label=None):
    with open(file_path, "a", encoding="utf-8") as f:
        f.write(f"------- Cup {count} Coordinates -------\n")
        if label is not None:
            f.write(f"Detection Label: {label}\n")
        if confidence is not None:
            f.write(f"Detection Confidence: {confidence:.1f}%\n")
        f.write(f"Robot Coordinates: {cam_coords}\n")
        f.write(f"Robot cycle time (start move -> left in tray): {process_time:.3f} seconds\n\n")


def log_time_summary(file_path, times_sec):
    if not times_sec:
        return
    avg = mean(times_sec)
    sd = stdev(times_sec) if len(times_sec) > 1 else 0.0
    with open(file_path, "a", encoding="utf-8") as f:
        f.write("---------- Timing Summary ----------\n")
        f.write(f"Trials: {len(times_sec)}\n")
        f.write(f"Average robot cycle time: {avg:.3f} s\n")
        f.write(f"Std dev: {sd:.3f} s\n\n")


def log_rms_error(file_path, rms_error):
    with open(file_path, "a", encoding="utf-8") as f:
        f.write("---------- Calibration Accuracy ----------\n")
        f.write(f"Table XY RMS alignment error: {rms_error:.3f} mm\n\n")


# ==============================================================================
# COMPREHENSIVE REPORT LOGGER
# ==============================================================================
# 9 physical table-level cup verification pairs used with HT_New_Run2_Mean + x_compensate
REFERENCE_CUP_PAIRS = (
    {"ht_xyz": [358.2118, 46.9082, 191.3635], "pendant_xyz": [301.31, 37.16, 39.79]},
    {"ht_xyz": [438.4847, 62.7487, 191.3635 if False else 108.5933], "pendant_xyz": [415.81, 56.22, 38.83]},
    {"ht_xyz": [498.0360, 145.6610, 43.2758], "pendant_xyz": [511.33, 143.29, 37.90]},
    {"ht_xyz": [594.7541, 60.0524, -55.0660], "pendant_xyz": [640.01, 55.30, 38.52]},
    {"ht_xyz": [608.6125, 203.1232, -69.6098], "pendant_xyz": [656.44, 199.36, 47.33]},
    {"ht_xyz": [345.5296, 105.3962, 199.4084], "pendant_xyz": [292.20, 104.56, 40.54]},
    {"ht_xyz": [514.4336, 152.7753, 36.7747], "pendant_xyz": [512.79, 149.42, 38.43]},
    {"ht_xyz": [503.8277, 52.4539, 44.8309], "pendant_xyz": [499.70, 39.00, 41.25]},
    {"ht_xyz": [456.4269, 63.7099, 94.7509], "pendant_xyz": [437.23, 54.21, 41.11]},
)


def _safe_stats_dict(values):
    """Return dictionary of mean, median, std, min, max, p95, count for a list of floats."""
    clean = [float(v) for v in values if v is not None and np.isfinite(v) and v > 0.0]
    if not clean:
        clean = [float(v) for v in values if v is not None and np.isfinite(v)]
    if not clean:
        return None
    arr = np.asarray(clean, dtype=np.float64)
    return {
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "std": float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0,
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "p95": float(np.percentile(arr, 95)),
        "count": int(len(arr)),
    }


def _table_row(label, values, unit="", decimals=2):
    """Format a single row for an ASCII summary table."""
    st = _safe_stats_dict(values)
    if st is None:
        return (
            f"| {label:<42} | {'N/A':>10} | {'N/A':>10} | "
            f"{'N/A':>10} | {'N/A':>10} | {'N/A':>10} |\n"
        )
    fmt = f".{decimals}f"
    u = f" {unit}" if unit else ""
    return (
        f"| {label:<42} | {st['mean']:{fmt}}{u:>3} | {st['median']:{fmt}}{u:>3} | "
        f"{st['std']:{fmt}}{u:>3} | {st['min']:{fmt}}{u:>3} | {st['max']:{fmt}}{u:>3} |\n"
    )


def _ascii_bar(val, max_val, width=28):
    if max_val <= 0 or val <= 0:
        return "[" + ("." * width) + "]"
    filled = int(round(min(1.0, val / max_val) * width))
    return "[" + ("#" * filled) + ("." * (width - filled)) + "]"


def _safe_mean(values, default=0.0):
    clean = [float(v) for v in values if v is not None and np.isfinite(v)]
    return float(np.mean(clean)) if clean else default


def _safe_std(values, default=0.0):
    clean = [float(v) for v in values if v is not None and np.isfinite(v)]
    return float(np.std(clean, ddof=1)) if len(clean) > 1 else default


def _safe_p95(values, default=0.0):
    clean = [float(v) for v in values if v is not None and np.isfinite(v)]
    return float(np.percentile(clean, 95)) if clean else default


def compute_calibration_metrics(x_comp_fn=None):
    """
    Evaluate the active HT_New_Run2_Mean + x_compensate transformation
    against the 9 physical table-level cup positions (does NOT use old saved_coordinates.txt).
    """
    metrics = {}
    try:
        xy_errors = []
        x_errors = []
        y_errors = []
        pair_details = []
        for idx, pair in enumerate(REFERENCE_CUP_PAIRS, start=1):
            ht_x, ht_y, _ht_z = pair["ht_xyz"]
            pend_x, pend_y, _pend_z = pair["pendant_xyz"]
            comp_x = float(x_comp_fn(ht_x)) if callable(x_comp_fn) else float(ht_x)
            comp_y = float(ht_y)
            dx = comp_x - pend_x
            dy = comp_y - pend_y
            dist_xy = float(np.hypot(dx, dy))
            x_errors.append(dx)
            y_errors.append(dy)
            xy_errors.append(dist_xy)
            pair_details.append({
                "idx": idx,
                "pred_xy": [round(comp_x, 2), round(comp_y, 2)],
                "actual_xy": [round(pend_x, 2), round(pend_y, 2)],
                "dx": round(dx, 2),
                "dy": round(dy, 2),
                "dist_xy": round(dist_xy, 2),
            })

        metrics["table_cup_pairs_count"] = len(REFERENCE_CUP_PAIRS)
        metrics["table_xy_rms_mm"] = float(np.sqrt(np.mean(np.square(xy_errors))))
        metrics["table_xy_mean_mm"] = float(np.mean(xy_errors))
        metrics["table_xy_median_mm"] = float(np.median(xy_errors))
        metrics["table_xy_max_mm"] = float(np.max(xy_errors))
        metrics["table_xy_min_mm"] = float(np.min(xy_errors))
        metrics["table_x_rmse_mm"] = float(np.sqrt(np.mean(np.square(x_errors))))
        metrics["table_y_rmse_mm"] = float(np.sqrt(np.mean(np.square(y_errors))))
        metrics["pair_details"] = pair_details
    except Exception:
        pass

    return metrics


class ReportLogger:
    """
    Collects all vision, neural network inference, flicker stability, calibration,
    and pure robot pick-and-place cycle times, and writes a complete visual report
    file automatically (rewriting the updated summary after every cup AND at exit).
    """

    def __init__(
        self,
        model_name,
        model_info=None,
        active_ht_name="",
        slot_source="camera",
        z_upward=55.0,
        z_downward=45.0,
        grasp_frame_count=5,
        same_cup_distance_mm=40.0,
        cal_metrics=None,
        tested_variable="",
    ):
        self.start_dt = datetime.now()
        self.run_id = self.start_dt.strftime("%Y-%m-%d_%H.%M.%S")
        self.model_name = str(model_name)
        self.model_info = model_info or {}
        self.active_ht_name = active_ht_name
        self.slot_source = slot_source
        self.z_upward = z_upward
        self.z_downward = z_downward
        self.grasp_frame_count = grasp_frame_count
        self.same_cup_distance_mm = same_cup_distance_mm
        self.cal_metrics = cal_metrics or {}
        self.tested_variable = tested_variable
        self.conclusion_note = ""

        self.folder = "Test_Protocols"
        os.makedirs(self.folder, exist_ok=True)
        self.report_txt_path = os.path.join(
            self.folder, f"Report_{self.run_id}_{self.model_name}.txt"
        )
        self.cups_csv_path = os.path.join(self.folder, "Report_Cups_Log.csv")
        self.runs_csv_path = os.path.join(self.folder, "Report_Runs_Summary.csv")

        # Vision & NN frame-by-frame metrics
        self.total_frames = 0
        self.idle_frames = 0
        self.frames_with_detections = 0
        self.frames_with_valid_target = 0
        self.on_chip_inference_ms = []  # OAK-D hardware clock time per NN inference frame (ms)
        self.cam_to_pc_latency_ms = []  # Fresh frame capture -> PC receipt latency (ms)
        self.usb_host_overhead_ms = []  # Camera->PC latency minus on-chip inference time (ms)
        self.nn_fps_list = []           # True OAK-D neural network throughput (FPS)
        self.host_loop_ms = []          # PC OpenCV display & loop time (ms)
        self.class_confidences = {}     # label -> list of confidence % (0-100)
        self.class_valid_depths = {}    # label -> list of camera Z mm

        # 5-Frame window & flicker tracking
        self.total_label_flicker_resets = 0
        self.total_distance_jump_resets = 0
        self.current_cup_label_resets = 0
        self.current_cup_distance_resets = 0
        self.current_cup_frames_inspected = 0
        self.window_start_perf = None

        # Per-cup recorded locks & completed cup records
        self.lock_records = []
        self.cup_records = []
        self._finalized = False

        self._rewrite_full_report_file()

    def _build_header_and_cups_text(self):
        nn_cfg = self.model_info.get("config", {}).get("nn_config", {})
        meta = nn_cfg.get("NN_specific_metadata", {})
        labels = self.model_info.get("config", {}).get("mappings", {}).get("labels", [])
        input_w = self.model_info.get("input_width", "N/A")
        input_h = self.model_info.get("input_height", "N/A")
        blob_path = os.path.basename(self.model_info.get("blob_path", "N/A"))
        conf_thr = meta.get("confidence_threshold", "N/A")
        iou_thr = meta.get("iou_threshold", "N/A")

        lines = []
        lines.append("=" * 88 + "\n")
        lines.append("                 ABB YuMi VISION & PICK-AND-PLACE EVALUATION REPORT\n")
        lines.append("=" * 88 + "\n")
        lines.append(f"Run ID / Start Time       : {self.start_dt.strftime('%Y-%m-%d %H:%M:%S')}\n")
        if self.tested_variable:
            lines.append(f"Tested Variable / Note    : {self.tested_variable}\n")

        lines.append("\n" + "-" * 88 + "\n")
        lines.append(" 1. DETECTION MODEL & VISION FILTER SETTINGS\n")
        lines.append("-" * 88 + "\n")
        lines.append(f"  Model Selected          : YOLO {self.model_name}\n")
        lines.append(f"  Model Blob File         : {blob_path}\n")
        lines.append(f"  Network Input Size      : {input_w} x {input_h} px\n")
        lines.append(f"  Confidence / IoU Thresh : Conf >= {conf_thr}  |  IoU <= {iou_thr}\n")
        lines.append(f"  Classes ({len(labels)})            : {', '.join(str(x) for x in labels)}\n")
        lines.append(
            f"  Stability Filter        : Requires {self.grasp_frame_count} consecutive distinct frames "
            f"with identical label & within {self.same_cup_distance_mm:.0f} mm\n"
        )

        lines.append("\n" + "-" * 88 + "\n")
        lines.append(" 2. HAND-EYE TRANSFORMATION MATRIX VERIFICATION (TABLE-LEVEL CUPS)\n")
        lines.append("-" * 88 + "\n")
        lines.append(f"  Active Matrix           : {self.active_ht_name}\n")
        lines.append("  Linear X Compensation   : robot_x = 1.401822 * ht_x - 198.521 mm (Y unchanged)\n")
        lines.append(f"  Grasp Heights (Fixed Z) : Upright = {self.z_upward:.1f} mm  |  Downward/Side = {self.z_downward:.1f} mm\n")

        if self.cal_metrics and "pair_details" in self.cal_metrics:
            lines.append("\n  Verification on 9 Physical Table Cup Positions (Predicted vs. Jogged Pendant):\n")
            lines.append("  +-------+-----------------------+-----------------------+----------+----------+------------+\n")
            lines.append("  | Cup # | Predicted XY (mm)     | Actual Jogged XY (mm) | Error dX | Error dY | 2D Err |e| |\n")
            lines.append("  +-------+-----------------------+-----------------------+----------+----------+------------+\n")
            for p in self.cal_metrics["pair_details"]:
                px, py = p["pred_xy"]
                ax, ay = p["actual_xy"]
                lines.append(
                    f"  |   #{p['idx']}  | [{px:>7.2f}, {py:>7.2f}]   | [{ax:>7.2f}, {ay:>7.2f}]   | "
                    f"{p['dx']:>+6.2f}mm | {p['dy']:>+6.2f}mm |  {p['dist_xy']:>6.2f} mm |\n"
                )
            lines.append("  +-------+-----------------------+-----------------------+----------+----------+------------+\n")
            lines.append(
                f"  Summary ->  2D XY RMS Error : {self.cal_metrics['table_xy_rms_mm']:.2f} mm  |  "
                f"Mean Error : {self.cal_metrics['table_xy_mean_mm']:.2f} mm  |  "
                f"Max Error : {self.cal_metrics['table_xy_max_mm']:.2f} mm\n"
            )
            lines.append(
                f"              X-Axis RMSE     : {self.cal_metrics['table_x_rmse_mm']:.2f} mm  |  "
                f"Y-Axis RMSE: {self.cal_metrics['table_y_rmse_mm']:.2f} mm\n"
            )

        lines.append("\n" + "=" * 88 + "\n")
        lines.append(" 3. PER-CUP PICK-AND-PLACE LOG\n")
        lines.append("=" * 88 + "\n\n")

        if not self.cup_records:
            lines.append("  (No cups picked yet in this session.)\n\n")
        else:
            for record in self.cup_records:
                placed_str = (
                    "Verified in Tray (OK)"
                    if record["placed_ok"] is True
                    else ("Skipped (Fixed Slot Mode)" if record["placed_ok"] is None else "Not Verified")
                )
                grasp_status = (
                    "ABORTED (Failed after 3 attempts)"
                    if not record["handover_completed"]
                    else ("First-Try Success" if record["first_try_ok"] else f"Succeeded on Attempt #{record['pick_attempts']} ({record['grasp_retries']}x retry)")
                )
                lines.append(f"  +------------------------------------------------------------------------------------+\n")
                lines.append(f"  | CUP #{record['cup_no']:<3}  ({record['timestamp']})  -  Orientation: {record['label']:<12}  Confidence: {record['conf_pct']:>5.1f}% |\n")
                lines.append(f"  +------------------------------------------------------------------------------------+\n")
                lines.append(
                    f"  | [Vision Speed]   On-Chip AI Inference: {record['ai_inference_ms']:>5.1f} ms  |  "
                    f"Camera->PC Latency: {record['nn_latency_ms']:>5.1f} ms  |  {record['nn_fps']:>4.1f} FPS\n"
                )
                lines.append(
                    f"  | [5-Frame Lock]   Time to Lock 5 Frames: {record['lock_time_s']:>5.2f} s  |  "
                    f"Flicker Resets: {record['total_resets']} "
                    f"(Label={record['label_resets']}, Jump={record['distance_resets']})\n"
                )
                lines.append(
                    f"  | [3D Coordinates] Camera XYZ: {record['cam_xyz_mm']} mm (Dist: {record['cam_distance_mm']:.1f} mm)\n"
                )
                lines.append(
                    f"  | [3D Stability]   5-Frame Jitter Range dXYZ: {record['spread_xyz_mm']} mm (Total Spread: {record['spread_norm_mm']:.2f} mm)\n"
                )
                lines.append(
                    f"  | [Robot Target]   Pick XYZ: {record['rob_xyz_mm']} mm  ->  Tray '{record['chosen_slot']}' @ {record['slot_coords_mm']} mm\n"
                )
                lines.append(
                    f"  | [Grasp & Tray]   Pick Attempts: {record['pick_attempts']} ({grasp_status})  |  Tray Check: {placed_str}\n"
                )
                if record["retry_history"]:
                    for r_item in record["retry_history"]:
                        lines.append(
                            f"  |   -> Retry #{r_item.get('attempt')}: "
                            f"Prev={r_item.get('previous_xyz')} -> Fresh={r_item.get('fresh_xyz')}\n"
                        )
                lines.append(
                    f"  | [Robot Timing]   Left Arm (Pick + Handover)         : {record['left_arm_time_s']:>6.2f} s\n"
                    f"  |                  Right Arm (Handover -> Leave Tray) : {record['right_arm_time_s']:>6.2f} s\n"
                    f"  |                  ROBOT CYCLE TIME (Start -> In Tray): {record['robot_cycle_time_s']:>6.2f} s\n"
                )
                lines.append(f"  +------------------------------------------------------------------------------------+\n\n")

        return "".join(lines)

    def _build_summary_sections_text(self):
        end_dt = datetime.now()
        session_duration_s = (end_dt - self.start_dt).total_seconds()
        det_rate_pct = (
            (100.0 * self.frames_with_detections / self.total_frames)
            if self.total_frames > 0
            else 0.0
        )
        valid_target_rate_pct = (
            (100.0 * self.frames_with_valid_target / self.total_frames)
            if self.total_frames > 0
            else 0.0
        )

        lines = []
        lines.append("=" * 88 + "\n")
        lines.append(" 4. VISION & NEURAL NETWORK LATENCY SUMMARY\n")
        lines.append("=" * 88 + "\n")
        lines.append("  HOW TO READ THESE LATENCY METRICS:\n\n")
        lines.append("    [Camera Sensor] ---> [OAK-D AI Chip (YOLO + Depth)] ---> [USB Transfer] ---> [Python PC]\n")
        lines.append("                    |<---- 1. On-Chip AI Inference ---->|<-- 2. USB/Queue ->|\n")
        lines.append("                    |<------------ 3. Camera -> PC Total Latency ---------->|\n\n")
        lines.append("   1. On-Chip AI Inference Time  : Exact time per detection on the OAK-D Myriad X chip\n")
        lines.append("                                   (measured via the camera's internal hardware clock).\n")
        lines.append("   2. USB Transfer & Host Delay  : Additional time for USB transfer and PC frame sync.\n")
        lines.append("   3. Camera -> PC Total Latency : Total real-time delay from camera sensor capture until\n")
        lines.append("                                   the 3D coordinates are read by Python on the PC.\n")
        lines.append("   4. Detection Speed (FPS)      : True neural network detections per second (1000 / AI ms).\n\n")

        lines.append(f"  Session Duration          : {session_duration_s:.1f} s  ({self.total_frames} distinct camera frames)\n")
        lines.append(
            f"  Detection Availability    : {self.frames_with_detections}/{self.total_frames} frames "
            f"({det_rate_pct:.1f}%) had detections | {valid_target_rate_pct:.1f}% had a valid 3D target\n\n"
        )

        lines.append("+--------------------------------------------+------------+------------+------------+------------+------------+\n")
        lines.append("| Latency / Speed Metric                     |    Average |     Median |    Std Dev |  Min (Best)| Max (Worst)|\n")
        lines.append("+--------------------------------------------+------------+------------+------------+------------+------------+\n")
        lines.append(_table_row("1. On-Chip AI Inference Time (OAK-D)", self.on_chip_inference_ms, "ms", 1))
        lines.append(_table_row("2. USB Transfer & Host Queue Delay", self.usb_host_overhead_ms, "ms", 1))
        lines.append(_table_row("3. Camera -> PC Total Latency", self.cam_to_pc_latency_ms, "ms", 1))
        lines.append(_table_row("4. Detection Speed (AI Throughput)", self.nn_fps_list, "FPS", 1))
        lines.append(_table_row("5. PC OpenCV Display Loop Time", self.host_loop_ms, "ms", 1))
        lines.append("+--------------------------------------------+------------+------------+------------+------------+------------+\n")

        avg_ai = _safe_mean(self.on_chip_inference_ms)
        avg_usb = _safe_mean(self.usb_host_overhead_ms)
        avg_e2e = _safe_mean(self.cam_to_pc_latency_ms)
        max_bar_ref = max(avg_ai, avg_usb, avg_e2e, 1.0)
        lines.append("\n  Visual Latency Breakdown (Average ms):\n")
        lines.append(f"    1. On-Chip AI Inference : {_ascii_bar(avg_ai, max_bar_ref)} {avg_ai:>6.1f} ms\n")
        lines.append(f"    2. USB & Host Overhead  : {_ascii_bar(avg_usb, max_bar_ref)} {avg_usb:>6.1f} ms\n")
        lines.append(f"    3. Total Cam->PC Latency: {_ascii_bar(avg_e2e, max_bar_ref)} {avg_e2e:>6.1f} ms\n")

        lines.append("\n" + "=" * 88 + "\n")
        lines.append(" 5. PER-CLASS DETECTION & CONFIDENCE BREAKDOWN\n")
        lines.append("=" * 88 + "\n")
        total_det_instances = sum(len(v) for v in self.class_confidences.values())
        if total_det_instances == 0:
            lines.append("  No object detections recorded during this session.\n")
        else:
            lines.append("+------------------+---------+---------+-------------+-------------+---------+---------+--------------+\n")
            lines.append("| Class Label      |   Count |   Share |   Mean Conf |    Std Conf |     Min |     Max | Mean Depth Z |\n")
            lines.append("+------------------+---------+---------+-------------+-------------+---------+---------+--------------+\n")
            for lbl, confs in sorted(
                self.class_confidences.items(), key=lambda kv: len(kv[1]), reverse=True
            ):
                cnt = len(confs)
                share = 100.0 * cnt / total_det_instances
                c_mean = _safe_mean(confs)
                c_std = _safe_std(confs)
                c_min = min(confs)
                c_max = max(confs)
                depths = self.class_valid_depths.get(lbl, [])
                d_str = f"{_safe_mean(depths):>9.1f} mm" if depths else "         N/A"
                lines.append(
                    f"| {lbl:<16} | {cnt:>7} | {share:>6.1f}% |    {c_mean:>6.1f} % |    {c_std:>6.1f} % | "
                    f" {c_min:>4.0f} % |  {c_max:>4.0f} % | {d_str} |\n"
                )
            lines.append("+------------------+---------+---------+-------------+-------------+---------+---------+--------------+\n")

        lines.append("\n" + "=" * 88 + "\n")
        lines.append(" 6. 5-FRAME STABILITY & FLICKER FILTER SUMMARY\n")
        lines.append("=" * 88 + "\n")
        total_resets = self.total_label_flicker_resets + self.total_distance_jump_resets
        lines.append(f"  Total 5-Frame Target Locks : {len(self.lock_records)}\n")
        lines.append(
            f"  Total Window Resets        : {total_resets}  "
            f"(Orientation Label Flicker: {self.total_label_flicker_resets}  |  "
            f"3D Jump >{self.same_cup_distance_mm:.0f}mm: {self.total_distance_jump_resets})\n\n"
        )
        if self.lock_records:
            lock_times = [r["lock_time_s"] for r in self.lock_records]
            spread_x = [r["spread_xyz_mm"][0] for r in self.lock_records]
            spread_y = [r["spread_xyz_mm"][1] for r in self.lock_records]
            spread_z = [r["spread_xyz_mm"][2] for r in self.lock_records]
            spread_norms = [r["spread_norm_mm"] for r in self.lock_records]
            lines.append("+--------------------------------------------+------------+------------+------------+------------+------------+\n")
            lines.append("| 5-Frame Stability Metric                   |    Average |     Median |    Std Dev |  Min (Best)| Max (Worst)|\n")
            lines.append("+--------------------------------------------+------------+------------+------------+------------+------------+\n")
            lines.append(_table_row("Time to Lock 5 Consistent Frames", lock_times, "s", 2))
            lines.append(_table_row("5-Frame Camera X Jitter (Max - Min)", spread_x, "mm", 2))
            lines.append(_table_row("5-Frame Camera Y Jitter (Max - Min)", spread_y, "mm", 2))
            lines.append(_table_row("5-Frame Depth Z Jitter (Max - Min)", spread_z, "mm", 2))
            lines.append(_table_row("5-Frame Total 3D Jitter Norm", spread_norms, "mm", 2))
            lines.append("+--------------------------------------------+------------+------------+------------+------------+------------+\n")

        lines.append("\n" + "=" * 88 + "\n")
        lines.append(" 7. ROBOT CYCLE TIME SUMMARY (START MOVE -> LEFT OBJECT IN TRAY)\n")
        lines.append("=" * 88 + "\n")
        lines.append("  NOTE: Robot Cycle Time measures strictly the physical robot execution from the\n")
        lines.append("        moment the robot starts moving toward the mug until the mug is left in the tray.\n")
        lines.append("        (Excludes 2D camera slot-selection and post-placement verification polling.)\n\n")

        n_cups = len(self.cup_records)
        completed_cups = [c for c in self.cup_records if c.get("handover_completed", True)]
        lines.append(f"  Total Cups Attempted       : {n_cups}  (Completed to Tray: {len(completed_cups)})\n")
        if n_cups > 0:
            first_try_cnt = sum(1 for c in self.cup_records if c["first_try_ok"] and c.get("handover_completed", True))
            total_retries = sum(c["grasp_retries"] for c in self.cup_records)
            verified_cnt = sum(1 for c in self.cup_records if c["placed_ok"] is True)
            verified_applicable = sum(1 for c in self.cup_records if c["placed_ok"] is not None)

            lines.append(
                f"  First-Try Grasp Success    : {first_try_cnt}/{n_cups} "
                f"({100.0 * first_try_cnt / n_cups:.1f}%)  |  Total Grasp Retries: {total_retries}\n"
            )
            if verified_applicable > 0:
                lines.append(
                    f"  2D Tray Verification Rate  : {verified_cnt}/{verified_applicable} "
                    f"({100.0 * verified_cnt / verified_applicable:.1f}%)\n"
                )
            lines.append("\n")
            timing_source = completed_cups if completed_cups else self.cup_records
            left_times = [c["left_arm_time_s"] for c in timing_source]
            right_times = [c["right_arm_time_s"] for c in timing_source]
            total_times = [c["robot_cycle_time_s"] for c in timing_source]
            lines.append("+--------------------------------------------+------------+------------+------------+------------+------------+\n")
            lines.append("| Robot Motion Phase                         |    Average |     Median |    Std Dev |  Min (Fast)| Max (Slow) |\n")
            lines.append("+--------------------------------------------+------------+------------+------------+------------+------------+\n")
            lines.append(_table_row("1. Left Arm (Pick + Handover)", left_times, "s", 2))
            lines.append(_table_row("2. Right Arm (Handover -> Leave in Tray)", right_times, "s", 2))
            lines.append(_table_row("TOTAL ROBOT CYCLE (Start Move -> In Tray)", total_times, "s", 2))
            lines.append("+--------------------------------------------+------------+------------+------------+------------+------------+\n")

            avg_l = _safe_mean(left_times)
            avg_r = _safe_mean(right_times)
            avg_t = max(_safe_mean(total_times), 0.001)
            lines.append("\n  Visual Robot Cycle Share (Average s):\n")
            lines.append(f"    Left Arm (Pick+Handover) : {_ascii_bar(avg_l, avg_t)} {avg_l:>6.2f} s ({100.0*avg_l/avg_t:4.1f}%)\n")
            lines.append(f"    Right Arm (Tray Place)   : {_ascii_bar(avg_r, avg_t)} {avg_r:>6.2f} s ({100.0*avg_r/avg_t:4.1f}%)\n")
            lines.append(f"    TOTAL ROBOT CYCLE TIME   : {_ascii_bar(avg_t, avg_t)} {avg_t:>6.2f} s (100.0%)\n")

        if self.conclusion_note:
            lines.append("\n" + "=" * 88 + "\n")
            lines.append(" 8. OPERATOR CONCLUSION NOTES\n")
            lines.append("=" * 88 + "\n")
            lines.append(f"  {self.conclusion_note}\n")

        lines.append("\n" + "=" * 88 + "\n")
        return "".join(lines)

    def _rewrite_full_report_file(self):
        """Write the complete report (Sections 1-7) so the file is always complete on disk."""
        full_text = self._build_header_and_cups_text() + self._build_summary_sections_text()
        with open(self.report_txt_path, "w", encoding="utf-8") as f:
            f.write(full_text)

    def record_frame(
        self,
        detections,
        labels,
        selected_target,
        on_chip_inference_ms,
        cam_to_pc_latency_ms,
        loop_ms,
        is_busy,
    ):
        """Record per-frame inference latency, FPS, and detection statistics for each distinct camera frame."""
        self.total_frames += 1
        if not is_busy:
            self.idle_frames += 1
            if selected_target is not None:
                self.current_cup_frames_inspected += 1

        if on_chip_inference_ms is not None and 2.0 <= on_chip_inference_ms <= 500.0:
            self.on_chip_inference_ms.append(float(on_chip_inference_ms))
            self.nn_fps_list.append(float(1000.0 / on_chip_inference_ms))

        if cam_to_pc_latency_ms is not None and 2.0 <= cam_to_pc_latency_ms <= 1000.0:
            self.cam_to_pc_latency_ms.append(float(cam_to_pc_latency_ms))
            if on_chip_inference_ms is not None and 2.0 <= on_chip_inference_ms <= cam_to_pc_latency_ms:
                self.usb_host_overhead_ms.append(float(cam_to_pc_latency_ms - on_chip_inference_ms))

        if loop_ms is not None and loop_ms >= 0.0:
            self.host_loop_ms.append(float(loop_ms))

        if detections:
            self.frames_with_detections += 1
        if selected_target is not None:
            self.frames_with_valid_target += 1

        for det in detections:
            lbl = labels[det.label] if 0 <= det.label < len(labels) else str(det.label)
            conf_pct = float(det.confidence * 100.0)
            self.class_confidences.setdefault(lbl, []).append(conf_pct)
            z_mm = float(det.spatialCoordinates.z)
            if np.isfinite(z_mm) and z_mm > 0:
                self.class_valid_depths.setdefault(lbl, []).append(z_mm)

    def mark_window_started(self):
        if self.window_start_perf is None:
            self.window_start_perf = datetime.now()

    def record_window_reset(self, reason):
        """Record why the 5-frame grasp window was reset ('label' or 'distance')."""
        if reason == "label":
            self.total_label_flicker_resets += 1
            self.current_cup_label_resets += 1
        else:
            self.total_distance_jump_resets += 1
            self.current_cup_distance_resets += 1
        self.window_start_perf = datetime.now()

    def record_target_lock(self, grasp_samples, median_xyz, spread_xyz, target_robot_xyz, label, median_conf):
        """Snapshot vision stability metrics when 5/5 consistent frames are locked."""
        now = datetime.now()
        lock_time_s = (
            (now - self.window_start_perf).total_seconds()
            if self.window_start_perf is not None
            else 0.0
        )
        stacked = np.vstack([s["xyz"] for s in grasp_samples])
        std_xyz = np.std(stacked, axis=0, ddof=1) if len(stacked) > 1 else np.zeros(3)
        spread_norm_mm = float(np.linalg.norm(spread_xyz))
        cam_dist_mm = float(np.linalg.norm(median_xyz))
        recent_ai_ms = _safe_mean(self.on_chip_inference_ms[-30:])
        recent_e2e_ms = _safe_mean(self.cam_to_pc_latency_ms[-30:])
        recent_fps = _safe_mean(self.nn_fps_list[-30:])

        lock_info = {
            "timestamp": now.strftime("%Y-%m-%d %H:%M:%S"),
            "label": label,
            "median_conf_pct": float(median_conf),
            "mean_conf_pct": float(np.mean([s["conf"] for s in grasp_samples])),
            "cam_median_xyz_mm": [round(float(v), 2) for v in median_xyz],
            "cam_distance_mm": round(cam_dist_mm, 2),
            "spread_xyz_mm": [round(float(v), 2) for v in spread_xyz],
            "spread_norm_mm": round(spread_norm_mm, 2),
            "std_xyz_mm": [round(float(v), 2) for v in std_xyz],
            "robot_target_xyz_mm": [round(float(v), 2) for v in target_robot_xyz],
            "lock_time_s": round(float(lock_time_s), 3),
            "frames_inspected": int(max(self.current_cup_frames_inspected, len(grasp_samples))),
            "label_resets": int(self.current_cup_label_resets),
            "distance_resets": int(self.current_cup_distance_resets),
            "total_resets": int(self.current_cup_label_resets + self.current_cup_distance_resets),
            "ai_inference_ms": round(recent_ai_ms, 2),
            "nn_latency_ms": round(recent_e2e_ms, 2),
            "nn_fps": round(recent_fps, 2),
        }
        self.lock_records.append(lock_info)

        # Reset per-cup window counters for the next cup
        self.window_start_perf = None
        self.current_cup_label_resets = 0
        self.current_cup_distance_resets = 0
        self.current_cup_frames_inspected = 0
        return lock_info

    def log_cup_event(
        self,
        cup_no,
        lock_info,
        chosen_slot,
        slot_coords,
        normal_vec,
        pick_attempts,
        retry_history,
        placed_ok,
        verify_polls,
        left_arm_time_s,
        right_arm_time_s,
        robot_cycle_time_s,
    ):
        """
        Record cup execution metrics and immediately rewrite the full report file
        (Sections 1-7) so the summary tables are always up to date on disk.
        """
        lock_info = lock_info or {}
        cam_xyz = lock_info.get("cam_median_xyz_mm", [0.0, 0.0, 0.0])
        spread_xyz = lock_info.get("spread_xyz_mm", [0.0, 0.0, 0.0])
        std_xyz = lock_info.get("std_xyz_mm", [0.0, 0.0, 0.0])
        rob_xyz = lock_info.get("robot_target_xyz_mm", [0.0, 0.0, 0.0])
        label = lock_info.get("label", "unknown")
        conf = lock_info.get("median_conf_pct", 0.0)
        handover_completed = float(right_arm_time_s) > 0.01
        first_try_ok = (int(pick_attempts) == 1) and handover_completed

        record = {
            "cup_no": int(cup_no),
            "timestamp": lock_info.get("timestamp", datetime.now().strftime("%Y-%m-%d %H:%M:%S")),
            "model": self.model_name,
            "label": label,
            "conf_pct": conf,
            "cam_xyz_mm": cam_xyz,
            "cam_distance_mm": lock_info.get("cam_distance_mm", 0.0),
            "spread_xyz_mm": spread_xyz,
            "spread_norm_mm": lock_info.get("spread_norm_mm", 0.0),
            "std_xyz_mm": std_xyz,
            "lock_time_s": lock_info.get("lock_time_s", 0.0),
            "frames_inspected": lock_info.get("frames_inspected", self.grasp_frame_count),
            "label_resets": lock_info.get("label_resets", 0),
            "distance_resets": lock_info.get("distance_resets", 0),
            "total_resets": lock_info.get("total_resets", 0),
            "ai_inference_ms": lock_info.get("ai_inference_ms", 0.0),
            "nn_latency_ms": lock_info.get("nn_latency_ms", 0.0),
            "nn_fps": lock_info.get("nn_fps", 0.0),
            "rob_xyz_mm": rob_xyz,
            "normal_vec": [round(float(v), 3) for v in (normal_vec or [0, 0, 0])],
            "chosen_slot": str(chosen_slot),
            "slot_coords_mm": [round(float(v), 1) for v in (slot_coords or [0, 0, 0])],
            "pick_attempts": int(pick_attempts),
            "grasp_retries": max(0, int(pick_attempts) - 1),
            "first_try_ok": bool(first_try_ok),
            "handover_completed": bool(handover_completed),
            "retry_history": list(retry_history or []),
            "placed_ok": placed_ok,
            "verify_polls": int(verify_polls),
            "left_arm_time_s": round(float(left_arm_time_s), 3),
            "right_arm_time_s": round(float(right_arm_time_s), 3),
            "robot_cycle_time_s": round(float(robot_cycle_time_s), 3),
        }
        self.cup_records.append(record)

        # Immediately rewrite the full report (with updated Sections 4-7 summary tables!)
        self._rewrite_full_report_file()

        # Append row to cumulative Report_Cups_Log.csv
        write_header = not os.path.exists(self.cups_csv_path)
        with open(self.cups_csv_path, "a", newline="", encoding="utf-8") as csv_f:
            writer = csv.writer(csv_f)
            if write_header:
                writer.writerow([
                    "run_id", "timestamp", "model", "cup_no", "label", "conf_pct",
                    "cam_x_mm", "cam_y_mm", "cam_z_mm", "cam_dist_mm",
                    "spread_x_mm", "spread_y_mm", "spread_z_mm", "spread_norm_mm",
                    "lock_time_s", "frames_inspected", "label_resets", "distance_resets",
                    "ai_inference_ms", "camera_to_pc_latency_ms", "nn_fps",
                    "rob_x_mm", "rob_y_mm", "rob_z_mm",
                    "slot_name", "slot_x_mm", "slot_y_mm", "slot_z_mm",
                    "pick_attempts", "grasp_retries", "first_try_ok", "placed_ok",
                    "left_arm_pick_handover_s", "right_arm_place_s", "robot_cycle_time_s",
                ])
            slot_xyz = record["slot_coords_mm"]
            writer.writerow([
                self.run_id, record["timestamp"], self.model_name, cup_no, label, conf,
                cam_xyz[0], cam_xyz[1], cam_xyz[2], record["cam_distance_mm"],
                spread_xyz[0], spread_xyz[1], spread_xyz[2], record["spread_norm_mm"],
                record["lock_time_s"], record["frames_inspected"],
                record["label_resets"], record["distance_resets"],
                record["ai_inference_ms"], record["nn_latency_ms"], record["nn_fps"],
                rob_xyz[0], rob_xyz[1], rob_xyz[2],
                chosen_slot, slot_xyz[0], slot_xyz[1], slot_xyz[2],
                record["pick_attempts"], record["grasp_retries"], record["first_try_ok"],
                record["placed_ok"],
                record["left_arm_time_s"], record["right_arm_time_s"], record["robot_cycle_time_s"],
            ])

    def finalize_report(self):
        """Write final visual tables to the report file and append run-summary CSV row."""
        if self._finalized:
            return self.report_txt_path
        self._finalized = True

        self._rewrite_full_report_file()

        end_dt = datetime.now()
        session_duration_s = (end_dt - self.start_dt).total_seconds()
        det_rate_pct = (
            (100.0 * self.frames_with_detections / self.total_frames)
            if self.total_frames > 0
            else 0.0
        )

        all_confs = [c for vals in self.class_confidences.values() for c in vals]
        write_run_header = not os.path.exists(self.runs_csv_path)
        with open(self.runs_csv_path, "a", newline="", encoding="utf-8") as csv_f:
            writer = csv.writer(csv_f)
            if write_run_header:
                writer.writerow([
                    "run_id", "model", "duration_s", "total_frames", "detection_rate_pct",
                    "ai_inference_mean_ms", "ai_inference_std_ms",
                    "camera_to_pc_mean_ms", "camera_to_pc_std_ms", "camera_to_pc_p95_ms",
                    "nn_fps_mean", "nn_fps_std",
                    "mean_conf_pct", "std_conf_pct",
                    "label_flicker_resets", "distance_jump_resets",
                    "mean_lock_time_s", "mean_3d_spread_norm_mm",
                    "cups_count", "first_try_success_rate_pct", "total_grasp_retries",
                    "mean_left_arm_s", "mean_right_arm_s",
                    "mean_robot_cycle_s", "std_robot_cycle_s",
                    "table_xy_rms_mm", "tested_variable", "conclusion",
                ])
            n_cups = len(self.cup_records)
            completed_cups = [c for c in self.cup_records if c.get("handover_completed", True)]
            timing_source = completed_cups if completed_cups else self.cup_records
            first_try_rate = (
                100.0 * sum(1 for c in self.cup_records if c["first_try_ok"]) / n_cups
                if n_cups > 0
                else 0.0
            )
            writer.writerow([
                self.run_id,
                self.model_name,
                round(session_duration_s, 2),
                self.total_frames,
                round(det_rate_pct, 2),
                round(_safe_mean(self.on_chip_inference_ms), 2),
                round(_safe_std(self.on_chip_inference_ms), 2),
                round(_safe_mean(self.cam_to_pc_latency_ms), 2),
                round(_safe_std(self.cam_to_pc_latency_ms), 2),
                round(_safe_p95(self.cam_to_pc_latency_ms), 2),
                round(_safe_mean(self.nn_fps_list), 2),
                round(_safe_std(self.nn_fps_list), 2),
                round(_safe_mean(all_confs), 2),
                round(_safe_std(all_confs), 2),
                self.total_label_flicker_resets,
                self.total_distance_jump_resets,
                round(_safe_mean([r["lock_time_s"] for r in self.lock_records]), 3),
                round(_safe_mean([r["spread_norm_mm"] for r in self.lock_records]), 2),
                n_cups,
                round(first_try_rate, 1),
                sum(c["grasp_retries"] for c in self.cup_records),
                round(_safe_mean([c["left_arm_time_s"] for c in timing_source]), 3),
                round(_safe_mean([c["right_arm_time_s"] for c in timing_source]), 3),
                round(_safe_mean([c["robot_cycle_time_s"] for c in timing_source]), 3),
                round(_safe_std([c["robot_cycle_time_s"] for c in timing_source]), 3),
                round(self.cal_metrics.get("table_xy_rms_mm", 0.0), 3),
                self.tested_variable,
                self.conclusion_note,
            ])

        print("\n" + "=" * 78)
        print(f"[REPORT SAVED] Full Visual Report : {os.path.abspath(self.report_txt_path)}")
        print(f"[REPORT SAVED] Per-Cup CSV Table  : {os.path.abspath(self.cups_csv_path)}")
        print(f"[REPORT SAVED] Model Summary CSV  : {os.path.abspath(self.runs_csv_path)}")
        print("=" * 78 + "\n")
        return self.report_txt_path
