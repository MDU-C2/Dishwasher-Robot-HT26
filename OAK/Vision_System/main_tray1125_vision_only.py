"""Vision-only OAK-D Pro 3D YOLO tester (1125) — NO RobotStudio or 2D tray camera required.

Reuses the exact camera_setup, fixed HT_New_Run2_Mean, X/Z compensation,
workspace range filter, target-holding logic, 5-frame median, Gripper/handle
filtering, and stretched-1080p [DOT] projection from main_tray1125.py.

Interactive hotkeys in the RGB window:
  1-5 : Hot-switch between the 5 YOLO models on OAK-D Pro without restarting
  s   : Save raw + annotated snapshot PNGs to ./snapshots/
  r   : Reset 5-frame grasp window and held target
  q   : Quit
"""

from __future__ import annotations

import os
import time
from datetime import datetime
from pathlib import Path

import cv2 as cv
import depthai as dai
import numpy as np

import camera_setup as cs
import main_tray1125 as mt

DIR = Path(__file__).resolve().parent
SNAPSHOT_DIR = DIR / "snapshots"

ACTIVE_HT_NAME = "raw / HT_New_Run2_Mean"
ROTATION_MATRIX = np.array(
    [
        [-0.002597731108439694, -0.3876317471376878, 0.9218106532276962],
        [-0.9999744383587065, -0.005133658742591244, -0.004976763717912172],
        [0.00666141293402215, -0.9218000185283671, -0.3876085027689978],
    ],
    dtype=np.float64,
)
TRANSLATION_VECTOR = np.array(
    [-84.04769438994447, 10.90679989710399, 319.0687334399627],
    dtype=np.float64,
)
ORIENTATION_MAP = {
    "Back": [0.0, 0.0, 1.0],
    "Front": [0.0, 0.0, -1.0],
    "left_side": [1.0, 0.0, 0.0],
    "right_side": [-1.0, 0.0, 0.0],
    "upright": [0.0, 1.0, 0.0],
    "upside_down": [0.0, -1.0, 0.0],
}


def compute_approach_normal(label: str, rotation_matrix: np.ndarray) -> np.ndarray | None:
    vec = ORIENTATION_MAP.get(label)
    if vec is None:
        return None
    norm = np.matmul(rotation_matrix, vec)
    if (abs(vec[1]) < abs(vec[2])) or (abs(vec[1]) < abs(vec[0])):
        norm = norm - np.array([0.0, 0.0, float(np.dot(norm, [0.0, 0.0, 1.0]))])
        n = float(np.linalg.norm(norm))
        if n > 0:
            norm = norm / n
    else:
        norm = np.array([0.0, 0.0, vec[1] / abs(vec[1])], dtype=np.float64)
    return norm


def save_snapshot(model_choice: str, raw_frame: np.ndarray, vis_frame: np.ndarray) -> tuple[Path, Path]:
    SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    raw_path = SNAPSHOT_DIR / f"{ts}_{model_choice}_raw.png"
    vis_path = SNAPSHOT_DIR / f"{ts}_{model_choice}_vis.png"
    cv.imwrite(str(raw_path), raw_frame)
    cv.imwrite(str(vis_path), vis_frame)
    print(f"[SNAP] Saved {vis_path.name} and {raw_path.name}")
    return raw_path, vis_path


def run_single_model_session(model_choice: str, homogeneous: np.ndarray) -> str | None:
    """Run live OAK-D Pro preview for model_choice.

    Returns:
      - next model name string if user pressed 1-5 to hot-switch
      - None if user pressed q / ESC to exit
    """
    pipeline, labels = cs.camera_setup(model_choice)
    menu_id = cs.DETECTION_MODELS[model_choice]["menu_id"]
    print(f"[VISION-ONLY] Running [{menu_id}] {model_choice} (no RobotStudio, no 2D tray camera).")
    print("[KEYS] 1-5: switch model | s: save snapshot | r: reset 5-frame window | q/ESC: quit")

    first_run = True
    grasp_samples = []
    held_xyz = None
    missed_frames = 0
    fps_ema = 0.0
    last_t = time.perf_counter()

    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        projection = None

        cv.namedWindow("RGB (Vision Only)", cv.WINDOW_NORMAL)

        while True:
            in_rgb = q_rgb.get()
            in_depth = q_depth.get()
            in_dets = q_det.get()

            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            if dt > 0:
                inst_fps = 1.0 / dt
                fps_ema = inst_fps if fps_ema == 0.0 else (0.85 * fps_ema + 0.15 * inst_fps)

            raw_frame = in_rgb.getCvFrame()
            frame = raw_frame.copy()
            if projection is None:
                projection = mt.load_preview_projection(
                    device, frame.shape[1], frame.shape[0], model_choice,
                )
            camera_matrix, distortion = projection
            _depth_frame = in_depth.getFrame()
            detections = in_dets.detections

            selected_target, held_xyz, missed_frames, keep_samples = mt.track_selected_target(
                detections, labels, homogeneous, held_xyz, missed_frames,
            )

            accept_grasp_sample = not first_run
            if first_run:
                time.sleep(0.5)
                first_run = False

            grasp_samples = mt.update_grasp_samples(
                grasp_samples,
                selected_target,
                labels,
                keep_samples,
                accept_grasp_sample,
                busy=False,
            )

            for det in detections:
                label = str(det.label)
                if 0 <= det.label < len(labels):
                    label = labels[det.label]
                if label == "handle":
                    continue

                conf = int(det.confidence * 100)
                x1 = int(det.xmin * frame.shape[1])
                y1 = int(det.ymin * frame.shape[0])
                x2 = int(det.xmax * frame.shape[1])
                y2 = int(det.ymax * frame.shape[0])

                if label in mt.NON_CUP_LABELS:
                    cv.rectangle(frame, (x1, y1), (x2, y2), (128, 128, 128), 1)
                    cv.putText(
                        frame,
                        f"{label} ({conf}%) [ignored]",
                        (x1 + 5, y1 + 20),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (200, 200, 200),
                        1,
                    )
                    continue

                coords = det.spatialCoordinates
                robot_xyz = mt.robot_xyz_from_relations(
                    coords.x, coords.y, coords.z, homogeneous,
                )
                if not mt.is_within_pickup_work_range(robot_xyz):
                    box_color = (128, 128, 128)
                else:
                    box_color = (0, 0, 255) if det is selected_target else (0, 255, 0)
                cv.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

                cv.putText(frame, f"{label} ({conf}%)", (x1 + 5, y1 + 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv.putText(frame, f"X: {int(coords.x)} mm", (x1 + 5, y1 + 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv.putText(frame, f"Y: {int(coords.y)} mm", (x1 + 5, y1 + 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv.putText(frame, f"Z: {int(coords.z)} mm", (x1 + 5, y1 + 65), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                pixel = mt.project_spatial_pixel(
                    coords, camera_matrix, distortion, frame.shape[:2],
                )
                if pixel is not None:
                    mt.draw_spatial_dot(frame, pixel, det is selected_target)

                cv.putText(
                    frame,
                    f"R: {robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}",
                    (x1 + 5, y1 + 80),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 255),
                    1,
                )
                if det is selected_target and grasp_samples:
                    median_xyz = mt.median_grasp_xyz(grasp_samples)
                    median_robot = mt.robot_xyz_from_relations(
                        float(median_xyz[0]),
                        float(median_xyz[1]),
                        float(median_xyz[2]),
                        homogeneous,
                    )
                    cv.putText(
                        frame,
                        f"M{len(grasp_samples)}: {median_robot[0]:.1f}, {median_robot[1]:.1f}, {median_robot[2]:.1f}",
                        (x1 + 5, y1 + 95),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        1,
                    )

            header = (
                f"[{menu_id}] {model_choice} | {fps_ema:4.1f} FPS | "
                f"grasp {len(grasp_samples)}/{mt.GRASP_FRAME_COUNT} | DRY-RUN"
            )
            cv.putText(frame, header, (10, 26), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv.putText(
                frame,
                "Keys: 1-5 switch | s snap | r reset | q quit",
                (10, frame.shape[0] - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.45,
                (220, 220, 220),
                1,
            )

            if len(grasp_samples) >= mt.GRASP_FRAME_COUNT:
                stacked = np.vstack([s["xyz"] for s in grasp_samples])
                median_xyz = np.median(stacked, axis=0)
                spread = np.max(stacked, axis=0) - np.min(stacked, axis=0)
                label = grasp_samples[-1]["label"]
                conf = int(np.median([s["conf"] for s in grasp_samples]))
                target_xyz = mt.robot_xyz_from_relations(
                    float(median_xyz[0]),
                    float(median_xyz[1]),
                    float(median_xyz[2]),
                    homogeneous,
                )
                norm = compute_approach_normal(label, ROTATION_MATRIX)
                norm_str = np.array2string(norm, precision=3) if norm is not None else "None"
                print(
                    f"[DRY-RUN 5/5] [{menu_id}] {model_choice} | {label} ({conf}%) | "
                    f"cam=({median_xyz[0]:.1f}, {median_xyz[1]:.1f}, {median_xyz[2]:.1f}) | "
                    f"spread=({spread[0]:.1f}, {spread[1]:.1f}, {spread[2]:.1f}) | "
                    f"robot={target_xyz} | normal={norm_str}"
                )
                grasp_samples = []

            cv.imshow("RGB (Vision Only)", frame)
            key = cv.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                return None
            if key == ord("s"):
                save_snapshot(model_choice, raw_frame, frame)
            if key == ord("r"):
                grasp_samples = []
                held_xyz = None
                missed_frames = 0
                print("[RESET] Cleared 5-frame window and held target.")
            if ord("1") <= key <= ord("5"):
                next_model = cs.resolve_model_name(chr(key))
                if next_model and next_model != model_choice:
                    print(f"\n[SWITCH] Hot-switching from {model_choice} -> {next_model} ...")
                    return next_model


def run() -> None:
    homogeneous = mt.build_homogeneous(ROTATION_MATRIX, TRANSLATION_VECTOR)
    print(f"[HT] Fixed transform: {ACTIVE_HT_NAME} (X and Z compensation enabled; RobotStudio disabled).")
    model_choice = mt.choose_detection_model()
    while model_choice is not None:
        model_choice = run_single_model_session(model_choice, homogeneous)
    cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        run()
    finally:
        cv.destroyAllWindows()
