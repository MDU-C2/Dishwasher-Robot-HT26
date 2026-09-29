from pathlib import Path
from datetime import datetime
from glob import glob
import argparse
import json
import time
import sys

import cv2
import depthai as dai
import numpy as np


# 1. The marker dictionary must match the printed markers.
DICTIONARY_NAME = "DICT_6X6_250"
dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_6X6_250
)
detector = cv2.aruco.ArucoDetector(
    dictionary, cv2.aruco.DetectorParameters()
)
output_dir = Path(__file__).resolve().parent / "evidence"
calibration_dir = Path(__file__).resolve().parent / "calibration"
repeatability_dir = Path(__file__).resolve().parent / "repeatability"

# Single-marker pose: one small printed ID 23, black-square side in millimetres.
TARGET_MARKER_ID = 23
MARKER_LENGTH_MM = 24.0
MARKER_LENGTH_IS_APPROXIMATE = True
MARKER_LENGTH_NOTE = (
    "Approximate design size of the black outer square (24.0 mm). "
    "The print is 30x30 mm including a 3 mm white border on each side. "
    "A user measurement of the black square was about 23.5-23.9 mm. "
    "This is not a verified metrology value; do not treat a pose reading as accuracy."
)
PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 360
PNP_FLAGS = cv2.SOLVEPNP_IPPE_SQUARE
PNP_METHOD_NAME = "cv2.SOLVEPNP_IPPE_SQUARE"
AXIS_LENGTH_MM = MARKER_LENGTH_MM * 0.75
SAMPLE_DURATION_S = 10.0
DISTORTION_NAMES = [
    "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6",
    "s1", "s2", "s3", "s4", "tau_x", "tau_y",
]
KNOWN_CASES = (
    "baseline_1",
    "baseline_2",
    "baseline_3",
    "right_50mm",
    "away_100mm",
    "return_origin",
    "working_distance",
    "tilt_left",
    "tilt_right",
)
STD_DEFINITION = "sample standard deviation, ddof=1 (divide by n-1)"
REPROJECTION_RMSE_DEFINITION = (
    "sqrt(mean(sum((projected-observed)**2, axis=1))) in pixels; "
    "per-corner Euclidean distances, then RMS"
)
REPROJECTION_RMSE_LIMITATION = (
    "A low reprojection RMSE means the estimated pose fits the four observed "
    "corners in the image. It does not prove that the millimetre pose is accurate."
)
PLACEMENT_PROTOCOL = {
    "camera": "fixed for the whole campaign",
    "marker": "one small printed ID 23, taped flat and fixed while a case is sampled",
    "marker_length_mm": MARKER_LENGTH_MM,
    "marker_length_is_approximate": True,
    "right_50mm_and_away_100mm": (
        "both relative to the same origin pose; reposition before switching cases"
    ),
    "not_a_metrology_setup": True,
}

# Object-point order matches OpenCV ArUco corners 0-1-2-3 and SOLVEPNP_IPPE_SQUARE:
# 0 top-left, 1 top-right, 2 bottom-right, 3 bottom-left of the marker pattern.
# Marker frame: origin at the black-square centre, X right, Y up, Z out of the print.
_MARKER_HALF_MM = MARKER_LENGTH_MM / 2.0
MARKER_OBJECT_POINTS_MM = np.array(
    [
        [-_MARKER_HALF_MM, _MARKER_HALF_MM, 0.0],
        [_MARKER_HALF_MM, _MARKER_HALF_MM, 0.0],
        [_MARKER_HALF_MM, -_MARKER_HALF_MM, 0.0],
        [-_MARKER_HALF_MM, -_MARKER_HALF_MM, 0.0],
    ],
    dtype=np.float64,
)

INTERPRETER = sys.executable
SCRIPT_PATH = str(Path(__file__).resolve())


def save_camera_parameters(camera_matrix, distortion, device_mxid):
    """Save device-stored RGB calibration, not camera-to-robot calibration."""
    coefficients = distortion.reshape(-1)
    if camera_matrix.shape != (3, 3) or coefficients.size != 14:
        raise ValueError("Expected a 3x3 camera matrix and 14 distortion coefficients")
    if not (np.isfinite(camera_matrix).all() and np.isfinite(coefficients).all()):
        raise ValueError("Camera parameters contain non-finite values")
    saved_at = datetime.now().astimezone()
    record = {
        "schema_version": 1,
        "label": "OAK RGB camera intrinsics and distortion - 640x360",
        "source": "Device-stored calibration read at program startup; not a new calibration",
        "saved_at": saved_at.isoformat(timespec="milliseconds"),
        "device_mxid": device_mxid,
        "camera_socket": "RGB (CAM_A)",
        "image_configuration": image_configuration(),
        "intrinsics_px": {
            "fx": float(camera_matrix[0, 0]),
            "fy": float(camera_matrix[1, 1]),
            "cx": float(camera_matrix[0, 2]),
            "cy": float(camera_matrix[1, 2]),
        },
        "camera_matrix": camera_matrix.tolist(),
        "distortion_model": "OpenCV 14-coefficient model",
        "distortion_order": DISTORTION_NAMES,
        "distortion_coefficients": coefficients.tolist(),
        "distortion_named": dict(zip(DISTORTION_NAMES, coefficients.tolist())),
        "opencv_version": cv2.__version__,
        "depthai_version": dai.__version__,
        "scope": "Camera imaging parameters only; no marker pose or robot transform",
    }
    calibration_dir.mkdir(exist_ok=True)
    path = calibration_dir / (
        f"oak_rgb_640x360_intrinsics_{saved_at:%Y%m%d_%H%M%S_%f}.json"
    )
    with path.open("x", encoding="utf-8") as file:
        json.dump(record, file, indent=2, ensure_ascii=False, allow_nan=False)
    print("Saved camera parameters:", path)
    return path


def image_configuration():
    return {
        "sensor_resolution": "1920x1080",
        "preview_size_px": [PREVIEW_WIDTH, PREVIEW_HEIGHT],
        "preview_keep_aspect_ratio": True,
        "host_rotation_degrees": 0,
        "color_order": "BGR",
        "fps_setting": 30,
    }


def as_float_list(values):
    if values is None:
        return None
    return [float(v) for v in np.asarray(values, dtype=np.float64).reshape(-1)]


def empty_pose(status, reason, extra=None):
    """Pose payload with explicit nulls. Never fill failed results with zeros."""
    record = {
        "status": status,
        "reason": reason,
        "target_id": TARGET_MARKER_ID,
        "marker_length_mm": MARKER_LENGTH_MM,
        "marker_length_is_approximate": MARKER_LENGTH_IS_APPROXIMATE,
        "marker_length_note": MARKER_LENGTH_NOTE,
        "units": "mm",
        "transform_direction": (
            "marker frame to camera frame; tvec is the marker origin in camera coordinates"
        ),
        "camera_frame": "X right, Y down, Z forward along the optical axis",
        "marker_frame": (
            "origin at black-square centre; X right; Y up (OpenCV ArUco); "
            "Z out of the printed face"
        ),
        "pnp_method": PNP_METHOD_NAME,
        "rvec_note": "Rodrigues rotation vector in radians, not Euler angles",
        "planar_ambiguity_note": (
            "A planar square can have two plausible poses. IPPE_SQUARE returns one "
            "solution, which can jump under tilt, noise, or corner error. Overlay "
            "values are not filtered and are not a verified accurate pose."
        ),
        "target_count": 0,
        "corners_px": None,
        "rvec": None,
        "tvec_mm": None,
        "position_camera_mm": None,
        "reprojection_rmse_px": None,
        "side_lengths_px": None,
        "reprojection_rmse_definition": REPROJECTION_RMSE_DEFINITION,
        "reprojection_rmse_limitation": REPROJECTION_RMSE_LIMITATION,
    }
    if extra:
        record.update(extra)
    return record


def marker_side_lengths_px(corners_px):
    points = np.asarray(corners_px, dtype=np.float64).reshape(-1, 2)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        return None
    lengths = [
        float(np.linalg.norm(points[(index + 1) % 4] - points[index]))
        for index in range(4)
    ]
    return lengths


def corner_reprojection_rmse_px(rvec, tvec, corners_px, camera_matrix, distortion):
    observed = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    projected, _ = cv2.projectPoints(
        MARKER_OBJECT_POINTS_MM.reshape(4, 1, 3),
        np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        camera_matrix,
        distortion,
    )
    projected = projected.reshape(4, 2)
    if not (np.isfinite(observed).all() and np.isfinite(projected).all()):
        return None
    residual = projected - observed
    return float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))


def relative_rotation_angle_deg(rvec_a, rvec_b):
    """Angle of the relative rotation matrix, not a difference of rvec components."""
    rotation_a, _ = cv2.Rodrigues(
        np.asarray(rvec_a, dtype=np.float64).reshape(3, 1)
    )
    rotation_b, _ = cv2.Rodrigues(
        np.asarray(rvec_b, dtype=np.float64).reshape(3, 1)
    )
    relative = rotation_a.T @ rotation_b
    cosine = (np.trace(relative) - 1.0) / 2.0
    cosine = float(np.clip(cosine, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def estimate_single_target_pose(
    corners,
    ids,
    camera_matrix,
    distortion,
    image_size_px,
):
    """Estimate one ID 23 pose relative to the camera from this frame only.

    Purpose: recover the rigid transform from the marker frame to the camera frame.
    Inputs: ArUco corners/ids, 640x360 camera matrix, 14 OpenCV distortion coefficients,
    and the current image size.
    Output: a pose dict. status is ok only for exactly one finite in-front ID 23.
    Failed cases keep rvec/tvec/position as null so a previous frame cannot leak through.
    """
    width, height = image_size_px
    if (width, height) != (PREVIEW_WIDTH, PREVIEW_HEIGHT):
        return empty_pose(
            "invalid",
            "Image size does not match the 640x360 intrinsics used for PnP",
            extra={"image_size_px": [int(width), int(height)]},
        )

    if ids is None or len(ids) == 0 or corners is None:
        return empty_pose("no_target", "No ArUco marker in this frame")

    target_corners = [
        np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
        for marker_corners, marker_id in zip(corners, ids.flatten())
        if int(marker_id) == TARGET_MARKER_ID
    ]
    target_count = len(target_corners)
    if target_count == 0:
        return empty_pose(
            "no_target",
            f"No marker with ID {TARGET_MARKER_ID} in this frame",
        )
    if target_count > 1:
        return empty_pose(
            "not_unique",
            f"Found {target_count} markers with ID {TARGET_MARKER_ID}; pose not chosen",
            extra={
                "target_count": target_count,
                "corners_px": [points.tolist() for points in target_corners],
            },
        )

    image_points = target_corners[0].reshape(4, 1, 2)
    object_points = MARKER_OBJECT_POINTS_MM.reshape(4, 1, 3)
    try:
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=PNP_FLAGS,
        )
    except cv2.error as error:
        return empty_pose(
            "invalid",
            f"solvePnP raised: {error}",
            extra={"target_count": 1, "corners_px": target_corners[0].tolist()},
        )

    if not success:
        return empty_pose(
            "invalid",
            "solvePnP did not return a solution",
            extra={"target_count": 1, "corners_px": target_corners[0].tolist()},
        )

    rvec = np.asarray(rvec, dtype=np.float64).reshape(3)
    tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
    if not (np.isfinite(rvec).all() and np.isfinite(tvec).all()):
        return empty_pose(
            "invalid",
            "solvePnP returned a non-finite rvec or tvec",
            extra={"target_count": 1, "corners_px": target_corners[0].tolist()},
        )
    if tvec[2] <= 0:
        return empty_pose(
            "invalid",
            "Estimated marker origin is not in front of the camera (Z <= 0 mm)",
            extra={"target_count": 1, "corners_px": target_corners[0].tolist()},
        )

    pose = empty_pose("ok", "Exactly one ID 23 with a finite in-front PnP solution")
    pose["target_count"] = 1
    pose["corners_px"] = target_corners[0].tolist()
    pose["rvec"] = as_float_list(rvec)
    pose["tvec_mm"] = as_float_list(tvec)
    pose["position_camera_mm"] = {
        "x": float(tvec[0]),
        "y": float(tvec[1]),
        "z": float(tvec[2]),
    }
    pose["side_lengths_px"] = marker_side_lengths_px(target_corners[0])
    pose["reprojection_rmse_px"] = corner_reprojection_rmse_px(
        rvec, tvec, target_corners[0], camera_matrix, distortion
    )
    return pose


def pose_overlay_text(pose):
    if pose["status"] == "ok":
        position = pose["position_camera_mm"]
        return (
            f"Pose ID{TARGET_MARKER_ID}: "
            f"X={position['x']:.1f} Y={position['y']:.1f} Z={position['z']:.1f} mm"
        )
    if pose["status"] == "not_unique":
        return f"ID {TARGET_MARKER_ID} not unique; no pose"
    if pose["status"] == "no_target":
        return f"No valid pose (no ID {TARGET_MARKER_ID})"
    return "No valid pose"


def pose_save_fields(
    pose,
    camera_matrix,
    distortion,
    image_size_px,
    frame_id,
    received_at,
    factory_calibration_size_px,
    distortion_model_name,
):
    record = dict(pose)
    record["received_at"] = received_at
    record["frame_id"] = int(frame_id)
    record["image_size_px"] = [int(image_size_px[0]), int(image_size_px[1])]
    record["intrinsics_requested_px"] = [PREVIEW_WIDTH, PREVIEW_HEIGHT]
    record["factory_calibration_size_px"] = factory_calibration_size_px
    record["camera_matrix"] = np.asarray(camera_matrix, dtype=np.float64).tolist()
    record["distortion_model_name"] = distortion_model_name
    record["distortion_order"] = DISTORTION_NAMES
    record["distortion_coefficients"] = as_float_list(distortion)
    return record


def draw_pose_axes(display, pose, camera_matrix, distortion):
    if pose["status"] != "ok":
        return
    rvec = np.asarray(pose["rvec"], dtype=np.float64).reshape(3, 1)
    tvec = np.asarray(pose["tvec_mm"], dtype=np.float64).reshape(3, 1)
    cv2.drawFrameAxes(
        display,
        camera_matrix,
        distortion,
        rvec,
        tvec,
        AXIS_LENGTH_MM,
    )


def numeric_series_stats(values):
    finite = [
        float(value) for value in values
        if value is not None and np.isfinite(value)
    ]
    count = len(finite)
    if count == 0:
        return {
            "n": 0,
            "mean": None,
            "std": None,
            "min": None,
            "max": None,
            "range": None,
            "std_definition": STD_DEFINITION,
            "unavailable_reason": "No valid samples",
        }
    array = np.asarray(finite, dtype=np.float64)
    stats = {
        "n": count,
        "mean": float(array.mean()),
        "std": None,
        "min": float(array.min()),
        "max": float(array.max()),
        "range": float(array.max() - array.min()),
        "std_definition": STD_DEFINITION,
        "unavailable_reason": None,
    }
    if count < 2:
        stats["unavailable_reason"] = (
            "Need at least 2 valid samples for standard deviation"
        )
    else:
        stats["std"] = float(array.std(ddof=1))
    return stats


def start_sampling_session(case_name):
    now = time.monotonic()
    return {
        "case": case_name,
        "t0": now,
        "started_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "samples": [],
        "last_processed_monotonic": None,
        "max_processed_frame_interval_s": 0.0,
    }


def sampling_elapsed_s(session, now=None):
    if now is None:
        now = time.monotonic()
    return float(now - session["t0"])


def sampling_remaining_s(session, now=None):
    return max(0.0, SAMPLE_DURATION_S - sampling_elapsed_s(session, now))


def sampling_timed_out(session, now=None):
    return sampling_elapsed_s(session, now) >= SAMPLE_DURATION_S


def append_sample(session, pose, frame_id, received_at, monotonic_s):
    previous = session["last_processed_monotonic"]
    if previous is None:
        interval = float(monotonic_s - session["t0"])
    else:
        interval = float(monotonic_s - previous)
    session["max_processed_frame_interval_s"] = max(
        session["max_processed_frame_interval_s"], interval
    )
    session["last_processed_monotonic"] = monotonic_s
    single_corners = pose["corners_px"]
    if (
        pose["status"] != "ok"
        and isinstance(single_corners, list)
        and single_corners
        and isinstance(single_corners[0], list)
        and len(single_corners) == 4
        and not isinstance(single_corners[0][0], list)
    ):
        side_lengths = marker_side_lengths_px(single_corners)
    else:
        side_lengths = pose.get("side_lengths_px")

    session["samples"].append({
        "sample_index": len(session["samples"]),
        "frame_id": int(frame_id),
        "received_at": received_at,
        "monotonic_s": float(monotonic_s),
        "elapsed_s": float(monotonic_s - session["t0"]),
        "interval_since_previous_processed_s": interval,
        "pose_status": pose["status"],
        "pose_reason": pose["reason"],
        "position_camera_mm": pose["position_camera_mm"],
        "rvec": pose["rvec"],
        "corners_px": pose["corners_px"],
        "side_lengths_px": side_lengths,
        "reprojection_rmse_px": pose.get("reprojection_rmse_px"),
    })


def orientation_change_stats(samples):
    valid = [
        (index, sample) for index, sample in enumerate(samples)
        if sample["pose_status"] == "ok" and sample["rvec"] is not None
    ]
    method_note = (
        "Angle of the relative rotation matrix R_a^T R_b in degrees. "
        "This is not a subtraction of rvec components. "
        "Adjacent valid samples may have intervening invalid frames; "
        "those gaps are counted and are not treated as continuous high-rate samples."
    )
    empty = {
        "method": method_note,
        "n_pairs": 0,
        "mean": None,
        "std": None,
        "min": None,
        "max": None,
        "range": None,
        "std_definition": STD_DEFINITION,
        "unavailable_reason": "Need at least 2 valid poses",
    }
    if len(valid) < 2:
        return {
            "relative_to_first_valid_deg": dict(empty),
            "adjacent_valid_deg": dict(empty),
        }

    first_index, first_sample = valid[0]
    vs_first = [
        relative_rotation_angle_deg(first_sample["rvec"], sample["rvec"])
        for _, sample in valid[1:]
    ]
    adjacent_angles = []
    intervening_invalid_pairs = 0
    for (index_a, sample_a), (index_b, sample_b) in zip(valid, valid[1:]):
        intervening = index_b - index_a - 1
        if intervening > 0:
            intervening_invalid_pairs += 1
        adjacent_angles.append(
            relative_rotation_angle_deg(sample_a["rvec"], sample_b["rvec"])
        )

    vs_first_stats = numeric_series_stats(vs_first)
    vs_first_stats["method"] = method_note
    vs_first_stats["n_pairs"] = len(vs_first)
    vs_first_stats["first_valid_sample_index"] = first_index

    adjacent_stats = numeric_series_stats(adjacent_angles)
    adjacent_stats["method"] = method_note
    adjacent_stats["n_pairs"] = len(adjacent_angles)
    adjacent_stats["pairs_with_intervening_invalid_frames"] = intervening_invalid_pairs
    adjacent_stats["note"] = (
        "adjacent means successive valid samples in capture order, "
        "not necessarily consecutive processed frames"
    )
    return {
        "relative_to_first_valid_deg": vs_first_stats,
        "adjacent_valid_deg": adjacent_stats,
    }


def build_repeatability_summary(session, context, completed, interrupted_by, ended_at_monotonic):
    samples = session["samples"]
    actual_duration_s = float(ended_at_monotonic - session["t0"])
    tail_gap = 0.0
    if session["last_processed_monotonic"] is None:
        max_interval = actual_duration_s
    else:
        tail_gap = float(ended_at_monotonic - session["last_processed_monotonic"])
        max_interval = max(session["max_processed_frame_interval_s"], tail_gap)

    processed = len(samples)
    status_counts = {}
    for sample in samples:
        status_counts[sample["pose_status"]] = (
            status_counts.get(sample["pose_status"], 0) + 1
        )
    valid_count = status_counts.get("ok", 0)
    if processed == 0:
        valid_ratio = None
        valid_ratio_reason = "No processed frames were recorded"
    else:
        valid_ratio = valid_count / processed
        valid_ratio_reason = None

    xs = []
    ys = []
    zs = []
    rmses = []
    min_side = []
    for sample in samples:
        if sample["pose_status"] != "ok":
            continue
        position = sample["position_camera_mm"] or {}
        xs.append(position.get("x"))
        ys.append(position.get("y"))
        zs.append(position.get("z"))
        rmses.append(sample.get("reprojection_rmse_px"))
        sides = sample.get("side_lengths_px")
        if sides:
            min_side.append(min(sides))

    xyz = {
        "x": numeric_series_stats(xs),
        "y": numeric_series_stats(ys),
        "z": numeric_series_stats(zs),
        "units": "mm",
        "std_definition": STD_DEFINITION,
    }
    mean_xyz = None
    if (
        xyz["x"]["mean"] is not None
        and xyz["y"]["mean"] is not None
        and xyz["z"]["mean"] is not None
    ):
        mean_xyz = [xyz["x"]["mean"], xyz["y"]["mean"], xyz["z"]["mean"]]

    return {
        "schema_version": 1,
        "kind": "aruco_repeatability_summary",
        "case": session["case"],
        "sampling_completed": bool(completed),
        "interrupted_by": interrupted_by,
        "intended_duration_s": SAMPLE_DURATION_S,
        "actual_duration_s": actual_duration_s,
        "started_at": session["started_at"],
        "ended_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "processed_frame_count": processed,
        "valid_frame_count": valid_count,
        "invalid_status_counts": {
            status: count for status, count in status_counts.items()
            if status != "ok"
        },
        "valid_frame_ratio": valid_ratio,
        "valid_frame_ratio_note": (
            "valid_frame_count / processed_frame_count; this is not a time-based rate. "
            "Use max_processed_frame_interval_s to see stalled capture."
        ),
        "valid_frame_ratio_unavailable_reason": valid_ratio_reason,
        "max_processed_frame_interval_s": float(max_interval),
        "tail_gap_after_last_processed_s": float(tail_gap),
        "xyz_mm": xyz,
        "mean_xyz_mm": mean_xyz,
        "orientation_deg": orientation_change_stats(samples),
        "reprojection_rmse_px": numeric_series_stats(rmses),
        "reprojection_rmse_definition": REPROJECTION_RMSE_DEFINITION,
        "reprojection_rmse_limitation": REPROJECTION_RMSE_LIMITATION,
        "min_side_length_px": numeric_series_stats(min_side),
        "min_side_length_note": (
            "Smallest of the four observed marker sides per valid frame, in pixels. "
            "Use this to see whether the marker became too small at distance."
        ),
        "target_id": TARGET_MARKER_ID,
        "marker_length_mm": MARKER_LENGTH_MM,
        "marker_length_is_approximate": MARKER_LENGTH_IS_APPROXIMATE,
        "marker_length_note": MARKER_LENGTH_NOTE,
        "pnp_method": PNP_METHOD_NAME,
        "dictionary": DICTIONARY_NAME,
        "device_mxid": context["device_mxid"],
        "camera_matrix": np.asarray(context["camera_matrix"], dtype=np.float64).tolist(),
        "distortion_coefficients": as_float_list(context["distortion"]),
        "distortion_order": DISTORTION_NAMES,
        "distortion_model_name": context["distortion_model_name"],
        "factory_calibration_size_px": context["factory_calibration_size_px"],
        "image_configuration": image_configuration(),
        "placement_protocol": PLACEMENT_PROTOCOL,
        "opencv_version": cv2.__version__,
        "depthai_version": dai.__version__,
        "no_filtering": True,
        "not_a_pass_fail_accuracy_test": True,
    }


def write_json(path, payload):
    path.parent.mkdir(exist_ok=True)
    with path.open("x", encoding="utf-8") as file:
        json.dump(payload, file, indent=2, ensure_ascii=False, allow_nan=False)
    return path


def finish_sampling(session, context, completed, interrupted_by=None):
    ended = time.monotonic()
    summary = build_repeatability_summary(
        session, context, completed, interrupted_by, ended
    )
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    case_name = session["case"]
    samples_payload = {
        "schema_version": 1,
        "kind": "aruco_repeatability_samples",
        "case": case_name,
        "sampling_completed": bool(completed),
        "interrupted_by": interrupted_by,
        "started_at": session["started_at"],
        "target_id": TARGET_MARKER_ID,
        "marker_length_mm": MARKER_LENGTH_MM,
        "marker_length_is_approximate": MARKER_LENGTH_IS_APPROXIMATE,
        "marker_length_note": MARKER_LENGTH_NOTE,
        "pnp_method": PNP_METHOD_NAME,
        "dictionary": DICTIONARY_NAME,
        "device_mxid": context["device_mxid"],
        "camera_matrix": np.asarray(context["camera_matrix"], dtype=np.float64).tolist(),
        "distortion_coefficients": as_float_list(context["distortion"]),
        "distortion_order": DISTORTION_NAMES,
        "distortion_model_name": context["distortion_model_name"],
        "factory_calibration_size_px": context["factory_calibration_size_px"],
        "image_configuration": image_configuration(),
        "placement_protocol": PLACEMENT_PROTOCOL,
        "opencv_version": cv2.__version__,
        "depthai_version": dai.__version__,
        "samples": session["samples"],
    }
    samples_path = repeatability_dir / f"{stamp}_{case_name}_samples.json"
    summary_path = repeatability_dir / f"{stamp}_{case_name}_summary.json"
    write_json(samples_path, samples_payload)
    write_json(summary_path, summary)
    state = "complete" if completed else "incomplete"
    print(
        f"Saved {state} sampling for {case_name}:",
        summary_path,
        f"| processed={summary['processed_frame_count']}",
        f"| valid={summary['valid_frame_count']}",
    )
    return samples_path, summary_path, summary


def sampling_overlay_text(session, case_name):
    remaining = sampling_remaining_s(session)
    processed = len(session["samples"])
    valid = sum(1 for sample in session["samples"] if sample["pose_status"] == "ok")
    return (
        f"Sampling {case_name}  {remaining:.1f}s left  "
        f"n={processed} ok={valid}"
    )


def expand_json_inputs(patterns):
    files = []
    for pattern in patterns:
        matches = sorted(glob(pattern)) if any(char in pattern for char in "*?") else []
        if matches:
            files.extend(Path(item) for item in matches)
        else:
            files.append(Path(pattern))
    return files


def load_summary_from_path(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    kind = payload.get("kind")
    if kind == "aruco_repeatability_summary":
        return payload
    if kind == "aruco_repeatability_samples":
        session = {
            "case": payload["case"],
            "t0": 0.0,
            "started_at": payload.get("started_at"),
            "samples": payload.get("samples", []),
            "last_processed_monotonic": None,
            "max_processed_frame_interval_s": 0.0,
        }
        samples = session["samples"]
        if samples:
            session["t0"] = float(samples[0]["monotonic_s"] - samples[0]["elapsed_s"])
            session["last_processed_monotonic"] = float(samples[-1]["monotonic_s"])
            session["max_processed_frame_interval_s"] = max(
                float(sample["interval_since_previous_processed_s"])
                for sample in samples
            )
            ended = float(samples[-1]["monotonic_s"])
        else:
            ended = session["t0"]
        context = {
            "device_mxid": payload.get("device_mxid"),
            "camera_matrix": payload.get("camera_matrix"),
            "distortion": payload.get("distortion_coefficients"),
            "distortion_model_name": payload.get("distortion_model_name"),
            "factory_calibration_size_px": payload.get("factory_calibration_size_px"),
        }
        return build_repeatability_summary(
            session,
            context,
            payload.get("sampling_completed", False),
            payload.get("interrupted_by"),
            ended,
        )
    raise ValueError(f"Unrecognized sampling JSON kind in {path}: {kind}")


def displacement_between_summaries(from_summary, to_summary, nominal_mm):
    from_mean = from_summary.get("mean_xyz_mm")
    to_mean = to_summary.get("mean_xyz_mm")
    result = {
        "from_case": from_summary.get("case"),
        "to_case": to_summary.get("case"),
        "from_mean_xyz_mm": from_mean,
        "to_mean_xyz_mm": to_mean,
        "nominal_length_mm": None if nominal_mm is None else float(nominal_mm),
        "displacement_mm": None,
        "displacement_length_mm": None,
        "length_minus_nominal_mm": None,
        "note": (
            "Displacement is the 3D length between two case mean XYZ values. "
            "A user-provided nominal length is a setup target, not calibrated ground truth. "
            "This is not a true pose error."
        ),
        "unavailable_reason": None,
    }
    if from_mean is None or to_mean is None:
        result["unavailable_reason"] = (
            "Need mean XYZ from both cases; a case without valid poses cannot be compared"
        )
        return result
    delta = [
        float(to_mean[0] - from_mean[0]),
        float(to_mean[1] - from_mean[1]),
        float(to_mean[2] - from_mean[2]),
    ]
    length = float(np.linalg.norm(delta))
    result["displacement_mm"] = delta
    result["displacement_length_mm"] = length
    if nominal_mm is not None:
        result["length_minus_nominal_mm"] = length - float(nominal_mm)
    return result


def compact_case_row(summary):
    xyz = summary.get("xyz_mm", {})
    orientation = summary.get("orientation_deg", {})
    return {
        "case": summary.get("case"),
        "sampling_completed": summary.get("sampling_completed"),
        "interrupted_by": summary.get("interrupted_by"),
        "actual_duration_s": summary.get("actual_duration_s"),
        "processed_frame_count": summary.get("processed_frame_count"),
        "valid_frame_count": summary.get("valid_frame_count"),
        "valid_frame_ratio": summary.get("valid_frame_ratio"),
        "invalid_status_counts": summary.get("invalid_status_counts"),
        "max_processed_frame_interval_s": summary.get("max_processed_frame_interval_s"),
        "xyz_std_mm": {
            "x": (xyz.get("x") or {}).get("std"),
            "y": (xyz.get("y") or {}).get("std"),
            "z": (xyz.get("z") or {}).get("std"),
        },
        "xyz_range_mm": {
            "x": (xyz.get("x") or {}).get("range"),
            "y": (xyz.get("y") or {}).get("range"),
            "z": (xyz.get("z") or {}).get("range"),
        },
        "mean_xyz_mm": summary.get("mean_xyz_mm"),
        "orientation_relative_to_first_deg": {
            "mean": (orientation.get("relative_to_first_valid_deg") or {}).get("mean"),
            "std": (orientation.get("relative_to_first_valid_deg") or {}).get("std"),
            "range": (orientation.get("relative_to_first_valid_deg") or {}).get("range"),
        },
        "orientation_adjacent_valid_deg": {
            "mean": (orientation.get("adjacent_valid_deg") or {}).get("mean"),
            "std": (orientation.get("adjacent_valid_deg") or {}).get("std"),
            "range": (orientation.get("adjacent_valid_deg") or {}).get("range"),
            "pairs_with_intervening_invalid_frames": (
                orientation.get("adjacent_valid_deg") or {}
            ).get("pairs_with_intervening_invalid_frames"),
        },
        "reprojection_rmse_px": {
            "mean": (summary.get("reprojection_rmse_px") or {}).get("mean"),
            "std": (summary.get("reprojection_rmse_px") or {}).get("std"),
            "range": (summary.get("reprojection_rmse_px") or {}).get("range"),
        },
        "min_side_length_px_mean": (summary.get("min_side_length_px") or {}).get("mean"),
    }


def run_offline_summarize(paths, comparisons):
    summaries = []
    by_case = {}
    for path in expand_json_inputs(paths):
        if not path.is_file():
            raise FileNotFoundError(path)
        summary = load_summary_from_path(path)
        summaries.append(summary)
        case_name = summary.get("case")
        if case_name in by_case:
            print("Warning: multiple files for case", case_name, "; using", path)
        by_case[case_name] = summary

    displacement_rows = []
    for from_case, to_case, nominal_text in comparisons:
        if from_case not in by_case or to_case not in by_case:
            displacement_rows.append({
                "from_case": from_case,
                "to_case": to_case,
                "unavailable_reason": "One or both case names were not present in the loaded files",
                "note": "This is not a true pose error.",
            })
            continue
        nominal = None if nominal_text in (None, "", "none") else float(nominal_text)
        displacement_rows.append(
            displacement_between_summaries(by_case[from_case], by_case[to_case], nominal)
        )

    campaign = {
        "schema_version": 1,
        "kind": "aruco_measurement_capability_campaign",
        "built_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "source_files": [str(Path(path).resolve()) for path in expand_json_inputs(paths)],
        "cases": [compact_case_row(summary) for summary in summaries],
        "displacements": displacement_rows,
        "no_pass_fail_threshold": True,
        "not_a_true_error_without_calibrated_reference": True,
    }
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out_path = repeatability_dir / f"{stamp}_campaign_summary.json"
    write_json(out_path, campaign)
    print(json.dumps(campaign, indent=2, ensure_ascii=False, allow_nan=False))
    print("Saved campaign summary:", out_path)
    return out_path, campaign


def usage_examples():
    return f"""
PowerShell examples using the current Python interpreter:

  & "{INTERPRETER}" "{SCRIPT_PATH}" --case baseline_1
  & "{INTERPRETER}" "{SCRIPT_PATH}" --case right_50mm
  & "{INTERPRETER}" "{SCRIPT_PATH}" --summarize "{repeatability_dir}\\*_summary.json"
  & "{INTERPRETER}" "{SCRIPT_PATH}" --summarize "{repeatability_dir}\\*_summary.json" --compare baseline_1 right_50mm 50 --compare baseline_1 away_100mm 100

Known --case names:
  {", ".join(KNOWN_CASES)}
""".strip()


def parse_cli(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "OAK ArUco ID 23 pose viewer and measurement-capability sampling. "
            "Sampling records raw unfiltered pose fluctuation; it is not a precision certificate."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=usage_examples(),
    )
    parser.add_argument(
        "--case",
        choices=KNOWN_CASES,
        help="Live capture label written into sampling files.",
    )
    parser.add_argument(
        "--summarize",
        nargs="+",
        metavar="JSON",
        help="Offline: read sampling or summary JSON files (glob allowed).",
    )
    parser.add_argument(
        "--compare",
        action="append",
        nargs=3,
        metavar=("FROM", "TO", "NOMINAL_MM"),
        help="Offline: 3D length between two case mean XYZ values vs a setup nominal mm.",
    )
    return parser.parse_args(argv)


def handle_key(key, pose, packet, raw, display, captured_at, camera_matrix, distortion, device, factory_calibration_size_px, distortion_model, markers, status, sampling, case_name, context):
    if key == ord("t"):
        if sampling is not None:
            print("Sampling already running; extra t ignored.")
            return sampling, False
        if not case_name:
            print(
                "t ignored: start the script with --case <name>. "
                + usage_examples()
            )
            return sampling, False
        print(f"Starting {SAMPLE_DURATION_S:.0f}s sampling for {case_name}. Keep camera and marker still.")
        return start_sampling_session(case_name), False

    if key == ord("r"):
        save_camera_parameters(camera_matrix, distortion, device.getMxId())
        return sampling, False

    if key == ord("s") and packet is not None:
        output_dir.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        raw_path = output_dir / f"{stamp}_raw.png"
        marked_path = output_dir / f"{stamp}_marked.png"
        if not cv2.imwrite(str(raw_path), raw):
            raise IOError("Raw image save failed")
        if not cv2.imwrite(str(marked_path), display):
            raise IOError("Marked image save failed")
        record = {
            "received_at": captured_at,
            "frame_id": int(packet.getSequenceNum()),
            "dictionary": DICTIONARY_NAME,
            "image_size_px": [raw.shape[1], raw.shape[0]],
            "markers": markers,
            "pose": pose_save_fields(
                pose,
                camera_matrix,
                distortion,
                image_size_px=(raw.shape[1], raw.shape[0]),
                frame_id=packet.getSequenceNum(),
                received_at=captured_at,
                factory_calibration_size_px=factory_calibration_size_px,
                distortion_model_name=str(distortion_model),
            ),
            "raw_image": raw_path.name,
            "marked_image": marked_path.name,
            "opencv_version": cv2.__version__,
            "depthai_version": dai.__version__,
        }
        record_path = output_dir / f"{stamp}.json"
        record_path.write_text(
            json.dumps(record, indent=2, ensure_ascii=False, allow_nan=False),
            encoding="utf-8",
        )
        print("Saved:", record_path, "|", status, "|", pose["status"])
        return sampling, False

    if key == ord("q"):
        if sampling is not None:
            finish_sampling(sampling, context, completed=False, interrupted_by="q")
        return None, True

    return sampling, False


def run_camera(case_name):
    # 2. Acquire color frames only; do not load YOLO or depth processing.
    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.ColorCamera)
    camera.setBoardSocket(dai.CameraBoardSocket.RGB)
    camera.setResolution(
        dai.ColorCameraProperties.SensorResolution.THE_1080_P
    )
    camera.setPreviewSize(PREVIEW_WIDTH, PREVIEW_HEIGHT)
    camera.setPreviewKeepAspectRatio(True)
    camera.setInterleaved(False)
    camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camera.setFps(30)

    rgb_output = pipeline.create(dai.node.XLinkOut)
    rgb_output.setStreamName("rgb")
    camera.preview.link(rgb_output.input)

    # 3. Detect markers in each new frame; do not reuse previous detections.
    # 4. Estimate camera-relative marker pose only when exactly one ID 23 is detected.
    # 5. Press t to record raw poses for 10 seconds; no filtering or PnP changes.
    try:
        with dai.Device(pipeline) as device:
            calibration = device.readCalibration()
            distortion_model = calibration.getDistortionModel(
                dai.CameraBoardSocket.RGB
            )
            default_matrix, calib_width, calib_height = (
                calibration.getDefaultIntrinsics(dai.CameraBoardSocket.RGB)
            )
            factory_calibration_size_px = [int(calib_width), int(calib_height)]

            camera_matrix = np.array(
                calibration.getCameraIntrinsics(
                    dai.CameraBoardSocket.RGB, PREVIEW_WIDTH, PREVIEW_HEIGHT
                ),
                dtype=np.float64,
            )
            distortion = np.array(
                calibration.getDistortionCoefficients(
                    dai.CameraBoardSocket.RGB
                ),
                dtype=np.float64,
            )
            context = {
                "device_mxid": device.getMxId(),
                "camera_matrix": camera_matrix,
                "distortion": distortion,
                "distortion_model_name": str(distortion_model),
                "factory_calibration_size_px": factory_calibration_size_px,
            }

            print("RGB distortion model:", distortion_model)
            print("Factory RGB calibration size:", calib_width, "x", calib_height)
            print("Default RGB camera matrix:")
            print(np.array(default_matrix, dtype=np.float64))
            print("RGB camera matrix for 640x360:")
            print(camera_matrix)
            print("Distortion coefficients:")
            print(distortion)
            if distortion_model != dai.CameraModel.Perspective:
                print(
                    "Warning: PnP expects the OpenCV perspective model; "
                    "this device reports",
                    distortion_model,
                )
            if case_name:
                print("Live case:", case_name)
            else:
                print("No --case given. r/s/q work; t sampling requires --case.")
            print(usage_examples())
            print(
                "Press t to sample 10s; r camera parameters; "
                "s displayed frame; q quit. Focus the camera window."
            )
            print(
                "Pose is estimated only when exactly one ID "
                f"{TARGET_MARKER_ID} is visible. Marker length "
                f"{MARKER_LENGTH_MM} mm is approximate."
            )

            queue = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
            last_frame_time = time.monotonic()
            sampling = None

            while True:
                now = time.monotonic()
                if sampling is not None and sampling_timed_out(sampling, now):
                    finish_sampling(sampling, context, completed=True)
                    sampling = None

                packet = queue.tryGet()

                if packet is None:
                    if now - last_frame_time > 1.0:
                        blank = np.zeros(
                            (PREVIEW_HEIGHT, PREVIEW_WIDTH, 3), dtype=np.uint8
                        )
                        cv2.putText(
                            blank, "No fresh RGB frame",
                            (25, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 0, 255), 2
                        )
                        if sampling is not None:
                            cv2.putText(
                                blank,
                                sampling_overlay_text(sampling, case_name),
                                (15, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (0, 255, 255), 2
                            )
                        cv2.imshow("OAK ArUco", blank)

                    key = cv2.waitKey(10) & 0xFF
                    sampling, should_quit = handle_key(
                        key, empty_pose("no_target", "No fresh RGB frame"),
                        None, None, None, None, camera_matrix, distortion,
                        device, factory_calibration_size_px, distortion_model,
                        [], "No fresh RGB frame", sampling, case_name, context,
                    )
                    if should_quit:
                        break
                    continue

                last_frame_time = now
                captured_at = datetime.now().astimezone().isoformat(
                    timespec="milliseconds"
                )
                raw = packet.getCvFrame()
                gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray)

                display = raw.copy()
                markers = []

                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(display, corners, ids)

                    for marker_corners, marker_id in zip(
                        corners, ids.flatten()
                    ):
                        points = marker_corners.reshape(4, 2)
                        markers.append({
                            "id": int(marker_id),
                            "corners_px": points.tolist(),
                        })

                        for index, point in enumerate(points):
                            x, y = np.rint(point).astype(int)
                            cv2.circle(display, (x, y), 4, (0, 0, 255), -1)
                            cv2.putText(
                                display, str(index), (x + 5, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 255), 1
                            )

                pose = estimate_single_target_pose(
                    corners,
                    ids,
                    camera_matrix,
                    distortion,
                    image_size_px=(raw.shape[1], raw.shape[0]),
                )
                if (
                    distortion_model != dai.CameraModel.Perspective
                    and pose["status"] == "ok"
                ):
                    pose = empty_pose(
                        "invalid",
                        "Device distortion model is not OpenCV perspective",
                        extra={
                            "target_count": pose["target_count"],
                            "corners_px": pose["corners_px"],
                        },
                    )
                draw_pose_axes(display, pose, camera_matrix, distortion)

                if sampling is not None and not sampling_timed_out(sampling):
                    append_sample(
                        sampling,
                        pose,
                        packet.getSequenceNum(),
                        captured_at,
                        time.monotonic(),
                    )

                status = (
                    "IDs: " + ", ".join(str(m["id"]) for m in markers)
                    if markers else "No marker"
                )
                cv2.putText(
                    display, status, (15, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 180, 255), 2
                )
                cv2.putText(
                    display, pose_overlay_text(pose), (15, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0) if pose["status"] == "ok" else (0, 0, 255),
                    2
                )
                if sampling is not None:
                    cv2.putText(
                        display,
                        sampling_overlay_text(sampling, case_name),
                        (15, 85), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 255), 2
                    )
                cv2.imshow("OAK ArUco", display)
                key = cv2.waitKey(1) & 0xFF
                sampling, should_quit = handle_key(
                    key, pose, packet, raw, display, captured_at,
                    camera_matrix, distortion, device,
                    factory_calibration_size_px, distortion_model,
                    markers, status, sampling, case_name, context,
                )
                if should_quit:
                    break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    arguments = parse_cli()
    if arguments.summarize:
        run_offline_summarize(arguments.summarize, arguments.compare or [])
    elif arguments.compare:
        raise SystemExit(
            "--compare is only used with --summarize. " + usage_examples()
        )
    else:
        run_camera(arguments.case)
