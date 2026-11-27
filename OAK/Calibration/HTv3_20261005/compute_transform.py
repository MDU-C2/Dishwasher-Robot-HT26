"""Offline camera -> tool0/wobj0 fit for session 20261005_124152_507239.

Same model as HTv2: rigid Kabsch, no scale. Does not read or write the session folder
except to load samples. No camera, network, or robot operations.
"""
from __future__ import annotations

import csv
import json
import os
import re
from datetime import datetime, timezone, timedelta
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent
SESSION = Path(os.environ.get("OAK_CALIB_SESSION", str(OUT)))

ROBTARGET_RE = re.compile(
    r"CONST\s+robtarget\s+(\w+)\s*:=\s*\[\[([^]]+)\],\[([^]]+)\]",
    re.IGNORECASE,
)
MOVE_RE = re.compile(r"Move[JL]\s+(\w+),", re.IGNORECASE)


def estimate_rigid_transform(camera_points, robot_points):
    cam = np.asarray(camera_points, dtype=np.float64)
    rob = np.asarray(robot_points, dtype=np.float64)
    if cam.shape != rob.shape or cam.ndim != 2 or cam.shape[1] != 3 or cam.shape[0] < 3:
        raise ValueError(f"need matching Nx3 points, got camera {cam.shape} robot {rob.shape}")
    camera_centroid = cam.mean(axis=0)
    robot_centroid = rob.mean(axis=0)
    cross_covariance = (cam - camera_centroid).T @ (rob - robot_centroid)
    U, _s, Vt = np.linalg.svd(cross_covariance)
    V = Vt.T
    det_correction = np.sign(np.linalg.det(V @ U.T))
    if det_correction == 0.0:
        det_correction = 1.0
    rotation = V @ np.diag([1.0, 1.0, det_correction]) @ U.T
    translation = robot_centroid - rotation @ camera_centroid
    return rotation, translation


def residuals(camera_points, robot_points, rotation, translation):
    cam = np.asarray(camera_points, dtype=np.float64)
    rob = np.asarray(robot_points, dtype=np.float64)
    predicted = (rotation @ cam.T).T + np.asarray(translation, dtype=np.float64).reshape(1, 3)
    return predicted, np.linalg.norm(predicted - rob, axis=1)


def parse_targets(mod_text):
    targets = {}
    for name, trans, quat in ROBTARGET_RE.findall(mod_text):
        xyz = [float(part) for part in trans.split(",")]
        q = [float(part) for part in quat.split(",")]
        if len(xyz) != 3 or len(q) != 4:
            raise ValueError(f"unexpected robtarget {name}")
        targets[name] = {"xyz": xyz, "quaternion_wxyz": q}
    moves = MOVE_RE.findall(mod_text)
    if not moves or moves[0] != "HOMEpos":
        raise ValueError(f"expected HOMEpos first, got {moves[:3]}")
    missing = [name for name in moves if name not in targets]
    if missing:
        raise ValueError(f"move names without declarations: {missing}")
    return targets, moves


def load_index(path):
    rows = {}
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            rows[row["sample_id"]] = row
    return rows


def main():
    targets, moves = parse_targets((SESSION / "Calibrate_new.mod").read_text(encoding="utf-8"))
    index = load_index(SESSION / "index.csv")
    points = []
    for number, robot_name in enumerate(moves, start=1):
        sample_id = f"sample_{number:04d}"
        expected = "HOMEpos" if number == 1 else f"s{number - 1}"
        if robot_name != expected:
            raise ValueError(f"{sample_id} move is {robot_name}, expected {expected}")
        entry = {
            "sample_number": number,
            "sample_id": sample_id,
            "robot_target": robot_name,
            "robot_xyz_mm": targets[robot_name]["xyz"],
            "robot_quaternion_wxyz": targets[robot_name]["quaternion_wxyz"],
        }
        row = index.get(sample_id)
        sample_json = SESSION / sample_id / "sample.json"
        if row is None or not sample_json.is_file():
            entry["prefilter_reason"] = "no OAK sample"
            points.append(entry)
            continue
        measurement = json.loads(sample_json.read_text(encoding="utf-8"))["measurement"]
        xyz = measurement["camera_xyz_mm"]
        entry.update(
            {
                "camera_xyz_mm": [xyz["x"], xyz["y"], xyz["z"]],
                "valid_depth_count": int(measurement["valid_depth_count"]),
                "depth_range_mm": float(measurement["depth_range_mm"]),
                "depth_std_mm": float(measurement["depth_std_mm"]),
            }
        )
        if entry["valid_depth_count"] < 15:
            entry["prefilter_reason"] = f"valid_depth_count {entry['valid_depth_count']} < 15"
        elif entry["depth_range_mm"] > 20.0:
            entry["prefilter_reason"] = f"depth_range_mm {entry['depth_range_mm']} > 20"
        points.append(entry)

    candidates = [p for p in points if "prefilter_reason" not in p]
    cam = np.array([p["camera_xyz_mm"] for p in candidates], dtype=np.float64)
    rob = np.array([p["robot_xyz_mm"] for p in candidates], dtype=np.float64)
    rotation, translation = estimate_rigid_transform(cam, rob)
    _predicted, errors = residuals(cam, rob, rotation, translation)
    for point, error in zip(candidates, errors):
        point["first_fit_residual_mm"] = float(error)

    other_median = float(np.median(np.delete(errors, int(np.argmax(errors)))))
    outlier_ids = []
    for point, error in zip(candidates, errors):
        if error > 100.0 and error > 4.0 * other_median:
            point["prefilter_reason"] = (
                f"residual outlier {float(error):.3f} mm after first fit "
                f"(median of the other prefiltered points {other_median:.3f} mm)"
            )
            outlier_ids.append(point["sample_id"])

    used = [p for p in candidates if "prefilter_reason" not in p]
    cam = np.array([p["camera_xyz_mm"] for p in used], dtype=np.float64)
    rob = np.array([p["robot_xyz_mm"] for p in used], dtype=np.float64)
    rotation, translation = estimate_rigid_transform(cam, rob)
    predicted, errors = residuals(cam, rob, rotation, translation)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8):
        raise RuntimeError("R is not orthogonal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8):
        raise RuntimeError(f"det(R)={np.linalg.det(rotation)}")

    for point, pred, error in zip(used, predicted, errors):
        point["predicted_robot_xyz_mm"] = pred.tolist()
        point["residual_mm"] = float(error)

    homogeneous = np.eye(4)
    homogeneous[:3, :3] = rotation
    homogeneous[:3, 3] = translation
    created = datetime.now(timezone(timedelta(hours=2))).isoformat(timespec="seconds")
    fit_error = {
        "rms": float(np.sqrt(np.mean(errors ** 2))),
        "mean": float(np.mean(errors)),
        "median": float(np.median(errors)),
        "max": float(np.max(errors)),
        "min": float(np.min(errors)),
    }
    audit = {
        "created_at": created,
        "source_session": str(SESSION),
        "source_unmodified": True,
        "pairing": "sample_0001 = HOMEpos; sample_000N = s(N-1); sample_0061 = s60",
        "robot_frame": "ABB robtarget translation in mm, tool0, wobj0. Quaternion stored but not used in the fit.",
        "camera_frame": "OAK RGB camera frame, mm. X right, Y down, Z forward. Point is the ID 23 center from depth backprojection, not PnP and not TCP.",
        "transform_direction": "p_wobj0_mm = R @ p_camera_mm + t",
        "model": "Rigid Kabsch, no scale. Same physical point is assumed. Marker-to-tool0 offset was not measured separately.",
        "marker_placement": "ID 23 is held between the gripper fingers. The saved camera point is still the marker center, and the robot point is still tool0.",
        "rejection_rules": {
            "no_oak_sample": "empty sample folders or rows absent from index.csv",
            "valid_depth_count_lt_15": "5x5 neighborhood has too few valid depth pixels",
            "depth_range_mm_gt_20": "neighborhood mixes widely different depths",
            "residual_outlier": "after the first fit, drop a point only when its residual exceeds 100 mm and four times the median residual of the other prefiltered points",
        },
        "counts": {
            "robot_targets": len(moves),
            "oak_samples": sum(1 for p in points if "camera_xyz_mm" in p),
            "prefiltered_before_outlier": len(candidates),
            "residual_outliers": outlier_ids,
            "used_in_fit": len(used),
            "rejected": len(points) - len(used),
        },
        "fit_error_mm": fit_error,
        "rotation_matrix": rotation.tolist(),
        "translation_mm": translation.tolist(),
        "homogeneous_4x4": homogeneous.tolist(),
        "determinant": float(np.linalg.det(rotation)),
        "limitations": [
            "This is a point-to-point rigid fit, not a finished hand-eye calibration.",
            "Camera point is the ArUco center from stereo depth. Robot point is tool0 in wobj0. Their fixed offset is not separately known.",
            "The marker sits between the fingers in this session, so it is nearer the grasp than an offset tag, but that does not make the marker center identical to tool0.",
            "RMS describes this filtered set only. It is not a certified robot accuracy.",
            "Depth alignment and the marker size were not re-verified in this fit.",
        ],
        "points": points,
    }
    spec = {
        "direction": "camera_to_RAPID_s_target_frame",
        "units": "mm",
        "formula": "p_robot = R @ p_camera + t; homogeneous column vectors",
        "status": "candidate_fitted_not_independently_validated",
        "pairing": audit["pairing"] + ". HOMEpos and other unreliable samples were not used in the fit.",
        "tool_and_workobject": "Calibrate_new.mod moves with tool0 and wobj0",
        "selected_s": [p["robot_target"] for p in used],
        "selected_samples": [p["sample_id"] for p in used],
        "R": rotation.tolist(),
        "t_mm": translation.tolist(),
        "T_4x4": homogeneous.tolist(),
        "fit_rmse_mm": fit_error["rms"],
        "fit_mean_mm": fit_error["mean"],
        "fit_max_mm": fit_error["max"],
        "per_point": [
            {
                "s": p["robot_target"],
                "sample_id": p["sample_id"],
                "predicted_robot_xyz_mm": p["predicted_robot_xyz_mm"],
                "residual_mm": p["residual_mm"],
            }
            for p in used
        ],
    }
    paired = [
        {
            "sample_id": p["sample_id"],
            "robot_target": p["robot_target"],
            "camera_xyz_mm": p["camera_xyz_mm"],
            "robot_xyz_mm": p["robot_xyz_mm"],
        }
        for p in used
    ]
    (OUT / "transform.json").write_text(json.dumps(audit, indent=2), encoding="utf-8")
    (OUT / "transform_camera_to_robot.json").write_text(json.dumps(spec, indent=2), encoding="utf-8")
    (OUT / "paired_points.json").write_text(json.dumps(paired, indent=2), encoding="utf-8")
    np.savetxt(OUT / "T_camera_to_robot.txt", homogeneous, fmt="%.12f")
    with (OUT / "paired_points.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "sample_id",
                "robot_target",
                "camera_x_mm",
                "camera_y_mm",
                "camera_z_mm",
                "robot_x_mm",
                "robot_y_mm",
                "robot_z_mm",
            ]
        )
        for p in used:
            writer.writerow(
                [
                    p["sample_id"],
                    p["robot_target"],
                    f"{p['camera_xyz_mm'][0]:.8f}",
                    f"{p['camera_xyz_mm'][1]:.8f}",
                    f"{p['camera_xyz_mm'][2]:.8f}",
                    f"{p['robot_xyz_mm'][0]:.8f}",
                    f"{p['robot_xyz_mm'][1]:.8f}",
                    f"{p['robot_xyz_mm'][2]:.8f}",
                ]
            )
    print(json.dumps({
        "used": len(used),
        "rejected": len(points) - len(used),
        "outliers": outlier_ids,
        "fit_error_mm": fit_error,
        "R": rotation.tolist(),
        "t_mm": translation.tolist(),
        "det": float(np.linalg.det(rotation)),
    }, indent=2))


if __name__ == "__main__":
    main()
