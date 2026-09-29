"""Automatic ID 23 centre selection with the existing depth sampling pipeline.

S saves the displayed pair; space optionally freezes/resumes; Q quits.
The primary XYZ uses stereo depth, NOT ArUco PnP. No robot control.
"""

import argparse
from datetime import datetime
from pathlib import Path
import time

import cv2
import depthai as dai
import numpy as np

import mouse_depth_sampling as base


SESSION_ROOT = Path(__file__).resolve().parent / "aruco_center_sessions"
METHOD = "aruco_center_depth_backprojection"
MAX_FRAME_AGE_S = 0.5
GEOMETRY_NOTE = (
    "Inherited from mouse_depth_sampling: depth aligned to RGB, indexed at the "
    "distorted RGB pixel, with the RGB ray undistorted once. Matching distorted "
    "RGB/depth pixel geometry still requires device-specific verification. "
    "This recording does not certify calibration accuracy."
)


def marker_center(points, matrix, distortion):
    """Intersect diagonals in undistorted coordinates, then redistort for ROI lookup."""
    points = np.asarray(points, dtype=np.float64).reshape(4, 2)
    if not np.isfinite(points).all():
        raise ValueError("Non-finite corners")
    rays = cv2.undistortPoints(points.reshape(-1, 1, 2), matrix, distortion).reshape(4, 2)
    homogeneous = np.column_stack((rays, np.ones(4)))
    crossing = np.cross(
        np.cross(homogeneous[0], homogeneous[2]),
        np.cross(homogeneous[1], homogeneous[3]),
    )
    if not np.isfinite(crossing).all() or abs(crossing[2]) < 1e-12:
        raise ValueError("Degenerate marker diagonals")
    ray = crossing[:2] / crossing[2]
    pixel, _ = cv2.projectPoints(
        np.array([[ray[0], ray[1], 1.0]]), np.zeros(3), np.zeros(3), matrix, distortion
    )
    pixel = pixel.reshape(2)
    if not np.isfinite(pixel).all():
        raise ValueError("Non-finite centre")
    return pixel, ray


def select_measurement(corners, ids, depth, matrix, distortion, target_id):
    """Reject absent/ambiguous targets; never reuse an earlier measurement."""
    selection = {"dictionary": "DICT_6X6_250", "target_id": target_id,
                 "detected_ids": [] if ids is None else np.asarray(ids).flatten().tolist(),
                 "center_method": "diagonal intersection in undistorted normalized coordinates"}
    matches = [i for i, value in enumerate(selection["detected_ids"]) if value == target_id]
    selection["target_count"] = len(matches)
    if len(matches) != 1:
        reason = "No target marker" if not matches else "Target ID is not unique"
        return None, selection, reason
    points = np.asarray(corners[matches[0]], dtype=np.float64).reshape(4, 2)
    try:
        center, ray = marker_center(points, matrix, distortion)
    except (ValueError, cv2.error) as error:
        return None, selection, str(error)
    selection.update({"corners_px": points.tolist(), "center_px": center.tolist()})
    if depth is None or not (0 <= center[0] < depth.shape[1] and 0 <= center[1] < depth.shape[0]):
        return None, selection, "Centre outside depth image"
    u, v = np.rint(center).astype(int)
    measurement, reason = base.measure_click(depth, int(u), int(v), matrix, distortion)
    if measurement is None:
        return None, selection, reason
    # Integer centre selects the 5x5 ROI; subpixel centre defines the optical ray.
    z = measurement["depth_median_mm"]
    xyz = np.array([ray[0] * z, ray[1] * z, z])
    if not np.isfinite(xyz).all():
        return None, selection, "Non-finite XYZ"
    measurement.update({
        "camera_xyz_mm": dict(zip(("x", "y", "z"), xyz.tolist())),
        "method": METHOD,
        "target_id": target_id,
        "center_subpixel_uv": center.tolist(),
        "normalized_center_ray": [float(ray[0]), float(ray[1]), 1.0],
        "backprojection_note": "Subpixel marker-centre ray times 5x5 median depth Z; not PnP XYZ",
        "geometry_verification": "pending",
        "geometry_note": GEOMETRY_NOTE,
    })
    return measurement, selection, "ok"


def evaluate_pair(pair, detector, matrix, distortion, target_id):
    gray = cv2.cvtColor(pair["rgb"], cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    measurement, selection, reason = select_measurement(
        corners, ids, pair["depth"], matrix, distortion, target_id
    )
    pair.update({"measurement": measurement, "selection": selection, "reason": reason})
    marked = pair["rgb"].copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(marked, corners, ids)
    if "center_px" in selection:
        center = tuple(np.rint(selection["center_px"]).astype(int))
        cv2.drawMarker(marked, center, (0, 255, 255), cv2.MARKER_CROSS, 16, 1)
    if measurement is not None:
        base.draw_probe(marked, measurement)
    pair["marked"] = marked
    return pair


def is_fresh(pair, now):
    return pair is not None and 0 <= now - pair["received_monotonic"] <= MAX_FRAME_AGE_S


def save_sample(session, pair, context, label):
    measurement = pair["measurement"]
    if measurement is None:
        raise ValueError("Cannot save invalid measurement: " + pair["reason"])
    name = f"sample_{base.next_sample_id(session):04d}"
    folder = session / name
    folder.mkdir(exist_ok=False)
    marked = pair["marked"].copy()
    xyz = measurement["camera_xyz_mm"]
    base.put_lines(marked, [name, f"Depth XYZ mm: {xyz['x']:.2f}, {xyz['y']:.2f}, {xyz['z']:.2f}"])
    for filename, frame in (("rgb.png", pair["rgb"]), ("marked.png", marked)):
        if not cv2.imwrite(str(folder / filename), frame):
            raise IOError("Image write failed: " + filename)
    np.save(folder / "depth.npy", pair["depth"])
    saved_at = datetime.now().astimezone().isoformat(timespec="milliseconds")
    record = {
        "schema_version": 1, "session_id": session.name, "sample_id": name,
        "saved_at": saved_at, "received_at": pair["received_at"],
        "point_label": label, "status": "valid_depth_measurement",
        "record_scope": "camera_only", "robot_correspondence": "unverified",
        "robot": {"tool": None, "reference_frame": None, "xyz_mm": None, "quaternion": None},
        "measurement": measurement, "aruco_selection": pair["selection"],
        "timing": pair["timing"], "context": context,
        "image_configuration": base.image_configuration(),
        "files": {"rgb": "rgb.png", "marked": "marked.png", "depth": "depth.npy"},
        "geometry_note": GEOMETRY_NOTE,
    }
    base.write_json(folder / "sample.json", record)
    base.append_index(session / "index.csv", {
        "sample_id": name, "saved_at": saved_at, "point_label": label,
        "pixel_u": measurement["pixel_u"], "pixel_v": measurement["pixel_v"],
        "x_mm": xyz["x"], "y_mm": xyz["y"], "z_mm": xyz["z"],
        "valid_depth_count": measurement["valid_depth_count"],
        "depth_range_mm": measurement["depth_range_mm"],
        "device_timestamp_delta_ms": pair["timing"]["device_timestamp_delta_ms"],
    })
    return folder / "sample.json"


def run(target_id, point_label):
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250), cv2.aruco.DetectorParameters()
    )
    latest = frozen = None
    notice = "Waiting for RGB/depth"
    try:
        with dai.Device(base.build_pipeline()) as device:
            calibration = device.readCalibration()
            socket = dai.CameraBoardSocket.RGB
            matrix = np.asarray(calibration.getCameraIntrinsics(socket, 640, 360), dtype=np.float64)
            distortion = np.asarray(calibration.getDistortionCoefficients(socket), dtype=np.float64)
            _, width, height = calibration.getDefaultIntrinsics(socket)
            if distortion.size != 14 or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
                raise ValueError("Unexpected RGB calibration")
            context = {
                "device_mxid": device.getMxId(), "camera_matrix": matrix.tolist(),
                "distortion_coefficients": distortion.tolist(), "distortion_order": base.DISTORTION_NAMES,
                "distortion_model_name": str(calibration.getDistortionModel(socket)),
                "factory_calibration_size_px": [width, height],
                "opencv_version": cv2.__version__, "depthai_version": dai.__version__,
                "method": METHOD, "target_id": target_id,
            }
            session = SESSION_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            session.mkdir(parents=True)
            base.write_json(session / "session.json", {
                "session_id": session.name, "point_label": point_label, "context": context,
                "image_configuration": base.image_configuration(), "geometry_note": GEOMETRY_NOTE,
                "max_pair_delta_ms": base.MAX_PAIR_DELTA_MS, "max_live_frame_age_s": MAX_FRAME_AGE_S,
            })
            queue = device.getOutputQueue(name="sync", maxSize=1, blocking=False)
            for name in ("OAK ArUco centre", "OAK centre depth"):
                cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
            print("Session:", session)
            print(f"ID {target_id}: s saves displayed centre DEPTH XYZ; space freezes/resumes; q quits")
            print("Geometry verification remains pending. Robot data must be paired separately.")
            while True:
                packet = queue.tryGet()
                if packet is not None and frozen is None:
                    rgb_frame, depth_frame = packet["rgb"], packet["depth"]
                    rgb, depth = rgb_frame.getCvFrame().copy(), depth_frame.getFrame().copy()
                    timing = base.pair_timestamps(rgb_frame, depth_frame)
                    valid, reason = base.geometry_status(rgb, depth)
                    age = (dai.Clock.now() - rgb_frame.getTimestamp()).total_seconds()
                    if not valid or timing["device_timestamp_delta_ms"] > base.MAX_PAIR_DELTA_MS or not 0 <= age <= MAX_FRAME_AGE_S:
                        latest = None
                        notice = reason if not valid else "Pair rejected: timestamp mismatch or stale RGB"
                    else:
                        latest = evaluate_pair({
                            "rgb": rgb, "depth": depth, "timing": timing,
                            "received_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                            "received_monotonic": time.monotonic() - age,
                        }, detector, matrix, distortion, target_id)
                if frozen is None and not is_fresh(latest, time.monotonic()):
                    latest = None
                displayed = frozen if frozen is not None else latest
                if displayed is None:
                    shown = np.zeros((360, 640, 3), dtype=np.uint8)
                    depth_vis = shown.copy()
                    lines = ["No fresh paired frame - save disabled"]
                else:
                    shown = displayed["marked"].copy()
                    depth_vis = base.colorize_depth(displayed["depth"])
                    measurement = displayed["measurement"]
                    lines = [f"ID {target_id}: {displayed['reason']}"]
                    if measurement is not None:
                        base.draw_probe(depth_vis, measurement)
                        xyz = measurement["camera_xyz_mm"]
                        lines.append(f"Depth XYZ mm: {xyz['x']:.2f}, {xyz['y']:.2f}, {xyz['z']:.2f}")
                        lines.append(f"Valid {measurement['valid_depth_count']}/25 | range {measurement['depth_range_mm']:.1f} mm")
                lines += ["FROZEN" if frozen is not None else "LIVE", "s save | space freeze/live | q quit", notice]
                base.put_lines(shown, lines)
                cv2.imshow("OAK ArUco centre", shown)
                cv2.imshow("OAK centre depth", depth_vis)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    if frozen is not None:
                        frozen = latest = None
                        notice = "Resumed: waiting for a new pair"
                    elif is_fresh(displayed, time.monotonic()):
                        frozen = displayed
                        notice = "Frozen pair; keep robot stationary until paired record is complete"
                if key == ord("s"):
                    if displayed is None or (frozen is None and not is_fresh(displayed, time.monotonic())):
                        notice = "Not saved: no fresh pair"
                    elif displayed["measurement"] is None:
                        notice = "Not saved: " + displayed["reason"]
                    else:
                        try:
                            path = save_sample(session, displayed, context, point_label)
                            notice = "Saved " + path.parent.name
                            print(notice, path)
                        except Exception as error:
                            notice = "Save failed: " + str(error)
                    print(notice)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-id", type=int, choices=range(250), default=23, metavar="0..249")
    parser.add_argument("--point-label", default=None)
    args = parser.parse_args()
    run(args.target_id, args.point_label or f"aruco_{args.target_id}_center")
