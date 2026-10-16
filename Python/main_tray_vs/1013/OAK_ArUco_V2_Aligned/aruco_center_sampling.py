"""ArUco centre XYZ sampling using the existing OAK RGB/Depth pipeline.

S snapshots the latest 10 distinct synchronized RGB/Depth pairs (no waiting):
  first S = HOMEpos, then s1..s60; EVERY S advances even on failed samples.
  >=5 valid pairs -> one batch with XYZ stats, otherwise a failed attempt log.
F saves one current frame for diagnostics; Space freezes/unfreezes display; Q exits.
Depth stereo backprojection is the primary XYZ, not PnP. No robot control.
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path
import time
import shutil

from recent_frame_buffer import (
    RecentFrameBuffer, assess_snapshot, WINDOW_SIZE, MIN_VALID_FRAMES,
)

import cv2
import depthai as dai
import numpy as np

import mouse_depth_sampling as base


SESSION_ROOT = Path(__file__).resolve().parent / "aruco_center_sessions"
METHOD = "aruco_center_depth_backprojection"
VERSION = "V2-ALIGNED (V2 functions + fixed RAPID labels)"
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
    capture_at = None if pair is None else pair.get("capture_monotonic")
    return capture_at is not None and 0 <= now - capture_at <= MAX_FRAME_AGE_S


def save_sample(session, pair, context, label, frozen_source=False):
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
        "sampling_mode": "single_frame_debug", "frozen_source": bool(frozen_source),
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




class PointSchedule:
    """One RAPID label per S keypress, never per successful save.

    Normal mode: index 0 -> HOMEpos, 1..60 -> s1..s60.
    V2 custom --point-label/--point-prefix flags remain available for debugging,
    but should not be used for the HOMEpos,s1..s60 acquisition campaign.
    """

    def __init__(self, point_label=None, point_prefix="s", point_start=0):
        if point_start < 0:
            raise ValueError("--point-start must be >= 0")
        if point_label is None and point_prefix == "s" and point_start > 60:
            raise ValueError("RAPID schedule has indices 0..60 only")
        self.fixed = point_label
        self.prefix = point_prefix
        self.start = point_start
        self.attempt_count = 0

    def peek(self):
        if self.fixed is not None:
            return self.fixed
        index = self.start + self.attempt_count
        if self.prefix == "s":
            if index > 60:
                return None
            return "HOMEpos" if index == 0 else f"s{index}"
        return f"{self.prefix}{index}"

    def consume(self):
        """Call immediately on every S press BEFORE any frame or disk checks."""
        label = self.peek()
        if label is not None:
            self.attempt_count += 1
        return label


def next_number(session, prefix):
    """Assign a new per-session sequential name, without overwriting prior files."""
    numbers = []
    for path in session.glob(prefix + "_*"):
        suffix = path.name[len(prefix) + 1:]
        if suffix.isdigit():
            numbers.append(int(suffix))
    return max(numbers, default=0) + 1


def append_batch_index(session, summary, label, batch_name, saved_at):
    path = session / "batch_index.csv"
    fields = [
        "batch_id", "saved_at", "point_label", "status", "valid_frames",
        "buffered_frames", "mean_x_mm", "mean_y_mm", "mean_z_mm",
        "std_x_mm", "std_y_mm", "std_z_mm",
    ]
    means = summary["mean_xyz_mm"] or [None, None, None]
    stds = summary["std_xyz_mm"] or [None, None, None]
    row = dict(zip(fields, [batch_name, saved_at, label, summary["status"],
                            summary["valid_frames"], summary["buffered_frames"],
                            *means, *stds]))
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        if stream.tell() == 0:
            writer.writeheader()
        writer.writerow(row)


def save_batch(session, pairs, summary, records, usable, context, label, save_all_frames=False):
    """Persist an instantaneous 10-pair window. Failed attempts never enter HT index.

    Default: save all per-frame XYZ, timing, and validity metadata, and only one
    representative full-resolution RGB/Depth pair, to avoid 10x large depth files.
    --save-all-frames additionally saves all 10 RGB/Depth arrays.
    """
    prefix = "batch" if summary["status"] == "success" else "failed"
    batch_name = f"{prefix}_{next_number(session, prefix):04d}"
    final = session / batch_name
    temp = session / ("." + batch_name + ".writing")
    if final.exists() or temp.exists():
        raise FileExistsError(f"Sample path already exists: {final}")
    temp.mkdir(parents=True)
    saved_at = datetime.now().astimezone().isoformat(timespec="milliseconds")
    try:
        frame_files = []
        for i, pair in enumerate(pairs):
            outputs = {}
            if save_all_frames:
                rgb_path = temp / f"frame_{i:02d}_rgb.png"
                depth_path = temp / f"frame_{i:02d}_depth.npy"
                if not cv2.imwrite(str(rgb_path), pair["rgb"]):
                    raise IOError(rgb_path)
                np.save(depth_path, pair["depth"])
                outputs = {"rgb": rgb_path.name, "depth": depth_path.name}
            frame_files.append(outputs)

        if usable:
            # Preserve a visual reference for the latest *valid* frame by default.
            representative = usable[-1]
            rgb_path = temp / "representative_rgb.png"
            marked_path = temp / "representative_marked.png"
            depth_path = temp / "representative_depth.npy"
            if not cv2.imwrite(str(rgb_path), representative["rgb"]):
                raise IOError(rgb_path)
            if not cv2.imwrite(str(marked_path), representative["marked"]):
                raise IOError(marked_path)
            np.save(depth_path, representative["depth"])
            representative_files = {
                "rgb": rgb_path.name, "marked": marked_path.name, "depth": depth_path.name,
            }
        else:
            representative_files = None

        for record, outputs in zip(records, frame_files):
            record["files"] = outputs

        record = {
            "schema_version": 2,
            "session_id": session.name,
            "batch_id": batch_name,
            "saved_at": saved_at,
            "point_label": label,
            "status": summary["status"],
            "sampling_mode": "recent_10_distinct_pairs",
            "record_scope": "camera_only",
            "robot_correspondence": "unverified",
            "robot": {"tool": None, "reference_frame": None,
                      "xyz_mm": None, "quaternion": None},
            "summary": summary,
            "frames": records,
            "representative_files": representative_files,
            "save_all_frames": save_all_frames,
            "context": context,
            "geometry_note": GEOMETRY_NOTE,
            "note": "Only success batches with a verified robot point correspondence may enter HT fitting.",
        }
        base.write_json(temp / "sample.json", record)
        temp.rename(final)
    except Exception:
        shutil.rmtree(temp, ignore_errors=True)
        raise

    # Persist both SUCCESS and FAILED attempts under their original RAPID label.
    # Never use the row number of successful samples to match robot targets.
    append_batch_index(session, summary, label, batch_name, saved_at)
    return final / "sample.json"


def run(target_id, point_label=None, point_prefix="s", point_start=0, save_all_frames=False):
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250), cv2.aruco.DetectorParameters()
    )
    recent = RecentFrameBuffer(WINDOW_SIZE)
    latest = frozen = None
    notice = "Waiting for RGB/depth"
    schedule = PointSchedule(point_label, point_prefix, point_start)

    def active_label():
        return schedule.peek() or "COMPLETE (HOMEpos + s1..s60)"

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
                "session_id": session.name, "point_label": point_label,
                "automatic_point_prefix": point_prefix,
                "automatic_point_start": point_start,
                "app_version": VERSION,
                "point_schedule": "HOMEpos,s1,s2,...,s60" if point_label is None and point_prefix == "s" else "custom V2 override",
                "numbering_rule": "every S consumes one label, even if camera invalid or disk save fails; F does not",
                "context": context, "image_configuration": base.image_configuration(),
                "geometry_note": GEOMETRY_NOTE,
                "max_pair_delta_ms": base.MAX_PAIR_DELTA_MS,
                "max_live_frame_age_s": MAX_FRAME_AGE_S,
                "rolling_window_size": WINDOW_SIZE,
                "minimum_valid_frames": MIN_VALID_FRAMES,
                "capture_timeout_s": None,
                "save_all_frames": save_all_frames,
                "sampling_keys": {"s": "recent window batch", "f": "single frame debug",
                                  "space": "freeze/unfreeze", "q": "quit"},
            })
            queue = device.getOutputQueue(name="sync", maxSize=1, blocking=False)
            for name in ("OAK ArUco centre", "OAK centre depth"):
                cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
            print("=" * 65)
            print("STARTING:", VERSION)
            print("KEYS: S = ten-frame batch | F = single frame | SPACE = freeze | Q = quit")
            print("RAPID LABELS: HOMEpos, s1 ... s60; every S consumes one label")
            print("=" * 65)
            print("Session:", session)
            print(f"ID {target_id}: S snapshots last 10 new pairs (>=5 valid); F saves one frame; space freezes; Q quits")
            print("No sampling wait or timeout. S requires LIVE display. Camera-only XYZ; no robot offsets.")
            print("Next batch point:", active_label())
            print("Geometry verification remains pending. Robot data must be paired separately.")

            while True:
                packet = queue.tryGet()
                if packet is not None and frozen is None:
                    rgb_frame, depth_frame = packet["rgb"], packet["depth"]
                    rgb = rgb_frame.getCvFrame().copy()
                    depth = depth_frame.getFrame().copy()
                    timing = base.pair_timestamps(rgb_frame, depth_frame)
                    aligned, reason = base.geometry_status(rgb, depth)
                    age = (dai.Clock.now() - rgb_frame.getTimestamp()).total_seconds()
                    timestamp_ok = 0 <= age <= MAX_FRAME_AGE_S
                    sync_ok = timing["device_timestamp_delta_ms"] <= base.MAX_PAIR_DELTA_MS
                    capture_at = time.monotonic() - age if timestamp_ok else None
                    new_pair = {
                        "rgb": rgb, "depth": depth, "timing": timing,
                        "received_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                        "capture_monotonic": capture_at,
                        "received_monotonic": capture_at,
                        "measurement": None, "selection": None,
                        "reason": None, "marked": rgb.copy(),
                    }
                    if not aligned:
                        new_pair["reason"] = reason
                    elif not sync_ok:
                        new_pair["reason"] = "RGB/depth device timestamp mismatch"
                    elif not timestamp_ok:
                        new_pair["reason"] = "Stale or negative RGB timestamp age"
                    else:
                        new_pair = evaluate_pair(new_pair, detector, matrix, distortion, target_id)
                    if recent.add(new_pair):
                        latest = new_pair
                    else:
                        notice = "Duplicate RGB/depth capture ignored (not a new frame)"

                if frozen is None and latest is not None and not is_fresh(latest, time.monotonic()):
                    latest = None
                displayed = frozen if frozen is not None else latest
                now = time.monotonic()
                current_summary, _, _ = assess_snapshot(recent.snapshot(), now)
                if displayed is None:
                    shown = np.zeros((360, 640, 3), dtype=np.uint8)
                    depth_vis = shown.copy()
                    lines = ["No fresh paired frame - F disabled"]
                else:
                    shown = displayed["marked"].copy()
                    depth = displayed["depth"]
                    depth_vis = (base.colorize_depth(depth) if depth.ndim == 2
                                 else np.zeros((360, 640, 3), dtype=np.uint8))
                    measurement = displayed["measurement"]
                    lines = [f"ID {target_id}: {displayed['reason']}"]
                    if measurement is not None:
                        base.draw_probe(depth_vis, measurement)
                        xyz = measurement["camera_xyz_mm"]
                        lines.append(f"Depth XYZ mm: {xyz['x']:.2f}, {xyz['y']:.2f}, {xyz['z']:.2f}")
                        lines.append(f"Valid {measurement['valid_depth_count']}/25 | range {measurement['depth_range_mm']:.1f} mm")

                lines.extend([
                    f"{VERSION} | next {active_label()}",
                    f"Last {len(recent)}/{WINDOW_SIZE} pairs | {current_summary['valid_frames']}/{WINDOW_SIZE} valid",
                    "FROZEN: S disabled" if frozen is not None else "LIVE",
                    "S batch snapshot | F one frame | space freeze/live | Q quit",
                    notice,
                ])
                base.put_lines(shown, lines)
                cv2.imshow("OAK ArUco centre", shown)
                cv2.imshow("OAK centre depth", depth_vis)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    if frozen is not None:
                        frozen = latest = None
                        recent = RecentFrameBuffer(WINDOW_SIZE)
                        notice = "Resumed; buffer cleared to prevent frozen/old data reuse"
                    elif displayed is not None and is_fresh(displayed, time.monotonic()):
                        frozen = displayed
                        notice = "Frozen frame for F debug; S batch requires LIVE"
                elif key == ord("s"):
                    # Critical invariant: acquire the NEXT RAPID label as soon as S is
                    # pressed, not after validating frames or saving to disk.
                    label = schedule.consume()
                    if label is None:
                        notice = "Schedule complete: HOMEpos and s1..s60 were attempted; S disabled"
                    else:
                        # Keep frozen display for F debugging, but a frozen S is
                        # recorded as FAILED and consumes its label (no silent shift).
                        pairs = [] if frozen is not None else recent.snapshot()
                        summary, records, good = assess_snapshot(pairs, time.monotonic())
                        if frozen is not None:
                            summary["status"] = "failed"
                            summary["failure_reason"] = "S pressed while display frozen; no live batch"
                        try:
                            path = save_batch(session, pairs, summary, records, good,
                                              context, label, save_all_frames=save_all_frames)
                            notice = (f"Sampling complete: {label}: "
                                      f"{summary['valid_frames']}/{WINDOW_SIZE} valid; "
                                      f"{summary['status'].upper()}; next {active_label()}")
                            print(notice, "->", path)
                        except Exception as error:
                            # Still advance. Never reuse s11 after a disk error.
                            notice = (f"{label}: SAVE ERROR ({error}); "
                                      f"next {active_label()} - verify disk / log")
                            print(notice)
                        finally:
                            # A new RAPID point must never reuse an old point's RGB/depth.
                            recent.clear()
                            latest = None
                elif key == ord("f"):
                    if displayed is None or (frozen is None and not is_fresh(displayed, time.monotonic())):
                        notice = "F not saved: no fresh pair"
                    elif displayed["measurement"] is None:
                        notice = "F not saved: " + str(displayed["reason"])
                    else:
                        try:
                            path = save_sample(session, displayed, context, active_label(),
                                               frozen_source=(frozen is not None))
                            notice = "Single frame saved " + path.parent.name
                            print(notice, path)
                        except Exception as error:
                            notice = "Single frame save failed: " + str(error)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-id", type=int, choices=range(250), default=23, metavar="0..249")
    parser.add_argument("--point-label", default=None,
                        help="V2 custom fixed label for isolated tests; omit for HOMEpos,s1..s60")
    parser.add_argument("--point-prefix", default="s", help="V2 custom point prefix; default s")
    parser.add_argument("--point-start", type=int, default=0,
                        help="Resume index: 0=HOMEpos (default), 1=s1, ..., 60=s60")
    parser.add_argument("--save-all-frames", action="store_true",
                        help="Additionally save RGB/Depth arrays for every buffered frame; requires extra disk space")
    args = parser.parse_args()
    run(args.target_id, args.point_label, args.point_prefix, args.point_start, args.save_all_frames)
