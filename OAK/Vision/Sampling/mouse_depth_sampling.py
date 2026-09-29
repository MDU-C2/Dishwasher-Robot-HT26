"""One left click measures and saves the displayed RGB/depth pair.

ArUco ID, corners, and center are drawn so the click can be aimed.
ArUco pose and corner depth are auxiliary records, not the primary click sample.
The saved XYZ is always the mouse depth measurement.
"""

from pathlib import Path
from datetime import datetime, timedelta
import argparse
import csv
import json

import cv2
import depthai as dai
import numpy as np

import aruco_rgb as aruco


PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 360
PROBE_HALF_SIZE = 2
PROBE_MIN_VALID = 5
MAX_PAIR_DELTA_MS = 33.0
MEASUREMENT_METHOD = "mouse_depth_backprojection"
DISTORTION_NAMES = [
    "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6",
    "s1", "s2", "s3", "s4", "tau_x", "tau_y",
]

SCRIPT_DIR = Path(__file__).resolve().parent
SESSION_ROOT = SCRIPT_DIR / "sessions"


def timedelta_to_ms(value):
    return float(value.total_seconds() * 1000.0)


def as_float_list(values):
    return [float(v) for v in np.asarray(values, dtype=np.float64).reshape(-1)]


def image_configuration():
    return {
        "sensor_resolution": "1920x1080",
        "preview_size_px": [PREVIEW_WIDTH, PREVIEW_HEIGHT],
        "preview_keep_aspect_ratio": True,
        "mono_resolution": "400p",
        "depth_align_socket": "RGB",
        "depth_output_size_px": [PREVIEW_WIDTH, PREVIEW_HEIGHT],
        "depth_output_keep_aspect_ratio": True,
        "host_rotation_degrees": 0,
        "color_order": "BGR",
        "fps_setting": 30,
        "rgb_host_undistort": False,
        "not_the_640x640_stretched_preview": True,
    }


def pair_timestamps(rgb_frame, depth_frame):
    """Pair by device Sync plus host timestamp delta. Sequence numbers are recorded only."""
    rgb_host_ms = timedelta_to_ms(rgb_frame.getTimestamp())
    depth_host_ms = timedelta_to_ms(depth_frame.getTimestamp())
    rgb_device_ms = timedelta_to_ms(rgb_frame.getTimestampDevice())
    depth_device_ms = timedelta_to_ms(depth_frame.getTimestampDevice())
    return {
        "rgb_sequence": int(rgb_frame.getSequenceNum()),
        "depth_sequence": int(depth_frame.getSequenceNum()),
        "rgb_host_timestamp_ms": rgb_host_ms,
        "depth_host_timestamp_ms": depth_host_ms,
        "host_timestamp_delta_ms": abs(rgb_host_ms - depth_host_ms),
        "rgb_device_timestamp_ms": rgb_device_ms,
        "depth_device_timestamp_ms": depth_device_ms,
        "device_timestamp_delta_ms": abs(rgb_device_ms - depth_device_ms),
        "max_pair_delta_ms": MAX_PAIR_DELTA_MS,
        "sync_method": (
            "dai.node.Sync groups rgb and depth by timestamp; "
            "host then rejects device-timestamp delta above max_pair_delta_ms. "
            "Equal sequence numbers are not treated as synchronization."
        ),
    }


def geometry_status(rgb, depth):
    """Alignment is setDepthAlign(RGB) at the preview size, not 'same shape means aligned'."""
    if rgb is None or depth is None:
        return False, "No RGB/depth pair yet"
    if rgb.ndim != 3 or depth.ndim != 2:
        return False, "Unexpected RGB or depth array shape"
    rgb_h, rgb_w = rgb.shape[:2]
    depth_h, depth_w = depth.shape[:2]
    if (rgb_w, rgb_h) != (PREVIEW_WIDTH, PREVIEW_HEIGHT):
        return False, f"RGB is {rgb_w}x{rgb_h}, expected {PREVIEW_WIDTH}x{PREVIEW_HEIGHT}"
    if (depth_w, depth_h) != (rgb_w, rgb_h):
        return (
            False,
            f"Aligned depth {depth_w}x{depth_h} does not match RGB {rgb_w}x{rgb_h}",
        )
    return True, "Depth output matches the RGB preview that it was aligned to"


def measure_click(depth, pixel_u, pixel_v, camera_matrix, distortion):
    """5x5 valid-depth median, then one undistort of the click, then Z times the ray.

    The RGB preview is the distorted ColorCamera image. Depth is warped onto that
    same view, so the click stays on the distorted pixel. undistortPoints is applied
    once to that pixel. The displayed image is not undistorted, so this is not a
    second correction of an already-rectified picture.
    """
    if depth is None:
        return None, "No frozen depth"
    height, width = depth.shape[:2]
    if not (0 <= pixel_u < width and 0 <= pixel_v < height):
        return None, "Click is outside the frozen RGB frame"

    x0 = max(0, pixel_u - PROBE_HALF_SIZE)
    x1 = min(width, pixel_u + PROBE_HALF_SIZE + 1)
    y0 = max(0, pixel_v - PROBE_HALF_SIZE)
    y1 = min(height, pixel_v + PROBE_HALF_SIZE + 1)
    roi = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64)
    valid = roi[np.isfinite(roi) & (roi > 0)]
    valid_count = int(valid.size)
    if valid_count < PROBE_MIN_VALID:
        return None, (
            f"Need at least {PROBE_MIN_VALID} valid depth pixels, got {valid_count}"
        )

    z_mm = float(np.median(valid))
    if not np.isfinite(z_mm) or z_mm <= 0:
        return None, "Median depth is not a positive finite value"

    pixel = np.array([[[float(pixel_u), float(pixel_v)]]], dtype=np.float64)
    try:
        normalized = cv2.undistortPoints(pixel, camera_matrix, distortion)
    except cv2.error as error:
        return None, f"undistortPoints failed: {error}"
    x_n, y_n = np.asarray(normalized, dtype=np.float64).reshape(2)
    xyz = np.array([x_n * z_mm, y_n * z_mm, z_mm], dtype=np.float64)
    if not np.isfinite(xyz).all():
        return None, "Back-projected XYZ is not finite"

    std_mm = float(valid.std(ddof=1)) if valid_count >= 2 else None
    measurement = {
        "pixel_u": int(pixel_u),
        "pixel_v": int(pixel_v),
        "depth_pixel_u": int(pixel_u),
        "depth_pixel_v": int(pixel_v),
        "depth_pixel_mapping": (
            "Same u,v index in the aligned depth array. "
            "Used only after the depth frame size matches the RGB preview."
        ),
        "camera_xyz_mm": {
            "x": float(xyz[0]),
            "y": float(xyz[1]),
            "z": float(xyz[2]),
        },
        "units": "mm",
        "camera_frame": "X right, Y down, Z forward; not zeroed to the first frame",
        "roi_px": {
            "x0": int(x0),
            "y0": int(y0),
            "x1": int(x1),
            "y1": int(y1),
            "half_size": PROBE_HALF_SIZE,
        },
        "valid_depth_count": valid_count,
        "roi_pixel_count": int(roi.size),
        "depth_median_mm": z_mm,
        "depth_std_mm": std_mm,
        "depth_min_mm": float(valid.min()),
        "depth_max_mm": float(valid.max()),
        "depth_range_mm": float(valid.max() - valid.min()),
        "depth_spread_note": (
            "std is the sample standard deviation (ddof=1) of valid depths in the "
            "5x5 neighborhood. The saved Z is their median, so an edge can mix "
            "object and background. A small spread does not prove the center surface."
        ),
        "method": MEASUREMENT_METHOD,
        "backprojection_note": (
            "cv2.undistortPoints once on the distorted RGB click, then multiply "
            "the normalized ray by the median depth Z. The RGB image itself is "
            "not undistorted."
        ),
    }
    return measurement, "ok"


def colorize_depth(depth):
    display = np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
    valid = np.isfinite(depth) & (depth > 0)
    if not np.any(valid):
        cv2.putText(
            display, "No valid depth", (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
        )
        return display
    values = np.asarray(depth[valid], dtype=np.float64)
    lowest = float(values.min())
    highest = float(np.percentile(values, 95))
    gray = np.zeros(depth.shape, dtype=np.uint8)
    if highest > lowest:
        clipped = np.clip(depth, lowest, highest)
        gray[valid] = (
            (clipped[valid] - lowest) / (highest - lowest) * 255.0
        ).astype(np.uint8)
    colored = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    display[valid] = colored[valid]
    return display


def annotate_aruco(image, camera_matrix, distortion, depth=None):
    """Draw marker ID, corners 0-3, and center. PnP stays a separate label."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detector.detectMarkers(gray)
    markers = []
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
            center = points.mean(axis=0)
            center_xy = (int(round(center[0])), int(round(center[1])))
            cv2.circle(image, center_xy, 4, (0, 0, 255), -1)
            cv2.putText(
                image, "C", (center_xy[0] + 6, center_xy[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1,
            )
            for index, point in enumerate(points):
                xy = tuple(np.rint(point).astype(int))
                cv2.circle(image, xy, 3, (0, 0, 255), -1)
                cv2.putText(
                    image, str(index), (xy[0] + 4, xy[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1,
                )
            corner_depth = []
            if depth is not None:
                for index, point in enumerate(points):
                    u, v = np.rint(point).astype(int)
                    result, reason = measure_click(depth, int(u), int(v), camera_matrix, distortion)
                    corner_depth.append({"corner_index": index,
                        "status": "ok" if result is not None else "invalid",
                        "reason": reason, "measurement": result})
            markers.append({
                "id": int(marker_id),
                "corners_px": points.tolist(),
                "corners_depth": corner_depth,
                "center_px": [float(center[0]), float(center[1])],
            })
    pose = aruco.estimate_single_target_pose(
        corners,
        ids,
        camera_matrix,
        distortion,
        image_size_px=(image.shape[1], image.shape[0]),
    )
    pnp_position = None
    if pose["status"] == "ok":
        pnp_position = pose["position_camera_mm"]
        pnp_line = (
            "ArUco PnP auxiliary  "
            f"X={pnp_position['x']:.1f} Y={pnp_position['y']:.1f} "
            f"Z={pnp_position['z']:.1f} mm"
        )
    else:
        pnp_line = f"ArUco PnP auxiliary  {pose['status']}"
    cv2.putText(
        image, pnp_line, (12, image.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1,
    )
    return {
        "role": "auxiliary_not_the_primary_clicked_xyz",
        "pose": pose,
        "corner_depth_note": "Rounded corner pixel, 5x5 depth median; edges may mix background. Not robot correspondences.",
        "dictionary": aruco.DICTIONARY_NAME,
        "markers": markers,
        "pnp_status": pose["status"],
        "pnp_reason": pose["reason"],
        "pnp_position_camera_mm": pnp_position,
        "note": (
            "Marker ID, corners, and center are a visual guide. "
            "pnp_position_camera_mm is the ID 23 tag pose when unique. "
            "The saved sample is the mouse depth XYZ, not this PnP value."
        ),
    }


def draw_probe(image, measurement):
    u = measurement["pixel_u"]
    v = measurement["pixel_v"]
    half = PROBE_HALF_SIZE
    cv2.rectangle(image, (u - half, v - half), (u + half, v + half), (0, 255, 255), 1)
    cv2.drawMarker(
        image, (u, v), (0, 255, 255),
        markerType=cv2.MARKER_CROSS, markerSize=16, thickness=1,
    )


def next_sample_id(session_dir):
    highest = 0
    for path in session_dir.glob("sample_*"):
        suffix = path.name.replace("sample_", "")
        if suffix.isdigit():
            highest = max(highest, int(suffix))
    return highest + 1


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as file:
        json.dump(payload, file, indent=2, ensure_ascii=False, allow_nan=False)


def append_index(path, row):
    fieldnames = [
        "sample_id", "saved_at", "point_label", "pixel_u", "pixel_v",
        "x_mm", "y_mm", "z_mm", "valid_depth_count", "depth_range_mm",
        "device_timestamp_delta_ms",
    ]
    new_file = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        if new_file:
            writer.writeheader()
        writer.writerow(row)


def save_sample(session_dir, sample_id, frozen, measurement, context, point_label, camera_matrix, distortion):
    """Write rgb, marked rgb, depth, and JSON. Success is reported only if all exist."""
    sample_name = f"sample_{sample_id:04d}"
    sample_dir = session_dir / sample_name
    if sample_dir.exists():
        raise FileExistsError(sample_dir)
    sample_dir.mkdir(parents=True)
    rgb_path = sample_dir / "rgb.png"
    marked_path = sample_dir / "marked.png"
    depth_path = sample_dir / "depth.npy"
    json_path = sample_dir / "sample.json"

    marked = frozen["rgb"].copy()
    aruco_annotation = annotate_aruco(marked, camera_matrix, distortion, frozen["depth"])
    draw_probe(marked, measurement)
    xyz = measurement["camera_xyz_mm"]
    cv2.putText(
        marked,
        f"saved depth {sample_name}  X={xyz['x']:.1f} Y={xyz['y']:.1f} Z={xyz['z']:.1f} mm",
        (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2,
    )
    if not cv2.imwrite(str(rgb_path), frozen["rgb"]):
        raise IOError(rgb_path)
    if not cv2.imwrite(str(marked_path), marked):
        raise IOError(marked_path)
    np.save(depth_path, np.asarray(frozen["depth"]))

    saved_at = datetime.now().astimezone().isoformat(timespec="milliseconds")
    record = {
        "session_id": session_dir.name,
        "sample_id": sample_name,
        "saved_at": saved_at,
        "point_label": point_label,
        "record_scope": "camera_only",
        "robot_correspondence": "unverified",
        "robot": {
            "tool": None,
            "reference_frame": None,
            "xyz_mm": None,
        },
        "measurement": measurement,
        "aruco_annotation": aruco_annotation,
        "timing": frozen["timing"],
        "device_mxid": context["device_mxid"],
        "camera_matrix": context["camera_matrix"],
        "distortion_coefficients": context["distortion_coefficients"],
        "distortion_order": DISTORTION_NAMES,
        "distortion_model_name": context["distortion_model_name"],
        "factory_calibration_size_px": context["factory_calibration_size_px"],
        "image_configuration": image_configuration(),
        "opencv_version": cv2.__version__,
        "depthai_version": dai.__version__,
        "files": {
            "rgb": rgb_path.name,
            "marked": marked_path.name,
            "depth": depth_path.name,
        },
    }
    write_json(json_path, record)
    required = [rgb_path, marked_path, depth_path, json_path]
    missing = [path.name for path in required if not path.is_file() or path.stat().st_size <= 0]
    if missing:
        raise IOError(f"Incomplete sample files: {missing}")
    append_index(
        session_dir / "index.csv",
        {
            "sample_id": sample_name,
            "saved_at": saved_at,
            "point_label": point_label,
            "pixel_u": measurement["pixel_u"],
            "pixel_v": measurement["pixel_v"],
            "x_mm": xyz["x"],
            "y_mm": xyz["y"],
            "z_mm": xyz["z"],
            "valid_depth_count": measurement["valid_depth_count"],
            "depth_range_mm": measurement["depth_range_mm"],
            "device_timestamp_delta_ms": frozen["timing"]["device_timestamp_delta_ms"],
        },
    )
    return json_path


def save_invalid_sample(session_dir, sample_id, captured, context, point_label, u, v, reason):
    """Reserve a click's sequence number without inventing a valid XYZ."""
    sample_name = f"sample_{sample_id:04d}"
    sample_dir = session_dir / sample_name
    sample_dir.mkdir(parents=True, exist_ok=False)
    saved_at = datetime.now().astimezone().isoformat(timespec="milliseconds")
    record = {
        "session_id": session_dir.name,
        "sample_id": sample_name,
        "saved_at": saved_at,
        "point_label": point_label,
        "status": "invalid",
        "reason": reason,
        "pixel_u": int(u), "pixel_v": int(v),
        "measurement": None,
        "record_scope": "camera_only",
        "robot_correspondence": "unverified",
        "robot": {"tool": None, "reference_frame": None, "xyz_mm": None},
        "timing": captured["timing"],
        "context": context,
        "files": {"rgb": "rgb.png", "depth": "depth.npy"},
    }
    # Write the failure record first so the number stays reserved even on image I/O failure.
    write_json(sample_dir / "sample.json", record)
    if not cv2.imwrite(str(sample_dir / "rgb.png"), captured["rgb"]):
        raise IOError("Failed to save invalid sample RGB")
    np.save(sample_dir / "depth.npy", captured["depth"])
    append_index(session_dir / "index.csv", {
        "sample_id": sample_name, "saved_at": saved_at, "point_label": point_label,
        "pixel_u": int(u), "pixel_v": int(v),
        "x_mm": None, "y_mm": None, "z_mm": None,
        "valid_depth_count": None, "depth_range_mm": None,
        "device_timestamp_delta_ms": captured["timing"]["device_timestamp_delta_ms"],
    })
    return sample_dir / "sample.json"


def overlay_lines(measurement, reason, frozen, point_label):
    return [reason or "Ready", f"LIVE {point_label} left click = save | q quit"]


def put_lines(image, lines, origin_y=14):
    for index, line in enumerate(lines):
        cv2.putText(
            image, line, (6, origin_y + index * 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1,
        )


def build_pipeline():
    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.ColorCamera)
    camera.setBoardSocket(dai.CameraBoardSocket.RGB)
    camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camera.setPreviewSize(PREVIEW_WIDTH, PREVIEW_HEIGHT)
    camera.setPreviewKeepAspectRatio(True)
    camera.setInterleaved(False)
    camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camera.setFps(30)

    cam_left = pipeline.create(dai.node.MonoCamera)
    cam_right = pipeline.create(dai.node.MonoCamera)
    cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    cam_left.setFps(30)
    cam_right.setFps(30)

    stereo = pipeline.create(dai.node.StereoDepth)
    cam_left.out.link(stereo.left)
    cam_right.out.link(stereo.right)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(False)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setOutputSize(PREVIEW_WIDTH, PREVIEW_HEIGHT)
    stereo.setOutputKeepAspectRatio(True)

    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(timedelta(milliseconds=MAX_PAIR_DELTA_MS))
    sync.setSyncAttempts(-1)
    camera.preview.link(sync.inputs["rgb"])
    stereo.depth.link(sync.inputs["depth"])

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("sync")
    sync.out.link(xout.input)
    return pipeline


def run(point_label):
    pipeline = build_pipeline()
    latest = None
    displayed = None
    cursor = None
    measurement = None
    measure_reason = "Waiting for a synced RGB/depth pair"

    def on_mouse(event, x, y, flags, param):
        nonlocal measurement, measure_reason, cursor
        cursor = (int(x), int(y))
        if event != cv2.EVENT_LBUTTONDOWN or displayed is None:
            return
        # Capture the exact raw pair behind the displayed image, not a future frame.
        captured = displayed
        measurement, reason = measure_click(captured["depth"], x, y, camera_matrix, distortion)
        if measurement is None:
            try:
                path = save_invalid_sample(
                    session_dir, next_sample_id(session_dir), captured,
                    context, point_label, x, y, reason,
                )
                measure_reason = f"INVALID {path.parent.name}: resample later"
                print("Invalid sample recorded:", path, "|", reason)
            except Exception as error:
                measure_reason = f"Invalid sample logging failed: {error}"
            print(measure_reason)
            return
        try:
            path = save_sample(session_dir, next_sample_id(session_dir), captured,
                measurement, context, point_label, camera_matrix, distortion)
        except Exception as error:
            measure_reason = f"Save failed: {error}"
            print(measure_reason)
            return
        measure_reason = f"Saved {path.parent.name} at ({x},{y})"
        print("Saved", path)

    try:
        with dai.Device(pipeline) as device:
            calibration = device.readCalibration()
            distortion_model = calibration.getDistortionModel(dai.CameraBoardSocket.RGB)
            _matrix, calib_w, calib_h = calibration.getDefaultIntrinsics(
                dai.CameraBoardSocket.RGB
            )
            camera_matrix = np.array(
                calibration.getCameraIntrinsics(
                    dai.CameraBoardSocket.RGB, PREVIEW_WIDTH, PREVIEW_HEIGHT
                ),
                dtype=np.float64,
            )
            distortion = np.array(
                calibration.getDistortionCoefficients(dai.CameraBoardSocket.RGB),
                dtype=np.float64,
            ).reshape(-1)
            if distortion.size != 14:
                raise RuntimeError(
                    f"Expected 14 RGB distortion coefficients, got {distortion.size}"
                )
            context = {
                "device_mxid": device.getMxId(),
                "camera_matrix": camera_matrix.tolist(),
                "distortion_coefficients": distortion.tolist(),
                "distortion_model_name": str(distortion_model),
                "factory_calibration_size_px": [int(calib_w), int(calib_h)],
            }
            session_dir = SESSION_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
            session_dir.mkdir(parents=True)
            write_json(
                session_dir / "session.json",
                {
                    "session_id": session_dir.name,
                    "point_label": point_label,
                    "record_scope": "camera_only",
                    "robot_correspondence": "unverified",
                    "created_at": datetime.now().astimezone().isoformat(
                        timespec="milliseconds"
                    ),
                    "context": context,
                    "image_configuration": image_configuration(),
                    "max_pair_delta_ms": MAX_PAIR_DELTA_MS,
                    "opencv_version": cv2.__version__,
                    "depthai_version": dai.__version__,
                },
            )
            queue = device.getOutputQueue(name="sync", maxSize=1, blocking=False)
            cv2.namedWindow("OAK RGB sample", cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow("OAK depth sample", cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback("OAK RGB sample", on_mouse)
            print("Session:", session_dir)
            print("Point label:", point_label)
            print(
                "Left click the live RGB to measure and save its displayed pair. q quits."
            )
            print(
                "ArUco ID, corners 0-3, and center C are a guide. "
                "PnP and corner depth are auxiliary; the primary XYZ is your click."
            )
            print(
                "Do not resize the RGB window. Saved XYZ is the mouse depth click. "
                "This script does not drive the robot."
            )

            while True:
                packet = queue.tryGet()
                if packet is not None:
                    rgb_frame = packet["rgb"]
                    depth_frame = packet["depth"]
                    rgb = np.array(rgb_frame.getCvFrame(), copy=True)
                    depth = np.array(depth_frame.getFrame(), copy=True)
                    timing = pair_timestamps(rgb_frame, depth_frame)
                    ok_shape, shape_reason = geometry_status(rgb, depth)
                    delta = timing["device_timestamp_delta_ms"]
                    if not ok_shape:
                        latest = None
                        measure_reason = shape_reason
                    elif delta > MAX_PAIR_DELTA_MS:
                        latest = None
                        measure_reason = (
                            f"Pair rejected: device dt {delta:.1f} ms "
                            f"> {MAX_PAIR_DELTA_MS:.0f} ms"
                        )
                    else:
                        latest = {"rgb": rgb, "depth": depth, "timing": timing}
                        if displayed is None:
                            measure_reason = "Ready: left click the target to save"

                shown_rgb = None if latest is None else latest["rgb"].copy()
                shown_depth = None if latest is None else latest["depth"]
                if shown_rgb is None:
                    shown_rgb = np.zeros(
                        (PREVIEW_HEIGHT, PREVIEW_WIDTH, 3), dtype=np.uint8
                    )
                    shown_depth_vis = shown_rgb.copy()
                else:
                    shown_depth_vis = colorize_depth(shown_depth)
                    annotate_aruco(shown_rgb, camera_matrix, distortion)
                if cursor is not None:
                    u, v = cursor
                    # Recompute on every displayed pair, even if the mouse is still.
                    hover, hover_reason = measure_click(
                        shown_depth, u, v, camera_matrix, distortion,
                    )
                    for preview in (shown_rgb, shown_depth_vis):
                        cv2.drawMarker(
                            preview, (u, v), (255, 0, 0),
                            markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2,
                        )
                    if hover is None:
                        cursor_line = f"Cursor ({u},{v}): depth unavailable"
                    else:
                        xyz = hover["camera_xyz_mm"]
                        cursor_line = (
                            f"Cursor depth ({u},{v})  X={xyz['x']:.1f} "
                            f"Y={xyz['y']:.1f} Z={xyz['z']:.1f} mm"
                        )
                else:
                    cursor_line = "Move mouse over RGB to preview depth XYZ"
                cv2.putText(
                    shown_rgb, cursor_line, (6, 44),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 2,
                )
                cv2.putText(
                    shown_rgb, cursor_line, (6, 44),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 180, 80), 1,
                )
                put_lines(
                    shown_rgb,
                    overlay_lines(measurement, measure_reason, None, point_label),
                )
                displayed = latest
                cv2.imshow("OAK RGB sample", shown_rgb)
                cv2.imshow("OAK depth sample", shown_depth_vis)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    break
    finally:
        cv2.destroyAllWindows()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Left click the live OAK RGB/depth preview to save a mouse-click "
            "camera XYZ sample. Camera only; no robot transform."
        )
    )
    parser.add_argument(
        "--point-label",
        default="unspecified",
        help="What physical point this click is meant to be. Not a robot frame.",
    )
    return parser.parse_args(argv)


if __name__ == "__main__":
    args = parse_args()
    run(args.point_label)
