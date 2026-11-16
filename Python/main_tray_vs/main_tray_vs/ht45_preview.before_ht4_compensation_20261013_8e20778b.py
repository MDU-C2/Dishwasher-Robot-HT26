"""Show one cup through all 45 HT matrices. No socket and no protocol.

Starts YOLOv5s and the OAK camera the same way as main_tray, then draws the
camera XYZ of the red box and the robot XYZ from each matrix. Nothing is sent.

q closes the windows.
"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import cv2 as cv
import depthai as dai
import numpy as np


OUTPUT_DIR = Path(__file__).resolve().parent
HT_FILES = (
    ("raw", OUTPUT_DIR / "ht_15_matrices.py"),
    ("cleaned", OUTPUT_DIR / "ht_15_cleaned_matrices.py"),
    ("reach", OUTPUT_DIR / "ht_15_matrices3.py"),
)
RECOMMENDED = ("cleaned", "HT_New_Combined_CleanedSensitivity")

# Same red-box rule as main_tray in this folder: nearest detection to this
# camera point. X right, Y down, Z forward.
LEFT_ENTRY_CAMERA_XYZ_MM = np.array(
    [-254.3162, -147.0899, 357.3541],
    dtype=np.float64,
)


def find_camera_setup_dir():
    """Directory that contains camera_setup.py and blob_v5s/best.blob."""
    candidates = [
        OUTPUT_DIR,
        OUTPUT_DIR.parent,
        Path(r"C:\Dishwasher loading and unloading robot\OAK\python"),
        Path(r"D:\python"),
    ]
    extra_roots = [
        Path(r"D:\python"),
        Path(r"C:\Dishwasher loading and unloading robot\OAK"),
    ]
    for root in extra_roots:
        if not root.is_dir():
            continue
        for child in root.iterdir():
            candidates.append(child)
            candidates.append(child / "python")
    seen = set()
    for folder in candidates:
        folder = folder.resolve()
        if folder in seen or not folder.is_dir():
            continue
        seen.add(folder)
        if (folder / "camera_setup.py").is_file() and (folder / "blob_v5s" / "best.blob").is_file():
            return folder
    raise FileNotFoundError(
        "camera_setup.py and blob_v5s/best.blob were not found next to this script "
        "or under D:\\python or the project OAK\\python folder."
    )


def load_camera_setup():
    folder = find_camera_setup_dir()
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))
    import camera_setup as cs
    print(f"[MODEL] camera_setup: {folder}")
    return cs


def load_ht_rows():
    """45 matrices: 15 from each of the three output files, in file order."""
    rows = []
    for group, path in HT_FILES:
        if not path.is_file():
            raise FileNotFoundError(path)
        spec = spec_from_file_location(f"ht_group_{group}", path)
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        for index, (name, matrix) in enumerate(module.MATRICES.items(), start=1):
            rows.append({
                "group": group,
                "index": index,
                "name": name,
                "matrix": np.asarray(matrix, dtype=np.float64),
                "recommended": (group, name) == RECOMMENDED,
            })
    if len(rows) != 45:
        raise RuntimeError(f"Expected 45 HT matrices, found {len(rows)}")
    return rows


def robot_xyz(camera_xyz, homogeneous):
    """Same product as camera_setup.convert_coordinates, kept as floats."""
    point = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], 1.0], dtype=np.float64)
    return homogeneous.dot(point)[:3]


def select_nearest_target(detections, labels):
    """Nearest valid detection to the left-entry reference. Mug pose is not used."""
    reference = LEFT_ENTRY_CAMERA_XYZ_MM.reshape(3)
    selected_target = None
    selected_distance = None
    for det in detections:
        if not 0 <= det.label < len(labels):
            continue
        if labels[det.label] == "handle":
            continue
        coords = det.spatialCoordinates
        if not np.isfinite([coords.x, coords.y, coords.z]).all() or coords.z <= 0:
            continue
        point = np.array([coords.x, coords.y, coords.z], dtype=np.float64)
        distance = float(np.linalg.norm(point - reference))
        if selected_target is None or distance < selected_distance:
            selected_target = det
            selected_distance = distance
    return selected_target


def build_v5s_pipeline(cs):
    """YOLOv5s pipeline copied from camera_setup.camera_setup, without a robot file."""
    detection_model = cs.load_detection_model("v5s")
    config = detection_model["config"]
    metadata = config.get("nn_config", {}).get("NN_specific_metadata", {})
    labels = config.get("mappings", {}).get("labels", [])
    print(f"[MODEL] v5s  input {detection_model['input_width']}x{detection_model['input_height']}")
    print(f"[MODEL] blob: {detection_model['blob_path']}")

    pipeline = dai.Pipeline()
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    detection_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_nn = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")
    xout_nn.setStreamName("detections")

    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setPreviewSize(detection_model["input_width"], detection_model["input_height"])
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)

    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setOutputSize(mono_left.getResolutionWidth(), mono_left.getResolutionHeight())
    stereo.setSubpixel(True)

    detection_nn.setConfidenceThreshold(metadata.get("confidence_threshold", 0.5))
    detection_nn.setNumClasses(metadata.get("classes", 7))
    detection_nn.setCoordinateSize(metadata.get("coordinates", 4))
    detection_nn.setAnchors(metadata.get("anchors", []))
    detection_nn.setAnchorMasks(metadata.get("anchor_masks", {}))
    detection_nn.setIouThreshold(metadata.get("iou_threshold", 0.5))
    detection_nn.setBlobPath(detection_model["blob_path"])
    detection_nn.setNumInferenceThreads(2)
    detection_nn.input.setBlocking(False)

    cam_rgb.preview.link(detection_nn.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(detection_nn.inputDepth)
    detection_nn.passthrough.link(xout_rgb.input)
    detection_nn.passthroughDepth.link(xout_depth.input)
    detection_nn.out.link(xout_nn.input)
    return pipeline, labels


def load_preview_projection(device, width, height):
    """RGB intrinsics for the stretched 416x416 preview. XYZ stays in millimeters."""
    calibration = device.readCalibration()
    distortion = np.asarray(
        calibration.getDistortionCoefficients(dai.CameraBoardSocket.RGB),
        dtype=np.float64,
    ).reshape(-1)
    if distortion.size != 14 or not np.isfinite(distortion).all():
        raise ValueError("Expected 14 finite RGB distortion coefficients")
    matrix = np.asarray(
        calibration.getCameraIntrinsics(
            dai.CameraBoardSocket.RGB,
            int(width),
            int(height),
            keepAspectRatio=False,
        ),
        dtype=np.float64,
    )
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("RGB intrinsics are not a finite 3x3 matrix")
    print(
        f"[HT] point projection {width}x{height} "
        f"fx={matrix[0, 0]:.1f} fy={matrix[1, 1]:.1f} "
        f"cx={matrix[0, 2]:.1f} cy={matrix[1, 2]:.1f}"
    )
    return matrix, distortion


def project_spatial_pixel(coords, camera_matrix, distortion, image_size):
    """Project one camera XYZ back onto the preview. None means it is not drawable.

    SpatialImgDetection is X right, Y up, Z forward. This preview uses optical
    Y down, so the dot uses (x, -y, z). The XYZ sent to HT45 is not changed.
    """
    height, width = image_size
    if not np.isfinite([coords.x, coords.y, coords.z]).all() or coords.z <= 0:
        return None
    try:
        projected, _ = cv.projectPoints(
            np.array([[coords.x, -coords.y, coords.z]], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            camera_matrix,
            distortion,
        )
    except cv.error:
        return None
    u, v = projected.reshape(2)
    if not (np.isfinite(u) and np.isfinite(v)):
        return None
    if not (0 <= u < width and 0 <= v < height):
        return None
    return int(round(u)), int(round(v))


def draw_detections(frame, detections, labels, selected_target, camera_matrix, distortion):
    for det in detections:
        label = labels[det.label] if 0 <= det.label < len(labels) else str(det.label)
        if label == "handle":
            continue
        x1 = int(det.xmin * frame.shape[1])
        y1 = int(det.ymin * frame.shape[0])
        x2 = int(det.xmax * frame.shape[1])
        y2 = int(det.ymax * frame.shape[0])
        color = (0, 0, 255) if det is selected_target else (0, 255, 0)
        cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        coords = det.spatialCoordinates
        cv.putText(frame, f"{label} ({int(det.confidence * 100)}%)", (x1 + 5, y1 + 20),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv.putText(frame, f"X: {int(coords.x)} mm", (x1 + 5, y1 + 35),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv.putText(frame, f"Y: {int(coords.y)} mm", (x1 + 5, y1 + 50),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv.putText(frame, f"Z: {int(coords.z)} mm", (x1 + 5, y1 + 65),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        pixel = project_spatial_pixel(coords, camera_matrix, distortion, frame.shape[:2])
        if pixel is None:
            continue
        if det is selected_target:
            # Red: the one camera point shown in the HT45 panel.
            cv.circle(frame, pixel, 5, (255, 255, 255), -1)
            cv.circle(frame, pixel, 3, (0, 0, 255), -1)
        else:
            # Cyan: every other detection. BGR, so (255, 255, 0).
            cv.circle(frame, pixel, 4, (0, 0, 0), -1)
            cv.circle(frame, pixel, 2, (255, 255, 0), -1)
    cv.putText(frame, "q quit   no robot link", (10, 18),
               cv.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    cv.putText(frame, "red dot = HT45    cyan dot = other", (10, 36),
               cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)


def render_ht_panel(rows, camera_xyz, label):
    panel = np.full((820, 980, 3), 24, dtype=np.uint8)
    if camera_xyz is None:
        header = "no red-box target"
    else:
        header = (
            f"{label}  cam {camera_xyz[0]:7.1f} {camera_xyz[1]:7.1f} {camera_xyz[2]:7.1f} mm"
        )
    cv.putText(panel, header, (12, 28), cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv.putText(panel, "group    matrix                         robot X      Y      Z",
               (12, 52), cv.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
    for row_number, row in enumerate(rows):
        y = 74 + row_number * 16
        if camera_xyz is None:
            robot = "      -        -        -"
        else:
            robot_point = robot_xyz(camera_xyz, row["matrix"])
            robot = f"{robot_point[0]:8.1f} {robot_point[1]:8.1f} {robot_point[2]:8.1f}"
        mark = "  <<" if row["recommended"] else ""
        text = f"{row['index']:02d} {row['group']:<7} {row['name']:<34} {robot}{mark}"
        color = (0, 220, 0) if row["recommended"] else (230, 230, 230)
        cv.putText(panel, text, (12, y), cv.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
    return panel


def run():
    rows = load_ht_rows()
    cs = load_camera_setup()
    pipeline, labels = build_v5s_pipeline(cs)
    print(f"[HT] {len(rows)} matrices. Recommended: {RECOMMENDED[0]} / {RECOMMENDED[1]}")
    print("[HT] Windows only. No coordinates are sent.")
    with dai.Device(pipeline) as device:
        # Depth is not drawn, but the queue must be read or the pipeline stops
        # after the first frames. Same three reads as main_tray.
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        projection = None
        while True:
            in_rgb = q_rgb.get()
            q_depth.get()
            in_det = q_det.get()
            frame = in_rgb.getCvFrame()
            if projection is None:
                projection = load_preview_projection(device, frame.shape[1], frame.shape[0])
            camera_matrix, distortion = projection
            detections = in_det.detections
            selected = select_nearest_target(detections, labels)
            draw_detections(frame, detections, labels, selected, camera_matrix, distortion)
            if selected is None:
                camera_xyz = None
                label = ""
            else:
                coords = selected.spatialCoordinates
                camera_xyz = np.array([coords.x, coords.y, coords.z], dtype=np.float64)
                label = labels[selected.label] if 0 <= selected.label < len(labels) else str(selected.label)
            cv.imshow("RGB", frame)
            cv.imshow("HT45", render_ht_panel(rows, camera_xyz, label))
            if (cv.waitKey(1) & 0xFF) == ord("q"):
                break
    cv.destroyAllWindows()


if __name__ == "__main__":
    run()
