"""Show one cup through four finalist HT matrices with X compensation.

Starts YOLOv5s and the OAK camera the same way as main_tray, then draws the
camera XYZ of the red box and the compensated robot XYZ from each finalist.
Only X is corrected; Y/Z are the original HT outputs. No socket or protocol.

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
)

# 2026-10-13 compara, nine cup positions. For each specific HT:
# delta_X = X_HT - X_controller = slope * X_controller + intercept_mm.
# Invert that relation AFTER applying HT: X_corrected = (X_HT - b) / (1 + a).
# These are empirical fits, not independently validated calibration results.
FINALISTS = (
    ("A", "raw", "HT_New_Run2_Mean", -0.2887334368353747, 142.60785325637488),
    ("B", "raw", "HT_Global", -0.30982101064125184, 142.84357866253285),
    ("C", "cleaned", "HT_Old_Run1_Mean_CleanedSensitivity", -0.3082109754172751, 149.42298248252223),
    ("D", "cleaned", "HT_New_Combined_CleanedSensitivity", -0.3152292685386935, 156.0796216563615),
)

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
    """Load only the four finalists, in A/B/C/D order, with their own X fits."""
    matrices_by_group = {}
    for group, path in HT_FILES:
        if not path.is_file():
            raise FileNotFoundError(path)
        spec = spec_from_file_location(f"ht_group_{group}", path)
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        matrices_by_group[group] = module.MATRICES
    rows = []
    for finalist_id, group, name, slope, intercept_mm in FINALISTS:
        if name not in matrices_by_group[group]:
            raise ValueError(f"Missing finalist matrix: {group} / {name}")
        matrix = np.asarray(matrices_by_group[group][name], dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError(f"Invalid finalist matrix: {group} / {name}")
        rows.append({
            "id": finalist_id,
            "group": group,
            "name": name,
            "matrix": matrix,
            "x_slope": slope,
            "x_intercept_mm": intercept_mm,
        })
    return rows


def robot_xyz(camera_xyz, homogeneous):
    """Same product as camera_setup.convert_coordinates, kept as floats."""
    point = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], 1.0], dtype=np.float64)
    return homogeneous.dot(point)[:3]


def compensate_robot_x(point, slope, intercept_mm):
    """Invert this HT's fitted X bias, leaving Y/Z and the input unchanged."""
    corrected = np.array(point, dtype=np.float64, copy=True)
    scale = 1.0 + slope
    if corrected.shape != (3,) or not np.isfinite(corrected).all():
        raise ValueError("Expected three finite robot coordinates in mm")
    if not np.isfinite([scale, intercept_mm]).all() or abs(scale) < 1e-12:
        raise ValueError("X compensation requires a finite, nonzero fitted scale")
    corrected[0] = (corrected[0] - intercept_mm) / scale
    return corrected


def compensated_robot_xyz(camera_xyz, row):
    """Camera XYZ -> finalist HT -> that finalist's X compensation."""
    point = robot_xyz(camera_xyz, row["matrix"])
    return compensate_robot_x(point, row["x_slope"], row["x_intercept_mm"])


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


def draw_detections(frame, detections, labels, selected_target):
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
    cv.putText(frame, "q quit   no robot link", (10, 24),
               cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)


def render_ht_panel(rows, camera_xyz, label):
    panel = np.full((420, 1000, 3), 24, dtype=np.uint8)
    if camera_xyz is None:
        header = "no red-box target"
    else:
        header = (
            f"{label}  cam {camera_xyz[0]:7.1f} {camera_xyz[1]:7.1f} {camera_xyz[2]:7.1f} mm"
        )
    cv.putText(panel, header, (12, 28), cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv.putText(panel, "HT4 | X compensated | Y/Z from HT | all coordinates in mm",
               (12, 54), cv.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    for row_number, row in enumerate(rows):
        y = 90 + row_number * 74
        title = f"{row['id']}  {row['group']} / {row['name']}"
        cv.putText(panel, title, (12, y), cv.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)
        if camera_xyz is None:
            robot = "X:        -     Y:        -     Z:        -"
        else:
            robot_point = compensated_robot_xyz(camera_xyz, row)
            robot = f"X: {robot_point[0]:8.1f}     Y: {robot_point[1]:8.1f}     Z: {robot_point[2]:8.1f}"
        cv.putText(panel, robot, (32, y + 29), cv.FONT_HERSHEY_SIMPLEX, 0.65, (230, 230, 230), 1)
    cv.putText(panel, "q quit   no robot link", (12, 396),
               cv.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    return panel


def run():
    rows = load_ht_rows()
    cs = load_camera_setup()
    pipeline, labels = build_v5s_pipeline(cs)
    print(f"[HT] {len(rows)} finalists. X compensation ON; Y/Z unchanged.")
    for row in rows:
        print(f"[HT] {row['id']} {row['group']} / {row['name']}")
    print("[HT] Windows only. No coordinates are sent.")
    with dai.Device(pipeline) as device:
        # Depth is not drawn, but the queue must be read or the pipeline stops
        # after the first frames. Same three reads as main_tray.
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        while True:
            in_rgb = q_rgb.get()
            q_depth.get()
            in_det = q_det.get()
            frame = in_rgb.getCvFrame()
            detections = in_det.detections
            selected = select_nearest_target(detections, labels)
            draw_detections(frame, detections, labels, selected)
            if selected is None:
                camera_xyz = None
                label = ""
            else:
                coords = selected.spatialCoordinates
                camera_xyz = np.array([coords.x, coords.y, coords.z], dtype=np.float64)
                label = labels[selected.label] if 0 <= selected.label < len(labels) else str(selected.label)
            cv.imshow("RGB", frame)
            cv.imshow("HT4 - X compensated", render_ht_panel(rows, camera_xyz, label))
            if (cv.waitKey(1) & 0xFF) == ord("q"):
                break
    cv.destroyAllWindows()


if __name__ == "__main__":
    run()
