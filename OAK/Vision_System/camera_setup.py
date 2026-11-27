import os
import json
import depthai as dai
import numpy as np

_MODEL_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_MODEL = "yolo11n_416"

# Single source of truth for the 5 unified 3D YOLO models (paper5).
# All 5 models use unified 1080p stretched RGB preview + matching stretched
# StereoDepth output (W x H, keepAspectRatio=False) so preview, aligned depth,
# and [DOT] projection intrinsics always share the exact same geometry.
DETECTION_MODELS = {
    "marc_v5s_640": {
        "menu_id": "1",
        "description": "MARC inherited YOLOv5s 640x640 (8 cls, 5 shaves)",
        "config": os.path.join(_MODEL_DIR, "models", "marc_v5s_640", "depthai_config.json"),
        "blob": os.path.join(_MODEL_DIR, "models", "marc_v5s_640", "marc_v5s_640_5shave.blob"),
        "preview_mode": "stretched_1080p",
    },
    "ft_marc_v5s_416": {
        "menu_id": "2",
        "description": "Fine-tuned MARC YOLOv5s 416x416 (7 cls, 5 shaves)",
        "config": os.path.join(_MODEL_DIR, "models", "ft_marc_v5s_416", "depthai_config.json"),
        "blob": os.path.join(_MODEL_DIR, "models", "ft_marc_v5s_416", "ft_marc_v5s_416_5shave.blob"),
        "preview_mode": "stretched_1080p",
    },
    "yolov5s_416": {
        "menu_id": "3",
        "description": "Custom GreG-4 YOLOv5s 416x416 120ep (6 cls, 5 shaves)",
        "config": os.path.join(_MODEL_DIR, "models", "yolov5s_416", "depthai_config.json"),
        "blob": os.path.join(_MODEL_DIR, "models", "yolov5s_416", "yolov5s_416_5shave.blob"),
        "preview_mode": "stretched_1080p",
    },
    "yolo11n_416": {
        "menu_id": "4",
        "description": "Custom GreG-4 YOLO11n 416x416 120ep (6 cls, 5 shaves) [DEFAULT]",
        "config": os.path.join(_MODEL_DIR, "models", "yolo11n_416", "depthai_config.json"),
        "blob": os.path.join(_MODEL_DIR, "models", "yolo11n_416", "yolo11n_416_5shave.blob"),
        "preview_mode": "stretched_1080p",
    },
    "yolo11n_640": {
        "menu_id": "5",
        "description": "Custom GreG-4 YOLO11n 640x640 120ep (6 cls, 5 shaves)",
        "config": os.path.join(_MODEL_DIR, "models", "yolo11n_640", "depthai_config.json"),
        "blob": os.path.join(_MODEL_DIR, "models", "yolo11n_640", "yolo11n_640_5shave.blob"),
        "preview_mode": "stretched_1080p",
    },
}

MENU_ORDER = [
    "marc_v5s_640",
    "ft_marc_v5s_416",
    "yolov5s_416",
    "yolo11n_416",
    "yolo11n_640",
]

MODEL_ALIASES = {
    "": DEFAULT_MODEL,
    "1": "marc_v5s_640",
    "2": "ft_marc_v5s_416",
    "3": "yolov5s_416",
    "4": "yolo11n_416",
    "5": "yolo11n_640",
    "marc": "marc_v5s_640",
    "ft_marc": "ft_marc_v5s_416",
    "v5s416": "yolov5s_416",
    "v11n416": "yolo11n_416",
    "v11n640": "yolo11n_640",
}


def resolve_model_name(choice):
    """Resolve menu number (1-5), empty/space (default 4), or model name."""
    if choice is None:
        return DEFAULT_MODEL
    key = str(choice).strip().lower()
    if key in DETECTION_MODELS:
        return key
    if key in MODEL_ALIASES:
        return MODEL_ALIASES[key]
    return None


def load_detection_model(model=DEFAULT_MODEL):
    """Return config, blob path, input size, and preview_mode for a registered model."""
    key = resolve_model_name(model)
    if key is None:
        valid = ", ".join(f"{DETECTION_MODELS[m]['menu_id']}={m}" for m in MENU_ORDER)
        raise ValueError(f"Unknown detection model '{model}'. Use {valid}.")
    entry = DETECTION_MODELS[key]
    with open(entry["config"], "r", encoding="utf-8") as f:
        config = json.load(f)
    nn_config = config.get("nn_config", {})
    width_text, height_text = str(nn_config.get("input_size", "416x416")).lower().split("x")
    return {
        "name": key,
        "menu_id": entry["menu_id"],
        "description": entry["description"],
        "config_path": entry["config"],
        "blob_path": entry["blob"],
        "preview_mode": entry["preview_mode"],
        "config": config,
        "input_width": int(width_text),
        "input_height": int(height_text),
    }


# ==================================== FUNCTIONS ====================================
def build_homogeneous(rotation_matrix, translation_vector):
    T_camera_to_base_effector = np.eye(4)
    T_camera_to_base_effector[:3, :3] = rotation_matrix
    T_camera_to_base_effector[:3, 3] = translation_vector.reshape(3)
    return T_camera_to_base_effector


def convert_coordinates(x, y, z, homogeneous_matrix):
    obj_camera_coordinates = np.array([x, y, z])
    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])
    obj_base_effector_coordinates_homo = homogeneous_matrix.dot(obj_camera_coordinates_homo)
    obj_base_coordinates = obj_base_effector_coordinates_homo[:3]
    return np.around(obj_base_coordinates, 2).tolist()


def estimate_rigid_transform(camera_points, robot_points):
    cam = np.asarray(camera_points, dtype=np.float64)
    rob = np.asarray(robot_points, dtype=np.float64)
    assert cam.shape == rob.shape and cam.shape[1] == 3 and cam.shape[0] >= 3, (
        "Value error, differing amount of coordinates, Camera:"
        + str(len(cam))
        + " Robot:"
        + str(len(rob))
    )

    camera_centroid = cam.mean(axis=0)
    robot_centroid = rob.mean(axis=0)
    camera_centered = cam - camera_centroid
    robot_centered = rob - robot_centroid

    cross_covariance = camera_centered.T @ robot_centered
    U, s, Vt = np.linalg.svd(cross_covariance)
    V = Vt.T

    det_correction = np.sign(np.linalg.det(V @ U.T))
    rotation_matrix = V @ np.diag([1.0, 1.0, det_correction]) @ U.T
    translation_vector = robot_centroid - rotation_matrix @ camera_centroid
    return rotation_matrix, translation_vector


def rms_alignment_error(camera_points, robot_points, rotation_matrix, translation_vector):
    cam = np.asarray(camera_points, dtype=np.float64)
    rob = np.asarray(robot_points, dtype=np.float64)
    t = np.asarray(translation_vector, dtype=np.float64).reshape(1, 3)
    predicted_robot_points = (rotation_matrix @ cam.T).T + t
    squared_errors = np.sum((predicted_robot_points - rob) ** 2, axis=1)
    return float(np.sqrt(np.mean(squared_errors)))


def extract_data(file):
    float_list = []
    with open(file, "r") as f:
        for line in f:
            float_list.append(json.loads(line))
    return list(float_list)


# ==================================== CAMERA SETUP ====================================
def camera_setup(model=DEFAULT_MODEL):
    """Build RGB/depth/YOLO pipeline. Fixed robot HT is owned by main_tray1125.

    Returns (pipeline, labels); no calibration point files are required.
    """
    detection_model = load_detection_model(model)
    config = detection_model["config"]
    nnConfig = config.get("nn_config", {})
    metadata = nnConfig.get("NN_specific_metadata", {})
    classes = metadata.get("classes", {})
    coordinates = metadata.get("coordinates", {})
    anchors = metadata.get("anchors", {})
    anchorMasks = metadata.get("anchor_masks", {})
    iouThreshold = metadata.get("iou_threshold", {})
    confidenceThreshold = metadata.get("confidence_threshold", {})
    labels = config.get("mappings", {}).get("labels", {})
    input_width = detection_model["input_width"]
    input_height = detection_model["input_height"]
    preview_mode = detection_model["preview_mode"]

    print(
        f"[MODEL] [{detection_model['menu_id']}] {detection_model['name']}  "
        f"input={input_width}x{input_height}  preview_mode={preview_mode}"
    )
    print(f"[MODEL] config: {detection_model['config_path']}")
    print(f"[MODEL] blob:   {detection_model['blob_path']}")
    print(metadata)

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

    # RGB camera configuration driven by preview_mode (single source of truth).
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setPreviewSize(input_width, input_height)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    if preview_mode == "stretched_1080p":
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewKeepAspectRatio(False)
    elif preview_mode == "center_crop_720p":
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        cam_rgb.setPreviewKeepAspectRatio(True)
    else:
        raise ValueError(f"Unsupported preview_mode '{preview_mode}' for {detection_model['name']}")

    # Mono cameras (for stereo depth)
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    # Stereo depth configuration matched to RGB preview geometry so spatial
    # coordinates project directly back inside each detection bounding box.
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    if preview_mode == "stretched_1080p":
        stereo.setOutputSize(input_width, input_height)
        stereo.setOutputKeepAspectRatio(False)
    else:
        stereo.setOutputSize(input_width, input_height)
        stereo.setOutputKeepAspectRatio(True)
    # 400P + extended covers 28 cm. 720P/800P + extended stops near 35 cm.
    # DepthAI 2.30 cannot enable subpixel together with extended disparity.
    stereo.setSubpixel(False)
    stereo.setExtendedDisparity(True)
    print(
        f"[DEPTH] mono=400P  output={input_width}x{input_height}  "
        f"keepAspectRatio={preview_mode != 'stretched_1080p'}  "
        f"ExtendedDisparity=True  Subpixel=False"
    )

    # Neural network settings
    detection_nn.setConfidenceThreshold(confidenceThreshold)
    detection_nn.setNumClasses(classes)
    detection_nn.setCoordinateSize(coordinates)
    detection_nn.setAnchors(anchors)
    detection_nn.setAnchorMasks(anchorMasks)
    detection_nn.setIouThreshold(iouThreshold)
    detection_nn.setBlobPath(detection_model["blob_path"])
    detection_nn.setNumInferenceThreads(2)
    detection_nn.input.setBlocking(False)

    # Linking
    cam_rgb.preview.link(detection_nn.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(detection_nn.inputDepth)

    detection_nn.passthrough.link(xout_rgb.input)
    detection_nn.passthroughDepth.link(xout_depth.input)
    detection_nn.out.link(xout_nn.input)

    return pipeline, labels
