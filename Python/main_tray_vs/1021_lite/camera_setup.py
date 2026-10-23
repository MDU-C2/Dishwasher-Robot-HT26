import os
import json
import depthai as dai

_MODEL_DIR = os.path.dirname(os.path.abspath(__file__))
DETECTION_MODELS = {
    "v5": {
        "config": os.path.join(_MODEL_DIR, "blob_v5", "best_Yolo5_small.json"),
        "blob": os.path.join(_MODEL_DIR, "blob_v5", "best_Yolo5_small_openvino_2022.1_5shave.blob"),
    },
    "v5s": {
        "config": os.path.join(_MODEL_DIR, "blob_v5s", "config_legacy.json"),
        "blob": os.path.join(_MODEL_DIR, "blob_v5s", "best.blob"),
    },
    "v8": {
        "config": os.path.join(_MODEL_DIR, "blob_v8n", "config_legacy.json"),
        "blob": os.path.join(_MODEL_DIR, "blob_v8n", "best.blob"),
    },
}


def load_detection_model(model):
    """Return config, blob path, and network input size for v5, v5s, or v8."""
    key = str(model).strip().lower()
    if key not in DETECTION_MODELS:
        raise ValueError(f"Unknown detection model '{model}'. Use v5, v5s, or v8.")
    paths = DETECTION_MODELS[key]
    with open(paths["config"], "r", encoding="utf-8") as f:
        config = json.load(f)
    nn_config = config.get("nn_config", {})
    width_text, height_text = str(nn_config.get("input_size", "640x640")).lower().split("x")
    return {
        "name": key,
        "config_path": paths["config"],
        "blob_path": paths["blob"],
        "config": config,
        "input_width": int(width_text),
        "input_height": int(height_text),
    }


def camera_setup(model="v5"):
    """Build the OAK detection pipeline. Camera-to-robot HT lives in main_tray.py."""
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
    print(f"[MODEL] {detection_model['name']}  input {input_width}x{input_height}")
    print(f"[MODEL] config: {detection_model['config_path']}")
    print(f"[MODEL] blob: {detection_model['blob_path']}")
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

    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setPreviewSize(input_width, input_height)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    if detection_model["name"] in ("v5s", "v8"):
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewKeepAspectRatio(False)
    else:
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)

    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    if detection_model["name"] in ("v5s", "v8"):
        stereo.setOutputSize(input_width, input_height)
        stereo.setOutputKeepAspectRatio(False)
    else:
        stereo.setOutputSize(mono_left.getResolutionWidth(), mono_left.getResolutionHeight())
    stereo.setSubpixel(False)
    stereo.setExtendedDisparity(True)
    print("[DEPTH] mono=400P  ExtendedDisparity=True  Subpixel=False")

    detection_nn.setConfidenceThreshold(confidenceThreshold)
    detection_nn.setNumClasses(classes)
    detection_nn.setCoordinateSize(coordinates)
    detection_nn.setAnchors(anchors)
    detection_nn.setAnchorMasks(anchorMasks)
    detection_nn.setIouThreshold(iouThreshold)
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
