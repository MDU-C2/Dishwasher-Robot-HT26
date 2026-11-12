import os
import cv2 as cv
import depthai as dai
import numpy as np
import json

_MODEL_DIR = os.path.dirname(os.path.abspath(__file__))
_V11Z_BLOB = os.path.join(_MODEL_DIR, "blob_v11z", "v11Z.blob")
if not os.path.isfile(_V11Z_BLOB):
    _V11Z_BLOB = os.path.join(_MODEL_DIR, "v11Z.blob")

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
    "v11z": {
        "config": os.path.join(_MODEL_DIR, "blob_v11z", "config_legacy.json"),
        "blob": _V11Z_BLOB,
    },
}
_MODEL_ALIASES = {
    "5": "v5",
    "5s": "v5s",
    "8": "v8",
    "11": "v11z",
    "11z": "v11z",
    "11s": "v11z",
    "v11": "v11z",
    "v11s": "v11z",
    "v11z": "v11z",
}


def _uses_raw_yolo_output(blob_path):
    """Return True if the blob outputs a single raw YOLO tensor (e.g. output0 [8400, 4+C, 1]) instead of split grid heads."""
    try:
        blob = dai.OpenVINO.Blob(blob_path)
        outputs = list(blob.networkOutputs.keys())
        if len(outputs) == 1:
            name = outputs[0].lower()
            if not any(tag in name for tag in ("yolov5", "yolov6", "yolov7", "yolov8", "yolov10")):
                return True
    except Exception:
        pass
    return False


def load_detection_model(model):
    """Return config, blob path, and network input size for v5, v5s, v8, or v11z."""
    key = str(model).strip().lower()
    key = _MODEL_ALIASES.get(key, key)
    if key not in DETECTION_MODELS:
        raise ValueError(f"Unknown detection model '{model}'. Use v5, v5s, v8, or v11z.")
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
        "raw_yolo_output": _uses_raw_yolo_output(paths["blob"]),
    }




#==================================== FUNCTIONS ====================================
def build_homogeneous(rotation_matrix, translation_vector):
    T_camera_to_base_effector = np.eye(4)
    T_camera_to_base_effector[:3, :3] = rotation_matrix
    T_camera_to_base_effector[:3, 3] = translation_vector.reshape(3)
    return T_camera_to_base_effector

def convert_coordinates(x ,y ,z, homogeneous_matrix): # X Y Z coordinates that should be translated into robot frame coordinates
    obj_camera_coordinates = np.array([x, y, z])
    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # Convert object coordinates to homogeneous coordinates
    obj_base_effector_coordinates_homo = homogeneous_matrix.dot(obj_camera_coordinates_homo)
    obj_base_coordinates = obj_base_effector_coordinates_homo[:3]  
    
    #return list(map(int, obj_base_coordinates)) # Uncomment this line if you want to send integers instead of floats
    return np.around(obj_base_coordinates,2).tolist() # Use this to get a list of new coordinates, chane the number to get the number of decimal numbers

def estimate_rigid_transform(camera_points, robot_points):
    cam = np.asarray(camera_points, dtype=np.float64)
    rob = np.asarray(robot_points,  dtype=np.float64)
    assert cam.shape == rob.shape and cam.shape[1] == 3 and cam.shape[0] >= 3, "Value error, differing amount of coordinates, Camera:" + str(len(cam)) + " Robot:"+ str(len(rob))
    

    camera_centroid = cam.mean(axis=0)
    robot_centroid  = rob.mean(axis=0)
    camera_centered = cam - camera_centroid
    robot_centered  = rob - robot_centroid

    cross_covariance = camera_centered.T @ robot_centered
    U, s, Vt = np.linalg.svd(cross_covariance)
    V = Vt.T

    # Ensure of proper rotation (det=+1)
    det_correction = np.sign(np.linalg.det(V @ U.T))
    rotation_matrix = V @ np.diag([1.0, 1.0, det_correction]) @ U.T

    translation_vector = robot_centroid - rotation_matrix @ camera_centroid
    return rotation_matrix, translation_vector

def rms_alignment_error(camera_points, robot_points, rotation_matrix, translation_vector):
    cam = np.asarray(camera_points, dtype=np.float64)
    rob = np.asarray(robot_points,  dtype=np.float64)
    t = np.asarray(translation_vector, dtype=np.float64).reshape(1, 3)
    predicted_robot_points = (rotation_matrix @ cam.T).T + t
    squared_errors = np.sum((predicted_robot_points - rob) ** 2, axis=1)
    rms_error = float(np.sqrt(np.mean(squared_errors)))
    return rms_error


def extract_data(file):
    
    float_list=[]
    with open(file, "r") as f:
        lines = f.readlines()
        for i in lines:
            x = json.loads(i)
            float_list.append(x)
    return list(float_list)

def cam_calibration():
     # Create DepthAI pipeline
    pipeline = dai.Pipeline()

    # Define the camera nodes
    cam_rgb   = pipeline.create(dai.node.ColorCamera)
    mono_left  = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo     = pipeline.create(dai.node.StereoDepth)
    detection_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

    # Define XLink outputs for streaming frames and detections to host
    xout_rgb   = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_nn    = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")
    xout_nn.setStreamName("detections")

    # Camera configuration (RGB camera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
    cam_rgb.setPreviewSize(640, 640) # Neural network input size. (MUST MATCH THE RUNNING MODELS SIZE)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # Mono cameras (for depth)
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    # Stereo depth configuration
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)       
    stereo.setOutputSize(mono_left.getResolutionWidth(), mono_left.getResolutionHeight())
    # 400P + extended covers 28 cm. 720P/800P + extended stops near 35 cm.
    # DepthAI 2.30 cannot enable subpixel together with extended disparity.
    stereo.setSubpixel(False)
    stereo.setExtendedDisparity(True)
    print("[DEPTH] mono=400P  ExtendedDisparity=True  Subpixel=False") 

    configPath = "blob_aruco/best_1.json" # Path to the config JSON file for the model (MUST BE CHANGED IF YOU WANT TO USE A NEW MODEL)
    with open(configPath, "r") as f:
        config = json.load(f)
    nnConfig = config.get("nn_config", {})

    # Extract the metadata from the JSON file
    metadata = nnConfig.get("NN_specific_metadata", {})
    classes = metadata.get("classes", {})
    coordinates = metadata.get("coordinates", {})
    anchors = metadata.get("anchors", {})
    anchorMasks = metadata.get("anchor_masks", {})
    iouThreshold = metadata.get("iou_threshold", {})
    confidenceThreshold = metadata.get("confidence_threshold", {})

    print(metadata)

    nnMappings = config.get("mappings", {})
    labels = nnMappings.get("labels", {})

    nnPath = "blob_aruco/best_1_openvino_2022.1_5shave.blob" # PATH TO THE .BLOB FILE (MUST BE CHANGED IF YOU WANT TO USE A NEW MODEL)

    # Specific settings for the network
    detection_nn.setConfidenceThreshold(confidenceThreshold)
    detection_nn.setNumClasses(classes)
    detection_nn.setCoordinateSize(coordinates)
    detection_nn.setAnchors(anchors)
    detection_nn.setAnchorMasks(anchorMasks)
    detection_nn.setIouThreshold(iouThreshold)
    detection_nn.setBlobPath(nnPath)
    detection_nn.setNumInferenceThreads(2)
    detection_nn.input.setBlocking(False)

    syncNN = True

    # Linking
    cam_rgb.preview.link(detection_nn.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(detection_nn.inputDepth)

    detection_nn.passthrough.link(xout_rgb.input) # Passthrough RGB frames (frames that went into NN)
    detection_nn.passthroughDepth.link(xout_depth.input) # Aligned depth frames
    detection_nn.out.link(xout_nn.input) # Detection outputs (bounding boxes + coordinates)

    return  syncNN, pipeline, labels


    #==================================== CAMERA & COMMUNICATION SETUP ====================================
def camera_setup(cam_coords, robot_file, model="v5"):


    camera_points = extract_data(cam_coords) # Get coordinates from the camera 
    robot_points = extract_data(robot_file) # Get coordinats from the robot (both .txt files)
    
    rotation_matrix, translation_vector = estimate_rigid_transform(camera_points, robot_points) # Do an estimation from both the 3d robot and camera coordinates to get rotation matrix and translation vector
    homogeneous = build_homogeneous(rotation_matrix,translation_vector) # Convert the rotation and translation into a 4x4 homogeneous matrix that can be used to convert camera coordinates into robotframe

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

    # Create DepthAI pipeline
    pipeline = dai.Pipeline()

    # Define the camera nodes
    cam_rgb   = pipeline.create(dai.node.ColorCamera)
    mono_left  = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo     = pipeline.create(dai.node.StereoDepth)
    if detection_model.get("raw_yolo_output"):
        detection_nn = pipeline.create(dai.node.NeuralNetwork)
    else:
        detection_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

    # Define XLink outputs for streaming frames and detections to host
    xout_rgb   = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_nn    = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")
    xout_nn.setStreamName("detections")

    # Camera configuration (RGB camera). Preview size comes from the selected model.
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setPreviewSize(input_width, input_height)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    if detection_model["name"] in ("v5s", "v8", "v11z"):
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewKeepAspectRatio(False)
    else:
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)

    # Mono cameras (for depth)
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    # Stereo depth configuration
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    if detection_model["name"] in ("v5s", "v8", "v11z"):
        stereo.setOutputSize(input_width, input_height)
        stereo.setOutputKeepAspectRatio(False)
    else:
        stereo.setOutputSize(mono_left.getResolutionWidth(), mono_left.getResolutionHeight())
    # 400P + extended covers 28 cm. 720P/800P + extended stops near 35 cm.
    # DepthAI 2.30 cannot enable subpixel together with extended disparity.
    stereo.setSubpixel(False)
    stereo.setExtendedDisparity(True)
    print("[DEPTH] mono=400P  ExtendedDisparity=True  Subpixel=False")

    syncNN = True

    if detection_model.get("raw_yolo_output"):
        print(f"[MODEL] Raw YOLO output tensor detected for {detection_model['name']} -> using host-side YOLO11/v8 decoder + spatial depth calculator.")
        detection_nn.setBlobPath(detection_model["blob_path"])
        detection_nn.setNumInferenceThreads(1)
        detection_nn.input.setBlocking(False)

        # Linking for raw NeuralNetwork + aligned StereoDepth
        cam_rgb.preview.link(detection_nn.input)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        detection_nn.passthrough.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)
        detection_nn.out.link(xout_nn.input)
    else:
        # Specific settings for the network
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

        detection_nn.passthrough.link(xout_rgb.input) # Passthrough RGB frames (frames that went into NN)
        detection_nn.passthroughDepth.link(xout_depth.input) # Aligned depth frames
        detection_nn.out.link(xout_nn.input) # Detection outputs (bounding boxes + coordinates)

    return homogeneous, syncNN, pipeline, labels, rotation_matrix, translation_vector, camera_points, robot_points


_CALIB_INTRINSICS_CACHE = {}


def _get_rgb_intrinsics(device, width, height):
    """Read and cache RGB camera intrinsics scaled to (width, height)."""
    key = (id(device), int(width), int(height))
    if key not in _CALIB_INTRINSICS_CACHE:
        try:
            calib = device.readCalibration()
            M = np.array(
                calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, 1920, 1080),
                dtype=np.float64,
            )
            sx = float(width) / 1920.0
            sy = float(height) / 1080.0
            fx = float(M[0, 0]) * sx
            fy = float(M[1, 1]) * sy
            cx = float(M[0, 2]) * sx
            cy = float(M[1, 2]) * sy
        except Exception:
            # Fallback approximate intrinsics for OAK-D RGB stretched to width x height
            fx = 1500.0 * (float(width) / 1920.0)
            fy = 1500.0 * (float(height) / 1080.0)
            cx = float(width) * 0.5
            cy = float(height) * 0.5
        _CALIB_INTRINSICS_CACHE[key] = (fx, fy, cx, cy)
    return _CALIB_INTRINSICS_CACHE[key]


def extract_detections(in_dets, in_depth=None, device=None, model_info=None):
    """Return a list of SpatialImgDetection objects from either ImgDetections or raw YOLO NNData."""
    if hasattr(in_dets, "detections"):
        return in_dets.detections

    if model_info is None:
        return []

    config = model_info.get("config", {})
    metadata = config.get("nn_config", {}).get("NN_specific_metadata", {})
    num_classes = int(metadata.get("classes", 7))
    conf_thres = float(metadata.get("confidence_threshold", 0.5))
    iou_thres = float(metadata.get("iou_threshold", 0.5))
    input_w = float(model_info.get("input_width", 640))
    input_h = float(model_info.get("input_height", 640))

    fp16_vals = in_dets.getFirstLayerFp16() if hasattr(in_dets, "getFirstLayerFp16") else []
    if len(fp16_vals) == 0:
        layer_names = in_dets.getAllLayerNames()
        if not layer_names:
            return []
        fp16_vals = in_dets.getLayerFp16(layer_names[0])
    raw = np.array(fp16_vals, dtype=np.float32)
    channels = 4 + num_classes
    if raw.size % channels != 0:
        return []

    preds = raw.reshape(channels, -1)  # shape: (4 + num_classes, num_anchors) e.g. (11, 8400)
    boxes_cxcywh = preds[:4, :]
    class_scores = preds[4:, :]

    class_ids = np.argmax(class_scores, axis=0)
    max_scores = class_scores[class_ids, np.arange(class_scores.shape[1])]

    mask = max_scores >= conf_thres
    if not np.any(mask):
        return []

    boxes_cxcywh = boxes_cxcywh[:, mask].T  # (N, 4)
    scores = max_scores[mask]
    c_ids = class_ids[mask]

    cx_arr = boxes_cxcywh[:, 0]
    cy_arr = boxes_cxcywh[:, 1]
    w_arr = boxes_cxcywh[:, 2]
    h_arr = boxes_cxcywh[:, 3]

    # Normalize to [0, 1] if coordinates are in pixels (e.g. 0..640)
    if np.max(boxes_cxcywh) > 2.0:
        cx_arr = cx_arr / input_w
        w_arr = w_arr / input_w
        cy_arr = cy_arr / input_h
        h_arr = h_arr / input_h

    xmin_arr = np.clip(cx_arr - 0.5 * w_arr, 0.0, 1.0)
    ymin_arr = np.clip(cy_arr - 0.5 * h_arr, 0.0, 1.0)
    xmax_arr = np.clip(cx_arr + 0.5 * w_arr, 0.0, 1.0)
    ymax_arr = np.clip(cy_arr + 0.5 * h_arr, 0.0, 1.0)

    nms_boxes = []
    for i in range(len(scores)):
        bx = float(xmin_arr[i] * input_w)
        by = float(ymin_arr[i] * input_h)
        bw = float((xmax_arr[i] - xmin_arr[i]) * input_w)
        bh = float((ymax_arr[i] - ymin_arr[i]) * input_h)
        nms_boxes.append([bx, by, bw, bh])

    indices = cv.dnn.NMSBoxes(nms_boxes, scores.tolist(), conf_thres, iou_thres)
    if len(indices) == 0:
        return []

    depth_frame = in_depth.getFrame() if in_depth is not None else None
    if depth_frame is not None and device is not None:
        dh, dw = depth_frame.shape[:2]
        fx, fy, opt_cx, opt_cy = _get_rgb_intrinsics(device, dw, dh)
    else:
        dh, dw = int(input_h), int(input_w)
        fx, fy, opt_cx, opt_cy = 1.0, 1.0, dw * 0.5, dh * 0.5

    results = []
    for idx in np.array(indices).flatten():
        det = dai.SpatialImgDetection()
        det.label = int(c_ids[idx])
        det.confidence = float(scores[idx])
        det.xmin = float(xmin_arr[idx])
        det.ymin = float(ymin_arr[idx])
        det.xmax = float(xmax_arr[idx])
        det.ymax = float(ymax_arr[idx])

        x_mm, y_mm, z_mm = 0.0, 0.0, 0.0
        if depth_frame is not None:
            # Use center 50% of bounding box (scale factor 0.5, matching YoloSpatialDetectionNetwork default)
            box_w = det.xmax - det.xmin
            box_h = det.ymax - det.ymin
            roi_xmin = det.xmin + 0.25 * box_w
            roi_xmax = det.xmax - 0.25 * box_w
            roi_ymin = det.ymin + 0.25 * box_h
            roi_ymax = det.ymax - 0.25 * box_h

            px1 = max(0, min(dw - 1, int(roi_xmin * dw)))
            px2 = max(px1 + 1, min(dw, int(roi_xmax * dw)))
            py1 = max(0, min(dh - 1, int(roi_ymin * dh)))
            py2 = max(py1 + 1, min(dh, int(roi_ymax * dh)))

            roi = depth_frame[py1:py2, px1:px2]
            valid = roi[(roi > 100) & (roi < 10000)]
            if valid.size > 0:
                z_mm = float(np.mean(valid))
                u_center = 0.5 * (det.xmin + det.xmax) * dw
                v_center = 0.5 * (det.ymin + det.ymax) * dh
                # DepthAI spatial coordinate convention: +X right, +Y up, +Z forward
                x_mm = (u_center - opt_cx) * z_mm / fx
                y_mm = (opt_cy - v_center) * z_mm / fy

        det.spatialCoordinates.x = float(x_mm)
        det.spatialCoordinates.y = float(y_mm)
        det.spatialCoordinates.z = float(z_mm)
        results.append(det)

    return results

def camera_setup2(cam_coords, robot_file):
    best_comb =  [9, 10, 4, 53, 38, 23, 78, 73, 72, 56, 75, 50, 21, 26, 22, 70, 35, 1, 13, 19, 30, 5, 77, 14, 18, 52, 68, 34, 62, 43, 29, 54, 41, 59, 42, 63, 7, 16, 67, 47, 48, 33, 74, 51, 37, 55, 44, 2, 32, 3, 0, 31, 64, 6, 8, 40, 36, 66, 20, 28, 61, 76, 57, 39, 15, 25, 27, 60, 11, 17, 45, 71, 12, 65, 24, 46, 49, 58]
    cam_points = []
    rob_points = []
    camera_points = extract_data(cam_coords) # Get coordinates from the camera 
    robot_points = extract_data(robot_file) # Get coordinats from the robot (both .txt files)
    for comb in best_comb:
        cam_points.append(camera_points[comb])
        rob_points.append(robot_points[comb])
    print(rob_points, cam_points)
    rotation_matrix, translation_vector = estimate_rigid_transform(cam_points, rob_points) # Do an estimation from both the 3d robot and camera coordinates to get rotation matrix and translation vector
    homogeneous = build_homogeneous(rotation_matrix,translation_vector) # Convert the rotation and translation into a 4x4 homogeneous matrix that can be used to convert camera coordinates into robotframe

    # Create DepthAI pipeline
    pipeline = dai.Pipeline()

    # Define the camera nodes
    cam_rgb   = pipeline.create(dai.node.ColorCamera)
    mono_left  = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo     = pipeline.create(dai.node.StereoDepth)
    detection_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

    # Define XLink outputs for streaming frames and detections to host
    xout_rgb   = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_nn    = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")
    xout_nn.setStreamName("detections")

    # Camera configuration (RGB camera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
    cam_rgb.setPreviewSize(640, 640) # Neural network input size. (MUST MATCH THE RUNNING MODELS SIZE)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # Mono cameras (for depth)
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    # Stereo depth configuration
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)       
    stereo.setOutputSize(mono_left.getResolutionWidth(), mono_left.getResolutionHeight())
    # 400P + extended covers 28 cm. 720P/800P + extended stops near 35 cm.
    # DepthAI 2.30 cannot enable subpixel together with extended disparity.
    stereo.setSubpixel(False)
    stereo.setExtendedDisparity(True)
    print("[DEPTH] mono=400P  ExtendedDisparity=True  Subpixel=False") 

    configPath = "blob_v5/best_Yolo5_small.json" # Path to the config JSON file for the model (MUST BE CHANGED IF YOU WANT TO USE A NEW MODEL)
    with open(configPath, "r") as f:
        config = json.load(f)
    nnConfig = config.get("nn_config", {})

    # Extract the metadata from the JSON file
    metadata = nnConfig.get("NN_specific_metadata", {})
    classes = metadata.get("classes", {})
    coordinates = metadata.get("coordinates", {})
    anchors = metadata.get("anchors", {})
    anchorMasks = metadata.get("anchor_masks", {})
    iouThreshold = metadata.get("iou_threshold", {})
    confidenceThreshold = metadata.get("confidence_threshold", {})

    print(metadata)

    nnMappings = config.get("mappings", {})
    labels = nnMappings.get("labels", {})

    nnPath = "blob_v5/best_Yolo5_small_openvino_2022.1_5shave.blob" # PATH TO THE .BLOB FILE (MUST BE CHANGED IF YOU WANT TO USE A NEW MODEL)

    # Specific settings for the network
    detection_nn.setConfidenceThreshold(confidenceThreshold)
    detection_nn.setNumClasses(classes)
    detection_nn.setCoordinateSize(coordinates)
    detection_nn.setAnchors(anchors)
    detection_nn.setAnchorMasks(anchorMasks)
    detection_nn.setIouThreshold(iouThreshold)
    detection_nn.setBlobPath(nnPath)
    detection_nn.setNumInferenceThreads(2)
    detection_nn.input.setBlocking(False)

    syncNN = True

    # Linking
    cam_rgb.preview.link(detection_nn.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(detection_nn.inputDepth)

    detection_nn.passthrough.link(xout_rgb.input) # Passthrough RGB frames (frames that went into NN)
    detection_nn.passthroughDepth.link(xout_depth.input) # Aligned depth frames
    detection_nn.out.link(xout_nn.input) # Detection outputs (bounding boxes + coordinates)

    return homogeneous, syncNN, pipeline, labels, rotation_matrix, translation_vector, camera_points, robot_points