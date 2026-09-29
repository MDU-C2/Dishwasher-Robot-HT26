# Learning objective: run YOLOv5 detection independently on OAK, without connecting to YuMi.
# Data flow: camera -> YOLO spatial detection node -> USB queues -> Python arrays/detections -> display.
# Center depth measures the central image ROI; YOLO XYZ describes each detected cup.
# Mouse probe: hover or click a pixel -> 5x5 valid-depth median -> camera-frame XYZ. Not a robot target.
# Limitation: RGB and detections are matched by sequence number; displayed depth is not strictly synchronized.
# This configuration uses 1080p on IMX378. Display updates depend on spatial detection throughput, not just setFps(30).
# Keep DepthAI 2.30.0.0. 
# Reading order: model configuration -> camera input -> detection node -> queue loop -> detection results.

import cv2  # Drawing, window display, keyboard handling, and image saving.
import numpy as np  # Depth filtering, median calculation, and display conversions.
import depthai as dai  # Build the OAK device pipeline and exchange data with the device.

import json
import csv
import math
from datetime import datetime
from pathlib import Path
from robot_coordinate_preview import load_matrix, target_rows

# Load once before opening the camera. Missing/invalid calibration fails visibly.
# Approximate marker-to-tool fit, NOT verified camera extrinsics for grasping.
robot_preview_matrix = load_matrix()

def project_yolo_xyz(coords, fx, fy, cx, cy, rgb_size, distortion):
    """Project returned camera XYZ to RGB pixels; return None if unusable/offscreen.

    Project into the original distorted RGB preview using device calibration.
    This visualizes a regional depth estimate, not a measured surface pixel or
    a grasp point. Intrinsics must match the stretched RGB view. XYZ is unchanged.
    """
    height, width = rgb_size
    if not all(math.isfinite(value) for value in
               (coords.x, coords.y, coords.z, fx, fy, cx, cy)):
        return None
    if coords.z <= 0 or fx <= 0 or fy <= 0:
        return None
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if distortion.size != 14 or not np.isfinite(distortion).all():
        return None
    camera_matrix = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64
    )
    # XYZ is already in the camera frame: no rotation/translation is applied.
    try:
        projected, _ = cv2.projectPoints(
            np.array([[coords.x, coords.y, coords.z]], dtype=np.float64),
            np.zeros(3), np.zeros(3), camera_matrix, distortion,
        )
    except cv2.error:
        return None
    u, v = projected.reshape(2)
    if not (math.isfinite(u) and math.isfinite(v)):
        return None
    if not (0 <= u < width and 0 <= v < height):
        return None
    pixel = (int(round(u)), int(round(v)))
    if not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
        return None
    return pixel


def select_target(detections,labels):
    # Select the valid upright target with the smallest camera X (leftmost).
    # This prioritizes left-side access; it is not a collision check.
    selected_target = None

    for det in detections:
        # The class ID must be a valid index into labels.
        if not 0 <= det.label < len(labels):
            continue

        # Phase 1 selects only the upright class.
        if labels[det.label] != "upright":
            continue

        coords = det.spatialCoordinates

        # Reject non-finite coordinates and invalid depth.
        if not np.isfinite([coords.x, coords.y, coords.z]).all():
            continue
        if coords.z <= 0:
            continue


        # Keep the first eligible target, then replace it if a smaller X is found.
        # Negative X is valid; equal X keeps the first eligible detection.
        if selected_target is None:
            selected_target = det
        elif coords.x < selected_target.spatialCoordinates.x:
            selected_target = det

    return selected_target


# 5x5 neighborhood around the probed pixel; half_size=2 -> 5 rows and 5 columns.
PROBE_HALF_SIZE = 2
# Reject the probe when too few pixels in the neighborhood have valid depth.
PROBE_MIN_VALID = 5


def on_rgb_mouse(event, x, y, flags, param):
    # OpenCV reports window coordinates in pixels of the displayed RGB frame.
    param["u"] = x
    param["v"] = y
    if event == cv2.EVENT_LBUTTONDOWN:
        param["save_click"] = True


def probe_pixel_xyz(
    depth_frame,
    pixel_u,
    pixel_v,
    fx,
    fy,
    cx,
    cy,
    rgb_size,
    half_size=PROBE_HALF_SIZE,
):
    # Build one camera-frame point from an RGB pixel and the aligned depth map.
    # Inputs: depth in mm, pixel (u, v) in the RGB image, RGB-scaled intrinsics.
    # Output: a dict of camera XYZ in mm, or None when depth is unusable.
    rgb_height, rgb_width = rgb_size
    depth_height, depth_width = depth_frame.shape[:2]

    if rgb_width <= 1 or rgb_height <= 1:
        return None
    if not (0 <= pixel_u < rgb_width and 0 <= pixel_v < rgb_height):
        return None

    # Sample depth at the corresponding pixel when RGB and depth resolutions differ.
    depth_x = int(round(pixel_u * (depth_width - 1) / (rgb_width - 1)))
    depth_y = int(round(pixel_v * (depth_height - 1) / (rgb_height - 1)))

    x0 = max(0, depth_x - half_size)
    x1 = min(depth_width, depth_x + half_size + 1)
    y0 = max(0, depth_y - half_size)
    y1 = min(depth_height, depth_y + half_size + 1)

    depth_roi = depth_frame[y0:y1, x0:x1]
    valid_depths = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0)]
    roi_count = int(depth_roi.size)
    valid_count = int(valid_depths.size)

    if valid_count < PROBE_MIN_VALID:
        return None

    z_mm = float(np.median(valid_depths))
    x_mm = (float(pixel_u) - cx) * z_mm / fx
    y_mm = (float(pixel_v) - cy) * z_mm / fy

    if not np.isfinite([x_mm, y_mm, z_mm]).all() or z_mm <= 0:
        return None

    return {
        "pixel_u": int(pixel_u),
        "pixel_v": int(pixel_v),
        "camera_x_mm": x_mm,
        "camera_y_mm": y_mm,
        "camera_z_mm": z_mm,
        "valid_depth_count": valid_count,
        "roi_size": roi_count,
    }


# Resolve model files relative to the script directory.
model_dir = Path(__file__).resolve().parent / "models"
model_path = model_dir / "best_Yolo5_small_openvino_2022.1_5shave.blob"
config_path = model_dir / "best_Yolo5_small.json"

# Report a missing model before attempting to start the camera.
if not model_path.is_file():
    raise FileNotFoundError(f"Model not found: {model_path}")

# Read the configuration supplied with the model.
with config_path.open("r", encoding="utf-8") as config_file:
    model_config = json.load(config_file)

# JSON contains configuration, not detections; .blob contains the compiled device model.
# Label order must match the model's output class IDs.
nn_config = model_config["nn_config"]
metadata = nn_config["NN_specific_metadata"]
labels = model_config["mappings"]["labels"]

print("Model:", model_path.name)
print("Input size:", nn_config["input_size"])
print("Classes:", metadata["classes"])
print("Labels:", labels)


# ============================================================
# Create the OAK pipeline.
# ============================================================

# This defines the pipeline; dai.Device(pipeline) deploys and starts it.
pipeline = dai.Pipeline()


# ============================================================
# Create the color camera.
# ============================================================

cam_rgb = pipeline.create(dai.node.ColorCamera)

# RGB identifies the central color camera.
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

# The IMX378 configuration uses 1080p rather than 720p.
cam_rgb.setResolution(
    dai.ColorCameraProperties.SensorResolution.THE_1080_P
)

# The preview sent to the host is 640 x 640.
cam_rgb.setPreviewSize(640, 640)

# False stretches the video field of view to 640 x 640; objects may appear distorted.
# This is a test configuration, not a YOLO requirement; it differs from MARC's default cropping.
# Cropping/stretching affects detection and coordinate mapping; keep RGB and depth geometry consistent.
cam_rgb.setPreviewKeepAspectRatio(False)
# False selects planar channel storage on the device for the network input.
# This controls memory layout, not RGB/BGR order; getCvFrame() produces an OpenCV array.
cam_rgb.setInterleaved(False)
# Use blue-green-red channel order, matching MARC's model input configuration.
# Channel order is unrelated to image flipping; the variable name rgb_frame does not imply RGB order.
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
# Requested capture rate; inference and display are not guaranteed to reach 30 FPS.
cam_rgb.setFps(30)


# ============================================================
# Create the left and right monochrome cameras.
# ============================================================
# OAK estimates depth from differences between the left and right views.

cam_left = pipeline.create(dai.node.MonoCamera)
cam_right = pipeline.create(dai.node.MonoCamera)

# Both monochrome cameras use 400p.
cam_left.setResolution(
    dai.MonoCameraProperties.SensorResolution.THE_400_P
)
cam_right.setResolution(
    dai.MonoCameraProperties.SensorResolution.THE_400_P
)

# Assign the left and right camera sockets.
cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

cam_left.setFps(30)
cam_right.setFps(30)


# ============================================================
# Create the stereo depth node.
# ============================================================

stereo = pipeline.create(dai.node.StereoDepth)

# Connect the monochrome cameras to the depth node.
cam_left.out.link(stereo.left)
cam_right.out.link(stereo.right)

# HIGH_DENSITY favors retaining more depth pixels.
stereo.setDefaultProfilePreset(
    dai.node.StereoDepth.PresetMode.HIGH_DENSITY
)

# Check consistency between left and right depth estimates.
# This helps reduce incorrect depth estimates.
stereo.setLeftRightCheck(True)

# Extended disparity is disabled in this test configuration; its effect on performance requires measurement.
# Recheck near-range depth validity at the intended working distances.
stereo.setExtendedDisparity(False)

# Align depth to the color camera viewpoint.
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

# Set the depth output dimensions; these are separate from the monochrome capture resolution.
stereo.setOutputSize(640, 640)
# Do not preserve the depth output aspect ratio, to match the stretched RGB geometry.
# Matching dimensions alone does not prove alignment or synchronization; verify with physical objects.
stereo.setOutputKeepAspectRatio(False)

# ============================================================
# Create and configure the YOLO spatial detection network.
# ============================================================

# The device detects objects and combines depth with detections to estimate camera-frame XYZ.
# Inference runs on OAK, not in host-side PyTorch; this script does not train the model.
detection_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

# Select the compiled model; str() converts the Path to the required string.
detection_nn.setBlobPath(str(model_path))
# Reject detections below the configured threshold (currently 0.5); confidence is not grasp success probability.
detection_nn.setConfidenceThreshold(metadata["confidence_threshold"])
# Number of classes (8); must match the model.
detection_nn.setNumClasses(metadata["classes"])
# Four bounding-box coordinate components; this is not the dimension of spatial XYZ.
detection_nn.setCoordinateSize(metadata["coordinates"])
# Preset width/height pairs for YOLO decoding, not robot calibration coordinates.
detection_nn.setAnchors(metadata["anchors"])
# Assign anchor indices to output layers, using the model's matching configuration.
detection_nn.setAnchorMasks(metadata["anchor_masks"])
# IoU threshold suppresses duplicate overlapping detections; it is not a depth accuracy threshold.
detection_nn.setIouThreshold(metadata["iou_threshold"])
# Device inference threads, not host CPU threads; two threads do not guarantee twice the speed.
detection_nn.setNumInferenceThreads(2)
# Allow old network inputs to be dropped when full; host get() would still wait for data.
detection_nn.input.setBlocking(False)

# Calculate spatial coordinates using a smaller region inside each detection box.
# Shrink the depth sampling ROI to reduce boundary/background contamination; the wrong surface can still be sampled.
detection_nn.setBoundingBoxScaleFactor(0.5)
# Minimum accepted spatial depth in mm; not the sensor's guaranteed minimum measurement range.
detection_nn.setDepthLowerThreshold(100)
# Maximum spatial depth; these thresholds do not change raw depth or central ROI statistics.
detection_nn.setDepthUpperThreshold(5000)

# RGB feeds detection; depth feeds spatial coordinate estimation.
cam_rgb.preview.link(detection_nn.input)
stereo.depth.link(detection_nn.inputDepth)

# ============================================================
# Send the network's input images and detection results to the host.
# ============================================================

rgb_output = pipeline.create(dai.node.XLinkOut)
depth_output = pipeline.create(dai.node.XLinkOut)
det_output = pipeline.create(dai.node.XLinkOut)

rgb_output.setStreamName("rgb")
depth_output.setStreamName("depth")
det_output.setStreamName("detections")

# Send RGB frames used by the network; this is not an independent high-rate preview.
detection_nn.passthrough.link(rgb_output.input)
# Send depth passing through the node; host-side temporal correspondence still requires attention.
detection_nn.passthroughDepth.link(depth_output.input)
# Send detection objects, not images with boxes already drawn.
detection_nn.out.link(det_output.input)

# ============================================================
# Start the camera.
# ============================================================

# Open OAK and start the pipeline; leaving the with block releases the device.
with dai.Device(pipeline) as device:

    # Print the negotiated USB speed.
    print("USB speed:", device.getUsbSpeed())

    # Three host queues: RGB, depth, and detections; names match the stream names above.
    # maxSize=4 buffers up to four packets; blocking=False allows old packets to be dropped when full.
    # get() would still wait; tryGet() below keeps the GUI responsive.
    rgb_queue = device.getOutputQueue(
        name="rgb",
        maxSize=4,
        blocking=False
    )

    depth_queue = device.getOutputQueue(
        name="depth",
        maxSize=4,
        blocking=False
    )

    det_queue = device.getOutputQueue(
        name="detections",
        maxSize=4,
        blocking=False
    )


    print("Camera started.")
    print("Move the mouse in OAK RGB to probe camera XYZ.")
    print("Left click saves one probe point. Press r to save the YOLO cup. Press q to quit.")

    # Scale RGB intrinsics to the 640x640 stretched preview used by this pipeline.
    calibration = device.readCalibration()
    # The RGB preview is not host-undistorted; draw XYZ with lens distortion.
    # Distortion coefficients do not scale when resizing the image; intrinsics do.
    rgb_distortion = np.asarray(
        calibration.getDistortionCoefficients(dai.CameraBoardSocket.RGB),
        dtype=np.float64,
    ).reshape(-1)
    if rgb_distortion.size != 14 or not np.isfinite(rgb_distortion).all():
        raise ValueError("Expected 14 finite RGB distortion coefficients")
    print("XYZ dot projection: device RGB distortion enabled (14 coefficients).")
    rgb_intrinsics = np.array(
        calibration.getCameraIntrinsics(
            dai.CameraBoardSocket.RGB,
            640,
            640,
            keepAspectRatio=False,
        ),
        dtype=np.float64,
    )
    fx = float(rgb_intrinsics[0, 0])
    fy = float(rgb_intrinsics[1, 1])
    cx = float(rgb_intrinsics[0, 2])
    cy = float(rgb_intrinsics[1, 2])
    print(
        f"RGB intrinsics at 640x640: fx={fx:.1f}, fy={fy:.1f}, "
        f"cx={cx:.1f}, cy={cy:.1f}"
    )

    mouse_state = {"u": None, "v": None, "save_click": False}
    cv2.namedWindow("OAK RGB")
    cv2.namedWindow("OAK Robot Coordinates", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("OAK Robot Coordinates", 850, 480)
    cv2.setMouseCallback("OAK RGB", on_rgb_mouse, mouse_state)

    # None means no packet is held; retain each stream separately while waiting for the others.
    rgb_packet = None
    depth_packet = None
    det_packet = None
    save_requested = False  # Remember the r request until a complete frame is ready for paired saving.
    displayed_target = None
    displayed_probe = None

    # Save beside the script, independent of the PowerShell working directory.
    record_path = Path(__file__).resolve().parent / "target_records_paired.csv"
    probe_record_path = Path(__file__).resolve().parent / "probe_records.csv"
    # Receive frames until q is pressed.
    while True:
        # Process GUI events and exit keys even when no new data is available.
        key = cv2.waitKey(10) & 0xFF

        if key == ord("q"):
            break
        
        if key == ord("r"):
            save_requested = True
                
        # Return None immediately if no packet is available.
        if rgb_packet is None:
            rgb_packet = rgb_queue.tryGet()

        if depth_packet is None:
            depth_packet = depth_queue.tryGet()

        if det_packet is None:
            det_packet = det_queue.tryGet()

        # If any stream is missing, return to the loop start to process keyboard events.
        if (
            rgb_packet is None
            or depth_packet is None
            or det_packet is None
        ):
            continue

        # Sequence numbers identify frames, not object classes.
        # Independent queues may drop packets; discard the older side to avoid mismatching RGB and detections.
        # Depth timestamps are not compared here; displayed depth and RGB are not strictly synchronized.
        rgb_seq = rgb_packet.getSequenceNum()
        det_seq = det_packet.getSequenceNum()

        if rgb_seq < det_seq:
            rgb_packet = None
            continue

        if det_seq < rgb_seq:
            det_packet = None
            continue

        # Extract the BGR image, depth array in mm, and list of detections.
        rgb_frame = rgb_packet.getCvFrame()
        depth_frame = depth_packet.getFrame()
        detections = det_packet.detections

        # This group has been extracted; receive new packets on the next iteration.
        rgb_packet = None
        depth_packet = None
        det_packet = None

        rgb_height, rgb_width = rgb_frame.shape[:2]
        # Shared pinhole for the mouse probe and returned YOLO XYZ projection.
        scale_fx = fx * rgb_width / 640.0
        scale_fy = fy * rgb_height / 640.0
        scale_cx = cx * rgb_width / 640.0
        scale_cy = cy * rgb_height / 640.0
        depth_height, depth_width = depth_frame.shape
        rgb_center_x = rgb_width // 2
        rgb_center_y = rgb_height // 2
        depth_center_x = depth_width // 2
        depth_center_y = depth_height // 2
        radius = 10

        # The central ROI is a separate learning measurement, not the YOLO target or a grasp point.
        # Arrays use [row y, column x]; the exclusive slice endpoint requires +1 for 21 x 21 = 441 pixels.
        # Depth values in this pipeline are expressed in millimeters.
        depth_roi = depth_frame[
            depth_center_y - radius:depth_center_y + radius + 1,
            depth_center_x - radius:depth_center_x + radius + 1
        ]

        # Exclude invalid zero-depth pixels.
        valid_depths = depth_roi[depth_roi > 0]

        # Use the median of valid values, or zero if none are available.
        if valid_depths.size > 0:
            center_depth_mm = int(np.median(valid_depths))
        else:
            center_depth_mm = 0

        # ----------------------------------------------------
        # Convert raw depth to a color visualization.
        # Only transform a display copy; keep depth_frame unchanged for numerical measurements.
        # ----------------------------------------------------

        # Clip the display range to 0-5000 mm.
        # Distances beyond five meters are outside the chosen visualization range.
        depth_for_display = np.clip(depth_frame, 0, 5000)

        # Map 0-5000 mm to 0-255 grayscale values.
        depth_for_display = (
            depth_for_display / 5000.0 * 255
        ).astype(np.uint8)

        # Invert the grayscale values:
        # Nearer depths receive higher values; farther depths receive lower values.
        depth_for_display = 255 - depth_for_display

        # Apply a color map to the depth visualization.
        depth_colormap = cv2.applyColorMap(
            depth_for_display,
            cv2.COLORMAP_JET
        )

        # Zero depth indicates an invalid measurement at that pixel.
        # Display invalid pixels as black.
        depth_colormap[depth_frame == 0] = 0

        cv2.drawMarker(
            rgb_frame,
            (rgb_center_x, rgb_center_y),
            (0, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2
        )

        cv2.drawMarker(
            depth_colormap,
            (depth_center_x, depth_center_y),
            (255, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2
        )

        cv2.rectangle(
            rgb_frame,
            (rgb_center_x - radius, rgb_center_y - radius),
            (rgb_center_x + radius, rgb_center_y + radius),
            (255, 255, 255),
            1
        )

        cv2.rectangle(
            depth_colormap,
            (depth_center_x - radius, depth_center_y - radius),
            (depth_center_x + radius, depth_center_y + radius),
            (255, 255, 255),
            1
        )

        if center_depth_mm == 0:
            # Zero indicates no valid depth in the central ROI.
            depth_text = "Center depth: invalid"
        else:
            depth_text = f"Center depth: {center_depth_mm} mm"

        depth_text += f" | Valid: {valid_depths.size}/{depth_roi.size}"

        cv2.putText(
            rgb_frame,
            depth_text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        displayed_probe = None
        probe_u = mouse_state["u"]
        probe_v = mouse_state["v"]

        if probe_u is not None and probe_v is not None:
            displayed_probe = probe_pixel_xyz(
                depth_frame,
                probe_u,
                probe_v,
                scale_fx,
                scale_fy,
                scale_cx,
                scale_cy,
                (rgb_height, rgb_width),
            )

            cv2.drawMarker(
                rgb_frame,
                (probe_u, probe_v),
                (255, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=20,
                thickness=2,
            )
            cv2.rectangle(
                rgb_frame,
                (probe_u - PROBE_HALF_SIZE, probe_v - PROBE_HALF_SIZE),
                (probe_u + PROBE_HALF_SIZE, probe_v + PROBE_HALF_SIZE),
                (255, 255, 0),
                1,
            )

            if displayed_probe is None:
                probe_text = (
                    f"Probe: invalid depth at ({probe_u}, {probe_v})"
                )
                probe_color = (0, 0, 255)
            else:
                probe_text = (
                    f"Probe XYZ: {displayed_probe['camera_x_mm']:.0f}, "
                    f"{displayed_probe['camera_y_mm']:.0f}, "
                    f"{displayed_probe['camera_z_mm']:.0f} mm "
                    f"| ({probe_u}, {probe_v})"
                )
                probe_color = (255, 255, 0)

            cv2.putText(
                rgb_frame,
                probe_text,
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                probe_color,
                2,
            )

        selected_target = select_target(detections, labels)
        robot_rows = target_rows(detections, labels, robot_preview_matrix)

        # An empty detection list skips this loop; still update the display and handle keys.
        for detection_number, det in enumerate(detections, start=1):
            x1 = max(0, min(rgb_width - 1, int(det.xmin * rgb_width)))
            y1 = max(0, min(rgb_height - 1, int(det.ymin * rgb_height)))
            x2 = max(0, min(rgb_width - 1, int(det.xmax * rgb_width)))
            y2 = max(0, min(rgb_height - 1, int(det.ymax * rgb_height)))

            # Map class IDs to names; preserve unknown IDs for diagnosis.
            label = labels[det.label] if 0 <= det.label < len(labels) else str(det.label)
            # OAK target coordinates in the camera frame, in mm; not YuMi/WorkObject coordinates.
            # Preserve these raw coordinates for the camera XYZ dot and HT input.
            coords = det.spatialCoordinates

            # Draw the selected target in red and other detections in green.
            if det is selected_target:
                box_color = (0, 0, 255)
            else:
                box_color = (0, 255, 0)

            cv2.rectangle(
                rgb_frame, (x1, y1), (x2, y2), box_color, 2
            )

            label_text = f"#{detection_number} {label} {det.confidence:.0%}"

            # Positive Z is only a basic validity check; it does not prove accuracy or a cup-rim location.
            if np.isfinite([coords.x, coords.y, coords.z]).all() and coords.z > 0:
                position_text = (
                    f"#{detection_number} XYZ: {coords.x:.0f}, "
                    f"{coords.y:.0f}, {coords.z:.0f} mm"
                )
                xyz_pixel = project_yolo_xyz(
                    coords, scale_fx, scale_fy, scale_cx, scale_cy,
                    (rgb_height, rgb_width), rgb_distortion,
                )
                if xyz_pixel is not None:
                    # Match the detection box; a black outline keeps the dot visible.
                    cv2.circle(rgb_frame, xyz_pixel, 6, (0, 0, 0), -1)
                    cv2.circle(rgb_frame, xyz_pixel, 4, box_color, -1)
                    cv2.putText(
                        rgb_frame, f"XYZ #{detection_number}",
                        (max(0, min(rgb_width - 100, xyz_pixel[0] + 12)),
                         max(15, xyz_pixel[1] - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1,
                    )
                else:
                    position_text += " | projection outside/invalid"
            else:
                position_text = f"#{detection_number} XYZ: depth unavailable"

            cv2.putText(
                rgb_frame, label_text,
                (x1, max(15, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 1
            )

            cv2.putText(
                rgb_frame, position_text,
                (x1, min(rgb_height - 5, y1 + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 1
            )

            robot_xyz = robot_rows[detection_number - 1]['robot_xyz_estimate_mm']
            robot_text = (f"#{detection_number} Robot est: " +
                          ', '.join(f'{value:.1f}' for value in robot_xyz) + ' mm'
                          if robot_xyz is not None else
                          f"#{detection_number} Robot est: invalid depth")
            cv2.putText(rgb_frame, robot_text,
                        (x1, min(rgb_height - 5, y1 + 37)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

        # A separate list keeps all target coordinates readable when boxes overlap.
        # Recreate the list each frame so missing/invalid targets cannot retain XYZ.
        panel = np.zeros((max(160, 100 + 64 * len(robot_rows)), 850, 3), dtype=np.uint8)
        panel_lines = [
            ('All targets | XYZ in mm | IDs are per frame', (255, 255, 255)),
            ('Robot ESTIMATE: 16-point approximate fit | no robot connection', (0, 200, 255)),
            ('Marker/tool offset and YOLO camera-frame agreement unverified', (0, 200, 255)),
        ]
        for line_number, (line, color) in enumerate(panel_lines):
            cv2.putText(panel, line, (12, 22 + line_number * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if not robot_rows:
            cv2.putText(panel, 'No targets', (12, 115),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        for index, row in enumerate(robot_rows):
            camera_xyz = row['camera_xyz_mm']
            robot_xyz = row['robot_xyz_estimate_mm']
            camera_text = (', '.join(f'{v:.1f}' for v in camera_xyz)
                           if camera_xyz is not None else 'invalid depth')
            robot_text = (', '.join(f'{v:.1f}' for v in robot_xyz)
                          if robot_xyz is not None else 'unavailable')
            for offset, line, color in (
                (0, f"#{row['number']} {row['label']} | Camera: {camera_text}", (220, 220, 220)),
                (24, f"    Robot est: {robot_text}", (255, 0, 255)),
            ):
                cv2.putText(panel, line, (12, 104 + index * 64 + offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        cv2.imshow('OAK Robot Coordinates', panel)
        
        displayed_target = None
        
        if selected_target is not None:
            coords = selected_target.spatialCoordinates
            
            displayed_target = {
                
                
                "frame_id":rgb_seq,
                "label": labels[selected_target.label],
                "confidence": float(selected_target.confidence),
                "camera_x_mm": float(coords.x),
                "camera_y_mm": float(coords.y),
                "camera_z_mm": float(coords.z),
                
            }
        cv2.imshow("OAK RGB", rgb_frame)
        #cv2.imshow("OAK Depth", depth_colormap)

        # Left click stores the mouse probe, not the YOLO cup and not a robot command.
        if mouse_state["save_click"]:
            mouse_state["save_click"] = False

            if displayed_probe is None:
                print("Not saved: probe depth invalid or no pixel selected.")
            else:
                probe_record = displayed_probe.copy()
                probe_record["frame_id"] = rgb_seq
                probe_record["saved_at"] = datetime.now().isoformat(
                    timespec="milliseconds"
                )

                image_dir = Path(__file__).resolve().parent / "images"
                image_dir.mkdir(exist_ok=True)

                image_number = 1
                image_path = image_dir / f"probe_{image_number:04d}.jpg"
                while image_path.exists():
                    image_number += 1
                    image_path = image_dir / f"probe_{image_number:04d}.jpg"

                probe_record["image_file"] = image_path.name

                try:
                    needs_header = (
                        not probe_record_path.exists()
                        or probe_record_path.stat().st_size == 0
                    )

                    with probe_record_path.open(
                        "a", newline="", encoding="utf-8"
                    ) as file:
                        writer = csv.DictWriter(
                            file, fieldnames=list(probe_record.keys())
                        )

                        saved = cv2.imwrite(str(image_path), rgb_frame)

                        if saved:
                            if needs_header:
                                writer.writeheader()

                            writer.writerow(probe_record)
                            print("Saved probe:", probe_record)
                            print("Image path:", image_path)
                        else:
                            print("Not saved: probe image write failed.")

                except PermissionError:
                    print(
                        "Save failed: permission denied. "
                        "Close the CSV in Excel and try again."
                    )

        # Handle the r request after selection and drawing to save paired image and target data.
        # Use the script directory and numbered images; append target records to CSV.
        if save_requested:
            save_requested = False

            if displayed_target is None:
                print("Not saved: no valid selected target.")
            else:
                record = displayed_target.copy()
                record["saved_at"] = datetime.now().isoformat(
                    timespec="milliseconds"
                )

                image_dir = Path(__file__).resolve().parent / "images"
                image_dir.mkdir(exist_ok=True)

                image_number = 1
                image_path = image_dir / f"oak_{image_number:04d}.jpg"

                # This loop only finds an unused image filename.
                while image_path.exists():
                    image_number += 1
                    image_path = image_dir / f"oak_{image_number:04d}.jpg"

                # Outside the loop: save once after the filename has been selected.
                record["image_file"] = image_path.name

                try:
                    needs_header = (
                        not record_path.exists()
                        or record_path.stat().st_size == 0
                    )

                    with record_path.open(
                        "a", newline="", encoding="utf-8"
                    ) as file:
                        writer = csv.DictWriter(
                            file, fieldnames=list(record.keys())
                        )

                        saved = cv2.imwrite(str(image_path), rgb_frame)

                        if saved:
                            if needs_header:
                                writer.writeheader()

                            writer.writerow(record)
                            print("Saved target:", record)
                            print("Image path:", image_path)
                        else:
                            print("Not saved: image write failed.")

                except PermissionError:
                    print(
                        "Save failed: permission denied. "
                        "Close the CSV in Excel and try again."
                    )
            
            

# Close all OpenCV windows.
cv2.destroyAllWindows()
