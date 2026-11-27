"""Unified entry (1125): 5-model 3D cup detection + 2D YOLO/ArUco tray detection.

Features:
  - 5 core 3D YOLO models selectable via 1-5 (Space/Enter defaults to 4: yolo11n_416)
  - Unified stretched 1080p RGB/depth/projection geometry driven by camera_setup.py
    so the red spatial dot stays inside the bounding box for all 5 models
  - Non-cup classes ('handle', 'Gripper') excluded from grasp target selection
  - Fixed HT_New_Run2_Mean with X and Z compensation preserved unchanged
"""

import cv2 as cv
import numpy as np
import depthai as dai
import time
from updated_communication import Communication
import camera_setup as cs
import test_protocol as tp
import tray_camera_yolo as tc
import threading

try:
    import yumi_capture
except ModuleNotFoundError as exc:
    if exc.name != "yumi_capture":
        raise
    yumi_capture = None

global busy, times_sec, count
busy = False
count = 0
times_sec = []

lock = threading.RLock()

# Classes emitted by 8-class (MARC) and 7-class (FT-MARC) models that must never
# be selected as cup grasp targets.
NON_CUP_LABELS = frozenset({"handle", "Gripper"})


def build_homogeneous(rotation_matrix, translation_vector):
    T_camera_to_base_effector = np.eye(4)
    T_camera_to_base_effector[:3, :3] = rotation_matrix
    T_camera_to_base_effector[:3, 3] = translation_vector.reshape(3)
    return T_camera_to_base_effector


def x_compensate(x):
    y = 1.4018216887159423 * x + -198.52114324095436
    return y


def z_compensate(z):
    """HT Z in, compensated Z out. 35 pairs, 2026-10-23, HT_New_Run2_Mean."""
    y = -0.0245556184637862 * z + 53.3973105975510762
    return y


def robot_xyz_from_relations(x, y, z, homogeneous):
    point = np.array([x, y, z, 1.0], dtype=np.float64)
    ht_xyz = homogeneous.dot(point)[:3]
    robot_x = x_compensate(ht_xyz[0])
    robot_z = z_compensate(ht_xyz[2])
    return np.around([robot_x, ht_xyz[1], robot_z], 2).tolist()


SLOT_SOURCE = "camera"
FIXED_SLOT_NAME = "slot_1"
FIXED_SLOT_COORDS = [302.0, -508, 248.0]

_cur = None


def setup():
    saved = bool(tc.boxes())
    if not saved:
        print("[TRAY] No saved calibration found.")
    if not saved or input("\nCalibrate slots? (y/n): ").strip().lower() == "y":
        if not tc.calibrate() and not saved:
            return False
    tc.report()

    # ArUco tracking check
    tracker = tc.get_tracker()
    if tracker.ref_corners is None:
        print("\n[ARUCO] No reference ArUco pose found for tray tracking.")
        if input("Do you want to calibrate the ArUco tray reference now? (y/n): ").strip().lower() == "y":
            from calibrate_aruco import calibrate_aruco_reference
            calibrate_aruco_reference()

    if not tc.start():
        print("[TRAY] Cannot read tray camera or YOLO model.")
        return False
    return True


def select_placement_slot():
    global _cur
    match SLOT_SOURCE:
        case "fixed":
            coords = [float(v) for v in FIXED_SLOT_COORDS]
            print(f"[TRAY] Source=fixed  slot '{FIXED_SLOT_NAME}' @ {coords}")
            return FIXED_SLOT_NAME, coords
        case "camera":
            _cur = _cur or tc.take()
            if _cur is None:
                return None, None
            print(f"[TRAY] Source=camera (YOLO)  slot '{_cur[0]}' @ {_cur[1]}")
            return _cur[0], list(_cur[1])
        case _:
            print(f"[TRAY] ERROR: unknown SLOT_SOURCE '{SLOT_SOURCE}'.")
            return None, None


def end_slot(name, check):
    global _cur
    _cur = None
    if SLOT_SOURCE != "camera" or name is None:
        return
    if check:
        tc.done(name)
    else:
        tc.free(name)


LEFT_ENTRY_CAMERA_XYZ_MM = np.array(
    [-302.0, -147.0899, 407.3541],
    dtype=np.float64,
)


MAX_PICKUP_X_MM = 600.0
MIN_PICKUP_Y_MM = -50.0
PICKUP_RANGE_TOLERANCE_MM = 15.0


def is_within_pickup_work_range(robot_xyz):
    """Accept compensated robot X/Y within 15 mm past the nominal workspace.

    Nominal limits remain X <= 600 and Y >= -50. With tolerance, the inclusive
    limits are X <= 615 and Y >= -65.
    """
    if robot_xyz is None or len(robot_xyz) < 2:
        return False
    x, y = float(robot_xyz[0]), float(robot_xyz[1])
    return (
        np.isfinite([x, y]).all()
        and x <= MAX_PICKUP_X_MM + PICKUP_RANGE_TOLERANCE_MM
        and y >= MIN_PICKUP_Y_MM - PICKUP_RANGE_TOLERANCE_MM
    )


TARGET_HOLD_RADIUS_MM = 40.0
TARGET_SWITCH_MARGIN_MM = 30.0
TARGET_MISS_LIMIT = 3
GRASP_FRAME_COUNT = 5
SAME_CUP_DISTANCE_MM = 25.0


def _entry_reference():
    if LEFT_ENTRY_CAMERA_XYZ_MM is None:
        return None
    reference = np.asarray(LEFT_ENTRY_CAMERA_XYZ_MM, dtype=np.float64).reshape(3)
    if not np.isfinite(reference).all():
        return None
    return reference


def _eligible_targets(detections, labels, homogeneous, reference):
    eligible = []
    for det in detections:
        if not 0 <= det.label < len(labels):
            continue
        if labels[det.label] in NON_CUP_LABELS:
            continue
        coords = det.spatialCoordinates
        if not np.isfinite([coords.x, coords.y, coords.z]).all() or coords.z <= 0:
            continue
        robot_xyz = robot_xyz_from_relations(coords.x, coords.y, coords.z, homogeneous)
        if not is_within_pickup_work_range(robot_xyz):
            continue
        point = np.array([coords.x, coords.y, coords.z], dtype=np.float64)
        distance = float(np.linalg.norm(point - reference))
        eligible.append((det, point, distance))
    return eligible


def select_nearest_target(detections, labels, homogeneous):
    reference = _entry_reference()
    if reference is None:
        return None
    eligible = _eligible_targets(detections, labels, homogeneous, reference)
    if not eligible:
        return None
    return min(eligible, key=lambda item: item[2])[0]


def track_selected_target(detections, labels, homogeneous, held_xyz=None, missed_frames=0):
    """Keep the red box on the current cup until another cup is 30 mm closer.

    A detection within 40 mm of the previous camera point is the same cup.
    If that ball is empty but a cup is still visible, move the anchor to the
    nearest cup and keep collecting. A miss is counted only when no eligible
    cup is visible. Up to 3 such frames keep the remembered point and samples.
    """
    reference = _entry_reference()
    if reference is None:
        return None, None, 0, False
    eligible = _eligible_targets(detections, labels, homogeneous, reference)
    held = None if held_xyz is None else np.asarray(held_xyz, dtype=np.float64).reshape(3)
    if held is not None and not np.isfinite(held).all():
        held = None

    if held is not None:
        same = None
        same_gap = None
        for item in eligible:
            gap = float(np.linalg.norm(item[1] - held))
            if gap <= TARGET_HOLD_RADIUS_MM and (same is None or gap < same_gap):
                same = item
                same_gap = gap
        if same is not None:
            det, point, distance = same
            challenger = None
            for item in eligible:
                if item[0] is det:
                    continue
                if challenger is None or item[2] < challenger[2]:
                    challenger = item
            if challenger is not None and distance - challenger[2] >= TARGET_SWITCH_MARGIN_MM:
                lead = distance - challenger[2]
                print(f"[SELECT] Red box switched. Other cup is {lead:.1f} mm closer to the entry.")
                return challenger[0], challenger[1], 0, False
            return det, point, 0, True
        if not eligible:
            missed_frames += 1
            if missed_frames <= TARGET_MISS_LIMIT:
                print(
                    f"[SELECT] Held cup missing {missed_frames}/{TARGET_MISS_LIMIT}. "
                    "Keeping the current window."
                )
                return None, held, missed_frames, True
            print(
                f"[SELECT] Held cup missing for more than {TARGET_MISS_LIMIT} frames. "
                "Clearing the window."
            )
            return None, None, 0, False
        chosen = min(eligible, key=lambda item: item[2])
        gap = float(np.linalg.norm(chosen[1] - held))
        print(
            f"[SELECT] Held point is {gap:.1f} mm from the nearest cup. "
            "Moving the anchor and keeping the window."
        )
        return chosen[0], chosen[1], 0, True

    if not eligible:
        return None, None, 0, False
    chosen = min(eligible, key=lambda item: item[2])
    return chosen[0], chosen[1], 0, True


def camera_xyz_of(detection):
    coords = detection.spatialCoordinates
    return np.array([coords.x, coords.y, coords.z], dtype=np.float64)


def grasp_sample_matches(samples, label, camera_xyz):
    if not samples:
        return True
    if label != samples[0]["label"]:
        return False
    center = np.median(np.vstack([sample["xyz"] for sample in samples]), axis=0)
    return float(np.linalg.norm(camera_xyz - center)) <= SAME_CUP_DISTANCE_MM


def median_grasp_xyz(samples):
    return np.median(np.vstack([sample["xyz"] for sample in samples]), axis=0)


def update_grasp_samples(samples, selected_target, labels, keep_samples, accept_sample, busy):
    """Apply one frame to the 5-frame window. A short tracking gap keeps the samples."""
    if busy or (selected_target is None and not keep_samples):
        return []
    if selected_target is None or not accept_sample:
        return samples
    if not keep_samples:
        samples = []
    camera_xyz = camera_xyz_of(selected_target)
    sample_label = labels[selected_target.label]
    previous_count = len(samples)
    if samples and not grasp_sample_matches(samples, sample_label, camera_xyz):
        print("[FRAME] Target changed. 5-frame window restarted.")
        samples = []
        previous_count = 0
    samples = list(samples)
    samples.append({
        "xyz": camera_xyz,
        "label": sample_label,
        "conf": int(selected_target.confidence * 100),
    })
    if len(samples) > GRASP_FRAME_COUNT:
        samples = samples[-GRASP_FRAME_COUNT:]
    if len(samples) < GRASP_FRAME_COUNT:
        print(
            f"[FRAME] {len(samples)}/{GRASP_FRAME_COUNT} "
            f"{sample_label} camera XYZ mm: "
            f"{camera_xyz[0]:.1f}, {camera_xyz[1]:.1f}, {camera_xyz[2]:.1f}"
        )
    elif previous_count < GRASP_FRAME_COUNT:
        print("[FRAME] 5/5 ready. Automatic pick will send the median.")
    return samples


def load_preview_projection(device, width, height, model_name):
    """Intrinsics for the preview this model actually displays.

    Reads preview_mode directly from camera_setup.load_detection_model(model_name)
    so the [DOT] projection intrinsics can never drift out of sync with the
    ColorCamera and StereoDepth settings in camera_setup.py.
    """
    model_info = cs.load_detection_model(model_name)
    preview_mode = model_info["preview_mode"]
    canonical_name = model_info["name"]

    calibration = device.readCalibration()
    distortion = np.asarray(
        calibration.getDistortionCoefficients(dai.CameraBoardSocket.RGB),
        dtype=np.float64,
    ).reshape(-1)
    if distortion.size != 14 or not np.isfinite(distortion).all():
        raise ValueError("Expected 14 finite RGB distortion coefficients")

    if preview_mode == "stretched_1080p":
        matrix = np.asarray(
            calibration.getCameraIntrinsics(
                dai.CameraBoardSocket.RGB,
                int(width),
                int(height),
                keepAspectRatio=False,
            ),
            dtype=np.float64,
        )
        print(f"[DOT] {canonical_name} stretched 1080p preview {width}x{height}")
    elif preview_mode == "center_crop_720p":
        video_w, video_h = 1280, 720
        video = np.asarray(
            calibration.getCameraIntrinsics(
                dai.CameraBoardSocket.RGB,
                video_w,
                video_h,
                keepAspectRatio=True,
            ),
            dtype=np.float64,
        )
        if width <= 0 or height <= 0:
            raise ValueError("Preview size must be positive")
        video_aspect = video_w / video_h
        preview_aspect = width / height
        if video_aspect > preview_aspect:
            crop_w = video_h * preview_aspect
            crop_h = float(video_h)
            x0 = (video_w - crop_w) / 2.0
            y0 = 0.0
        else:
            crop_w = float(video_w)
            crop_h = video_w / preview_aspect
            x0 = 0.0
            y0 = (video_h - crop_h) / 2.0
        scale_x = width / crop_w
        scale_y = height / crop_h
        matrix = video.copy()
        matrix[0, 0] *= scale_x
        matrix[1, 1] *= scale_y
        matrix[0, 2] = (video[0, 2] - x0) * scale_x
        matrix[1, 2] = (video[1, 2] - y0) * scale_y
        print(
            f"[DOT] {canonical_name} 720p center crop {width}x{height} "
            f"x0={x0:.1f} y0={y0:.1f}"
        )
    else:
        raise ValueError(f"Unsupported preview_mode '{preview_mode}' for {canonical_name}")

    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("Preview intrinsics are not a finite 3x3 matrix")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("Preview intrinsics are not usable")
    print(
        f"[DOT] fx={matrix[0, 0]:.1f} fy={matrix[1, 1]:.1f} "
        f"cx={matrix[0, 2]:.1f} cy={matrix[1, 2]:.1f}"
    )
    return matrix, distortion


def project_spatial_pixel(coords, camera_matrix, distortion, image_size):
    """Draw spatial XYZ on the preview. Detection Y is up; image Y is down."""
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


def draw_spatial_dot(frame, pixel, selected):
    """Red dot: the selected camera point. Cyan dot: every other detection."""
    if selected:
        cv.circle(frame, pixel, 5, (255, 255, 255), -1)
        cv.circle(frame, pixel, 3, (0, 0, 255), -1)
    else:
        cv.circle(frame, pixel, 4, (0, 0, 0), -1)
        cv.circle(frame, pixel, 2, (255, 255, 0), -1)


def local_move(orient, client, target_xyz, normalized_vector, save_protocol, file_path, conf=None, label=None):
    global busy, times_sec, count

    with lock:
        if busy:
            return

        start_time = time.time()
        busy = True

        chosen_slot = None
        started = False
        finished = False

        try:
            # Recheck the final median-derived target before reserving a slot or moving.
            if not is_within_pickup_work_range(target_xyz):
                print(f"[RANGE] Pickup skipped: robot XYZ outside work range: {target_xyz}")
                return

            try:
                chosen_slot, coords = select_placement_slot()
                if chosen_slot is None or coords is None:
                    print("[TRAY] ERROR: No placement slot.")
                    return

                client.SetFreeSlot(coords)
                print(f"[TRAY] FreeSlot stored: {client.FreeSlot}")
            except Exception as exc:
                print(f"[TRAY] ERROR selecting free slot: {exc}")
                return

            client.MoveHome()
            print("[ROBOT] Starting PickUpSequence...")
            started = True

            client.PickUpSequence(target_xyz, orient, normalized_vector)
            print("[ROBOT] PickUpSequence completed.")

            finished = True
            end_slot(chosen_slot, True)

            end_time = time.time()
            process_time = end_time - start_time

            if save_protocol:
                count += 1
                tp.cup_information(
                    file_path,
                    count,
                    target_xyz,
                    process_time,
                    confidence=conf,
                    label=label,
                )
                times_sec.append(process_time)

        except Exception as exc:
            print(f"[ROBOT] ERROR in local_move: {exc}")

        finally:
            try:
                if not finished:
                    end_slot(chosen_slot, started)
            finally:
                with lock:
                    busy = False
                    client.FreeSlot = None


def choose_detection_model():
    """Prompt user for 1-5; Space+Enter or Enter selects default 4 (yolo11n_416)."""
    print("\nSelect 3D YOLO model:")
    for model_name in cs.MENU_ORDER:
        info = cs.DETECTION_MODELS[model_name]
        print(f"  {info['menu_id']}: {model_name:18s} - {info['description']}")
    while True:
        raw = input("Enter 1-5 (or Space/Enter for default 4 = yolo11n_416): ")
        model = cs.resolve_model_name(raw)
        if model is not None:
            menu_id = cs.DETECTION_MODELS[model]["menu_id"]
            print(f"[MODEL] Selected [{menu_id}] {model}.")
            return model
        print("Invalid selection. Enter 1-5, model name, or press Space/Enter for default (4).")


def run():
    global busy, times_sec
    model_choice = choose_detection_model()
    file_path = None
    normalized_vector = [0, 0, 1]
    orientation_map = {
        "Back": [0.0, 0.0, 1.0],
        "Front": [0.0, 0.0, -1.0],
        "left_side": [1.0, 0.0, 0.0],
        "right_side": [-1.0, 0.0, 0.0],
        "upright": [0.0, 1.0, 0.0],
        "upside_down": [0.0, -1.0, 0.0],
    }
    quaternion = [1, 0, 0, 0]
    client = Communication()

    def find_empty_slot_in_dishwasher():
        try:
            if client.FreeSlot is not None:
                print(f"[DYNAMIC TRAY] Returning pre-cached slot coordinates: {client.FreeSlot}")
                return list(client.FreeSlot)
            chosen_slot, coords = select_placement_slot()
            if coords is None:
                print("[DYNAMIC TRAY] WARNING: No placement slot.")
                return None
            return coords
        except Exception as exc:
            print(f"[DYNAMIC TRAY ERROR] {exc}")
            return None

    client.get_free_slot_logic = find_empty_slot_in_dishwasher

    if SLOT_SOURCE == "camera" and not setup():
        tc.close()
        return

    if client.connectV2():
        pipeline, labels = cs.camera_setup(model_choice)

        active_ht_name = "raw / HT_New_Run2_Mean"
        rotation_matrix = np.array([
            [-0.002597731108439694, -0.3876317471376878, 0.9218106532276962],
            [-0.9999744383587065, -0.005133658742591244, -0.004976763717912172],
            [0.00666141293402215, -0.9218000185283671, -0.3876085027689978],
        ], dtype=np.float64)
        translation_vector = np.array(
            [-84.04769438994447, 10.90679989710399, 319.0687334399627],
            dtype=np.float64,
        )
        homogeneous = build_homogeneous(rotation_matrix, translation_vector)
        print(f"[HT] Using fixed transform: {active_ht_name}; X and Z compensation enabled.")

        save_protocol = input("Do you want to save the test protocol? (y/n): ").lower() == "y"
        if save_protocol:
            file_path = tp.create_today_textfile()
            tp.ask_user(file_path, "start")
            tp.fill_meta_data(
                file_path, active_ht_name, rotation_matrix, translation_vector,
            )

        first_run = True
        grasp_samples = []
        held_xyz = None
        missed_frames = 0
        if LEFT_ENTRY_CAMERA_XYZ_MM is None:
            print(
                "[SELECT] LEFT_ENTRY_CAMERA_XYZ_MM is not set. "
                "No target will be chosen until that camera XYZ is filled in."
            )
        with dai.Device(pipeline) as device:
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            projection = None

            cv.namedWindow("RGB", cv.WINDOW_NORMAL)

            while True:
                if SLOT_SOURCE == "camera":
                    tray_img = tc.view()
                    if tray_img is not None:
                        cv.imshow("Tray Status (YOLO)", tray_img)

                in_rgb = q_rgb.get()
                in_depth = q_depth.get()
                in_dets = q_det.get()

                frame = in_rgb.getCvFrame()
                if projection is None:
                    projection = load_preview_projection(
                        device, frame.shape[1], frame.shape[0], model_choice,
                    )
                camera_matrix, distortion = projection
                depth_frame = in_depth.getFrame()
                detections = in_dets.detections
                selected_target, held_xyz, missed_frames, keep_samples = track_selected_target(
                    detections, labels, homogeneous, held_xyz, missed_frames,
                )

                accept_grasp_sample = not first_run
                if first_run:
                    time.sleep(1)
                    first_run = False

                grasp_samples = update_grasp_samples(
                    grasp_samples,
                    selected_target,
                    labels,
                    keep_samples,
                    accept_grasp_sample,
                    busy,
                )

                for det in detections:
                    label = str(det.label)
                    if 0 <= det.label < len(labels):
                        label = labels[det.label]
                    if label == "handle":
                        continue

                    conf = int(det.confidence * 100)
                    x1 = int(det.xmin * frame.shape[1])
                    y1 = int(det.ymin * frame.shape[0])
                    x2 = int(det.xmax * frame.shape[1])
                    y2 = int(det.ymax * frame.shape[0])

                    if label in NON_CUP_LABELS:
                        # Non-cup auxiliary detection (e.g. Gripper in MARC / FT-MARC models):
                        # show in muted gray for debugging only; never select or send to robot.
                        cv.rectangle(frame, (x1, y1), (x2, y2), (128, 128, 128), 1)
                        cv.putText(
                            frame,
                            f"{label} ({conf}%) [ignored]",
                            (x1 + 5, y1 + 20),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            (200, 200, 200),
                            1,
                        )
                        continue

                    coords = det.spatialCoordinates
                    robot_xyz = robot_xyz_from_relations(
                        coords.x, coords.y, coords.z, homogeneous,
                    )
                    if not is_within_pickup_work_range(robot_xyz):
                        box_color = (128, 128, 128)
                    else:
                        box_color = (0, 0, 255) if det is selected_target else (0, 255, 0)
                    cv.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

                    cv.putText(frame, f"{label} ({conf}%)", (x1 + 5, y1 + 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv.putText(frame, f"X: {int(coords.x)} mm", (x1 + 5, y1 + 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv.putText(frame, f"Y: {int(coords.y)} mm", (x1 + 5, y1 + 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv.putText(frame, f"Z: {int(coords.z)} mm", (x1 + 5, y1 + 65), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    pixel = project_spatial_pixel(
                        coords, camera_matrix, distortion, frame.shape[:2],
                    )
                    if pixel is not None:
                        draw_spatial_dot(frame, pixel, det is selected_target)
                    if det is selected_target:
                        with client._mutex_variable:
                            client.MugCoordinates = robot_xyz
                    cv.putText(frame, f"R: {robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}", (x1 + 5, y1 + 80), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    if det is selected_target and grasp_samples:
                        median_xyz = median_grasp_xyz(grasp_samples)
                        median_robot = robot_xyz_from_relations(
                            float(median_xyz[0]),
                            float(median_xyz[1]),
                            float(median_xyz[2]),
                            homogeneous,
                        )
                        cv.putText(
                            frame,
                            f"M{len(grasp_samples)}: {median_robot[0]:.1f}, {median_robot[1]:.1f}, {median_robot[2]:.1f}",
                            (x1 + 5, y1 + 95),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 255),
                            1,
                        )

                cv.putText(
                    frame,
                    f"[{cs.DETECTION_MODELS[model_choice]['menu_id']}] {model_choice} | grasp {len(grasp_samples)}/{GRASP_FRAME_COUNT}",
                    (10, 30),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                )
                cv.imshow("RGB", frame)
                if yumi_capture is not None:
                    yumi_capture.handle(frame)
                key = cv.waitKey(1) & 0xFF
                if not busy and len(grasp_samples) >= GRASP_FRAME_COUNT:
                    stacked = np.vstack([sample["xyz"] for sample in grasp_samples])
                    median_xyz = np.median(stacked, axis=0)
                    spread = np.max(stacked, axis=0) - np.min(stacked, axis=0)
                    label = grasp_samples[-1]["label"]
                    conf = int(np.median([sample["conf"] for sample in grasp_samples]))
                    grasp_samples = []
                    target_xyz = robot_xyz_from_relations(
                        float(median_xyz[0]),
                        float(median_xyz[1]),
                        float(median_xyz[2]),
                        homogeneous,
                    )
                    print(
                        f"[AUTO] Next mug from {GRASP_FRAME_COUNT} frames "
                        f"({label}, {conf}%)"
                    )
                    print(
                        f"[HT] median camera XYZ mm: "
                        f"{median_xyz[0]:.1f}, {median_xyz[1]:.1f}, {median_xyz[2]:.1f}"
                    )
                    print(
                        f"[HT] 5-frame range mm: "
                        f"{spread[0]:.1f}, {spread[1]:.1f}, {spread[2]:.1f}"
                    )
                    print(f"[HT] robot XYZ mm (X/Z compensated): {target_xyz}")
                    try:
                        normalized_vector = orientation_map.get(label)
                        if normalized_vector is None:
                            print(f"[WARN] Unsupported grasp orientation label '{label}'; skipping move.")
                            continue
                        norm = np.matmul(rotation_matrix, normalized_vector)
                        print(f"[DEBUG] normal vector: {normalized_vector}")
                        if (abs(normalized_vector[1]) < abs(normalized_vector[2])) or (
                            abs(normalized_vector[1]) < abs(normalized_vector[0])
                        ):
                            norm -= [0, 0, np.dot(norm, [0, 0, 1])]
                            norm = norm / np.sqrt(np.dot(norm, norm))
                            np.set_printoptions(precision=3)
                        else:
                            norm[2] = normalized_vector[1] / abs(normalized_vector[1])
                            norm[1] = 0
                            norm[0] = 0
                        print(f"[DEBUG] normal vector after matrix: {norm}")
                        threading.Thread(
                            target=local_move,
                            args=(
                                quaternion,
                                client,
                                target_xyz,
                                [float(norm[0]), float(norm[1]), float(norm[2])],
                                save_protocol,
                                file_path,
                                conf,
                                label,
                            ),
                            daemon=True,
                        ).start()
                        time.sleep(0.5)
                    except Exception as e:
                        print(f"Error {e}")
                if key == ord("r"):
                    client.connect()
                if key == ord("q"):
                    break

        cv.destroyAllWindows()
        tc.close()

        if save_protocol:
            tp.ask_user(file_path, "end")
            tp.log_time_summary(file_path, times_sec)


if __name__ == "__main__":
    try:
        run()
    finally:
        tc.close()
        cv.destroyAllWindows()
