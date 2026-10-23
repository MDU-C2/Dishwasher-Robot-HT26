import cv2 as cv
import numpy as np
import depthai as dai
import time
from updated_communication import Communication
import camera_setup as cs
import test_protocol as tp
import threading
try:
    import yumi_capture
except ImportError:
    yumi_capture = None

#==================================== THREADING SETUP ====================================
global busy, times_sec, count
busy = False
count = 0
times_sec = []

lock = threading.Lock()


def build_homogeneous(rotation_matrix, translation_vector):
    T_camera_to_base_effector = np.eye(4)
    T_camera_to_base_effector[:3, :3] = rotation_matrix
    T_camera_to_base_effector[:3, 3] = translation_vector.reshape(3)
    return T_camera_to_base_effector


def x_compensate(x):
    """HT X in, compensated X out. y = k*x + m from the nine 2026-10-13 cups."""
    y = 1.4018216887159423 * x + -198.52114324095436
    return y


def z_compensate(z):
    """HT Z in, compensated Z out. y = k*z + m from the same nine cups."""
    y = -0.01013848696276861 * z + 41.08061364523784
    return y


def robot_xyz_from_relations(x, y, z, homogeneous):
    """Camera XYZ -> this HT -> x_compensate -> z_compensate -> round once."""
    point = np.array([x, y, z, 1.0], dtype=np.float64)
    ht_xyz = homogeneous.dot(point)[:3]
    robot_x = x_compensate(ht_xyz[0])
    robot_z = z_compensate(ht_xyz[2])
    return np.around([robot_x, ht_xyz[1], robot_z], 2).tolist()

# Fixed right-arm place target. The 2D tray camera is not part of this module.
FIXED_SLOT_NAME = "slot_1"
FIXED_SLOT_COORDS = [302.0, -508.0, 248.0]  # mm, robot frame


def select_placement_slot():
    """Return the fixed place target. No tray camera is opened."""
    coords = [float(v) for v in FIXED_SLOT_COORDS]
    print(f"[TRAY] Fixed slot '{FIXED_SLOT_NAME}' @ {coords}")
    return FIXED_SLOT_NAME, coords

# Camera XYZ of pSafeEntryNormal trans [182, 255, 55] mm, inverted with the
# active HT in this file. X right, Y down, Z forward. Quaternion is not used.
LEFT_ENTRY_CAMERA_XYZ_MM = np.array(
    [-254.3162, -147.0899, 357.3541],
    dtype=np.float64,
)


def select_nearest_target(detections, labels):
    """Nearest valid detection to the left-entry reference. Mug pose is not used."""
    if LEFT_ENTRY_CAMERA_XYZ_MM is None:
        return None
    reference = np.asarray(LEFT_ENTRY_CAMERA_XYZ_MM, dtype=np.float64).reshape(3)
    if not np.isfinite(reference).all():
        return None
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


# Five detections of one cup. Median rejects a single depth spike better than a mean.
GRASP_FRAME_COUNT = 5
SAME_CUP_DISTANCE_MM = 40.0


def camera_xyz_of(detection):
    coords = detection.spatialCoordinates
    return np.array([coords.x, coords.y, coords.z], dtype=np.float64)


def grasp_sample_matches(samples, label, camera_xyz):
    """True when this detection is still the cup already in the window."""
    if not samples:
        return True
    if label != samples[0]["label"]:
        return False
    center = np.median(np.vstack([sample["xyz"] for sample in samples]), axis=0)
    return float(np.linalg.norm(camera_xyz - center)) <= SAME_CUP_DISTANCE_MM


def median_grasp_xyz(samples):
    return np.median(np.vstack([sample["xyz"] for sample in samples]), axis=0)


def load_preview_projection(device, width, height, model_name):
    """Intrinsics for the preview this model actually displays.

    v5 keeps the default center crop of 720p. v5s and v8 stretch 1080p to the
    network size. The dot uses these intrinsics only; camera XYZ is unchanged.
    """
    calibration = device.readCalibration()
    distortion = np.asarray(
        calibration.getDistortionCoefficients(dai.CameraBoardSocket.RGB),
        dtype=np.float64,
    ).reshape(-1)
    if distortion.size != 14 or not np.isfinite(distortion).all():
        raise ValueError("Expected 14 finite RGB distortion coefficients")
    if model_name in ("v5s", "v8"):
        matrix = np.asarray(
            calibration.getCameraIntrinsics(
                dai.CameraBoardSocket.RGB,
                int(width),
                int(height),
                keepAspectRatio=False,
            ),
            dtype=np.float64,
        )
        print(f"[DOT] {model_name} stretched 1080p preview {width}x{height}")
    else:
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
            f"[DOT] {model_name} 720p center crop {width}x{height} "
            f"x0={x0:.1f} y0={y0:.1f}"
        )
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


# Move to one cup coordinate. The caller passes the 5-frame median of the red-box target.
# Do not queue earlier cups; the next target is chosen again from the live frame.
def local_move(orient, client, target_xyz, normalized_vector, save_protocol, file_path, conf=None, label=None):

    global busy, times_sec, count

    with lock:

        if busy:
            return

        start_time = time.time()

        busy = True

        try:

            try:

                chosen_slot, coords = select_placement_slot()

                if chosen_slot is None or coords is None:

                    print(
                        "[TRAY] ERROR: "
                        "No placement slot."
                    )

                    return

                client.SetFreeSlot(
                    coords
                )

                print(
                    f"[TRAY] FreeSlot stored: "
                    f"{client.FreeSlot}"
                )

            except Exception as exc:

                print(
                    f"[TRAY] ERROR selecting "
                    f"free slot: {exc}"
                )

                return

            client.MoveHome()

            print(
                "[ROBOT] Starting PickUpSequence..."
            )

            client.PickUpSequence(
                target_xyz,
                orient,
                normalized_vector
            )

            print(
                "[ROBOT] PickUpSequence completed."
            )

            end_time = time.time()

            process_time = (
                end_time - start_time
            )

            if save_protocol:

                count += 1

                tp.cup_information(
                    file_path,
                    count,
                    target_xyz,
                    process_time,
                    confidence=conf,
                    label=label
                )

                times_sec.append(
                    process_time
                )

        except Exception as exc:

            print(
                f"[ROBOT] ERROR in local_move: "
                f"{exc}"
            )

        finally:

            busy = False

def choose_detection_model():
    """Ask once which copied detection package to load."""
    model_by_input = {"5": "v5", "5s": "v5s", "8": "v8"}
    while True:
        choice = input("Run detection model v5, v5s, or v8? Enter 5, 5s, or 8: ").strip().lower()
        model = model_by_input.get(choice)
        if model is not None:
            print(f"[MODEL] Selected {model}.")
            return model
       


def run():
    global busy, times_sec
    model_choice = choose_detection_model()
    file_path = None
    normalized_vector = [0,0,1] # Initial orientation vector for the gripper
    #Normalized orientation vectors for different cup orientations
    orientation_map = {
            'Back': [ 0.0, 0.0,1.0],
            'Front': [0.0, 0.0,-1.0],
            'left_side': [1.0, 0.0, 0.0],
            'right_side': [-1.0, 0.0, 0.0],
            'upright': [0.0, 1.0 ,0.0],
            'upside_down': [0.0, -1.0, 0.0],
            'Gripper': [0.0, 0.0, -1.0],
        }
    quaternion = [1,0,0,0] # Dump value, not used in robot but needs to be sent.
    client = Communication()

    def find_empty_slot_in_dishwasher():
        """Dynamic callback triggered when RAPID sends 'Ask_LeavePosition'."""
        try:
            chosen_slot, coords = select_placement_slot()
            if coords is None:
                print("[DYNAMIC TRAY] WARNING: No placement slot.")
                return None
            print(f"[DYNAMIC TRAY] Selected slot: '{chosen_slot}' @ {coords}")
            return coords
        except Exception as exc:
            print(f"[DYNAMIC TRAY ERROR] {exc}")
            return None

    client.get_free_slot_logic = find_empty_slot_in_dishwasher

    if client.connectV2():

        pipeline, labels = cs.camera_setup(model_choice)

        # Best cup-table matrix: raw HT_New_Run2_Mean.
        # x_compensate and z_compensate are the fitted results.
        # To change matrix later: replace this matrix, collect new cup pairs outside
        # this file, then replace the two y = k*input + m lines.
        active_ht_name = "raw / HT_New_Run2_Mean"
        rotation_matrix = np.array([
            [0.01987069782385702, -0.41217386010538953, 0.9108885027344543], 
            [-0.9996927890512148, 0.005309202233500268, 0.024210326116078228,], 
            [-0.014814954842861787, -0.9110897438877604, -0.4119417382294947,]
        ], dtype=np.float64),
 #       rotation_matrix = np.array([
  #          [-0.002597731108439694, -0.3876317471376878, 0.9218106532276962],
   #         [-0.9999744383587065, -0.005133658742591244, -0.004976763717912172],
    #        [0.00666141293402215, -0.9218000185283671, -0.3876085027689978],
     #   ], dtype=np.float64)
        translation_vector = np.array(
            [-76.79838535532201,-7.726452078895317,322.76940785633104],
            dtype=np.float64,
  #      translation_vector = np.array(
  #         [-84.04769438994447, 10.90679989710399, 319.0687334399627],
  #          dtype=np.float64,
  #      )
        homogeneous = build_homogeneous(rotation_matrix, translation_vector)

        
        print(f"[HT] active {active_ht_name}. Y unchanged.")
        print("[HT] x_compensate: y = 1.401822 * x - 198.521")
        print("[HT] z_compensate: y = -0.010138 * z + 41.081")

  #      'HT_Old_Combined_CleanedSensitivity': np.array([[0.018982770100808496, -0.4068607861158505, 0.9132929185975833, -76.47106945112552], [-0.9997373150593256, 0.004010000404984149, 0.022565920648672393, -8.531241392687775], [-0.012843493187991908, -0.9134813739852301, -0.406677788998595, 319.037447170287], [0.0, 0.0, 0.0, 1.0]], dtype=np.float64),
  #  'HT_New_Combined_CleanedSensitivity': np.array([[0.018756353021267352, -0.41313445155433615, 0.9104768663509464, -77.88135492386857], [-0.9998122222058595, -0.0033143365670869193, 0.019092812807459576, -3.224783864504765], [-0.004870271976205742, -0.9106640105505528, -0.4131190389450238, 324.7791678928043], [0.0, 0.0, 0.0, 1.0]], dtype=np.float64),
   # 'HT_Global_CleanedSensitivity': np.array([[0.01987069782385702, -0.41217386010538953, 0.9108885027344543, -76.79838535532201], [-0.9996927890512148, 0.005309202233500268, 0.024210326116078228, -7.726452078895317], [-0.014814954842861787, -0.9110897438877604, -0.4119417382294947, 322.76940785633104], [0.0, 0.0, 0.0, 1.0]], dtype=np.float64),
}

        #==================================== TEST PROTOCOL SETUP ====================================


        save_protocol = input("Do you want to save the test protocol? (y/n): ").lower() == 'y'
        if save_protocol:
            file_path = tp.create_today_textfile()
            tp.ask_user(file_path, "start")
            tp.fill_meta_data(file_path, active_ht_name)



        #====================================  MAIN  ====================================


        first_run = True
        grasp_samples = []
        if LEFT_ENTRY_CAMERA_XYZ_MM is None:
            print(
                "[SELECT] LEFT_ENTRY_CAMERA_XYZ_MM is not set. "
                "No target will be chosen until that camera XYZ is filled in."
            )
        with dai.Device(pipeline) as device:
            # Output queues to retrieve frames and detections
            q_rgb   = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            q_det   = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            projection = None

            cv.namedWindow('RGB', cv.WINDOW_NORMAL)

            while True:
                in_rgb   = q_rgb.get() # latest RGB frame
                in_depth = q_depth.get() # latest depth frame (aligned to RGB)
                in_dets  = q_det.get() # latest detection results


                frame = in_rgb.getCvFrame() # OpenCV BGR frame from color camera
                if projection is None:
                    projection = load_preview_projection(
                        device, frame.shape[1], frame.shape[0], model_choice,
                    )
                camera_matrix, distortion = projection
                depth_frame = in_depth.getFrame() # Depth data in millimeters
                detections = in_dets.detections # List of spatial detections
                selected_target = select_nearest_target(detections, labels)

                # The first packet is only for focus and is not part of the 5-frame window.
                accept_grasp_sample = not first_run
                if first_run:
                    time.sleep(1)
                    first_run = False

                if busy:
                    grasp_samples = []
                elif accept_grasp_sample and selected_target is not None:
                    camera_xyz = camera_xyz_of(selected_target)
                    sample_label = labels[selected_target.label]
                    previous_count = len(grasp_samples)
                    if grasp_samples and not grasp_sample_matches(grasp_samples, sample_label, camera_xyz):
                        print('[FRAME] Target changed. 5-frame window restarted.')
                        grasp_samples = []
                        previous_count = 0
                    grasp_samples.append({
                        'xyz': camera_xyz,
                        'label': sample_label,
                        'conf': int(selected_target.confidence * 100),
                    })
                    if len(grasp_samples) > GRASP_FRAME_COUNT:
                        grasp_samples = grasp_samples[-GRASP_FRAME_COUNT:]
                    if len(grasp_samples) < GRASP_FRAME_COUNT:
                        print(
                            f'[FRAME] {len(grasp_samples)}/{GRASP_FRAME_COUNT} '
                            f'{sample_label} camera XYZ mm: '
                            f'{camera_xyz[0]:.1f}, {camera_xyz[1]:.1f}, {camera_xyz[2]:.1f}'
                        )
                    elif previous_count < GRASP_FRAME_COUNT:
                        print('[FRAME] 5/5 ready. Automatic pick will send the median.')

                # Iterate over detections and draw bounding boxes and labels
                for det in detections:

                    # Determine label text to display
                    label = str(det.label)
                    if det.label < len(labels):
                        label = labels[det.label]
                    if label != 'handle':

                        conf  = int(det.confidence * 100) # Confidence percentage

                        # Get bounding box coordinates
                        x1 = int(det.xmin * frame.shape[1])
                        y1 = int(det.ymin * frame.shape[0])
                        x2 = int(det.xmax * frame.shape[1])
                        y2 = int(det.ymax * frame.shape[0])

                        # Nearest target to the left-entry reference is red; others stay green.
                        box_color = (0, 0, 255) if det is selected_target else (0, 255, 0)
                        cv.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

                        # Draw label and confidence
                        cv.putText(frame, f'{label} ({conf}%)', (x1+5, y1+20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

                        # Camera XYZ stays white. X/Z-compensated robot XYZ is purple.
                        coords = det.spatialCoordinates  # Spatial coordinates relative to camera
                        cv.putText(frame, f'X: {int(coords.x)} mm', (x1+5, y1+35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                        cv.putText(frame, f'Y: {int(coords.y)} mm', (x1+5, y1+50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                        cv.putText(frame, f'Z: {int(coords.z)} mm', (x1+5, y1+65), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                        pixel = project_spatial_pixel(
                            coords, camera_matrix, distortion, frame.shape[:2],
                        )
                        if pixel is not None:
                            draw_spatial_dot(frame, pixel, det is selected_target)
                        robot_xyz = robot_xyz_from_relations(
                            coords.x, coords.y, coords.z, homogeneous,
                        )
                        if det is selected_target:
                            with client._mutex_variable:
                                client.MugCoordinates = robot_xyz                        
                        cv.putText(frame, f'R: {robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}', (x1+5, y1+80), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
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
                                f'M{len(grasp_samples)}: {median_robot[0]:.1f}, {median_robot[1]:.1f}, {median_robot[2]:.1f}',
                                (x1+5, y1+95),
                                cv.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 255),
                                1,
                            )

                    # Show the frames in windows
                cv.putText(
                    frame,
                    f'grasp frames {len(grasp_samples)}/{GRASP_FRAME_COUNT}',
                    (10, 30),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
                cv.imshow('RGB', frame)
                if yumi_capture is not None:
                    yumi_capture.handle(frame)
                key = cv.waitKey(1) & 0xFF
                # ==========================================
                # AUTOMATIC PICK-AND-PLACE
                # Five frames of the same red-box cup, then send without space.
                # ==========================================
                if not busy and len(grasp_samples) >= GRASP_FRAME_COUNT:
                    stacked = np.vstack([sample['xyz'] for sample in grasp_samples])
                    median_xyz = np.median(stacked, axis=0)
                    spread = np.max(stacked, axis=0) - np.min(stacked, axis=0)
                    label = grasp_samples[-1]['label']
                    conf = int(np.median([sample['conf'] for sample in grasp_samples]))
                    grasp_samples = []
                    target_xyz = robot_xyz_from_relations(
                        float(median_xyz[0]),
                        float(median_xyz[1]),
                        float(median_xyz[2]),
                        homogeneous,
                    )
                    print(
                        f'[AUTO] Next mug from {GRASP_FRAME_COUNT} frames '
                        f'({label}, {conf}%)'
                    )
                    print(
                        f'[HT] median camera XYZ mm: '
                        f'{median_xyz[0]:.1f}, {median_xyz[1]:.1f}, {median_xyz[2]:.1f}'
                    )
                    print(
                        f'[HT] 5-frame range mm: '
                        f'{spread[0]:.1f}, {spread[1]:.1f}, {spread[2]:.1f}'
                    )
                    print(f'[HT] robot XYZ mm (X/Z compensated): {target_xyz}')
                    try:
                        normalized_vector = orientation_map.get(label)
                        norm = np.matmul(rotation_matrix, normalized_vector)
                        print(f"[DEBUG] normal vector: {normalized_vector}")
                        if (abs(normalized_vector[1]) < abs(normalized_vector[2])) or (abs(normalized_vector[1]) < abs(normalized_vector[0])):
                            norm -= [0, 0, np.dot(norm, [0, 0, 1])]
                            norm = norm / np.sqrt(np.dot(norm, norm))
                            np.set_printoptions(precision=3)
                        else:
                            norm[2] = normalized_vector[1] / abs(normalized_vector[1])
                            norm[1] = 0
                            norm[0] = 0
                        print(f"[DEBUG] normal vector after matrix: {norm}")
                        threading.Thread(target=local_move, args=(quaternion, client, target_xyz, [float(norm[0]), float(norm[1]), float(norm[2])], save_protocol, file_path, conf, label), daemon=True).start()
                        # Prevent instant re-trigger
                        time.sleep(0.5)
                    except Exception as e:
                        print(f"Error {e}")
                if key == ord('r'):
                    client.connect()
                # Exit on 'q' key
                if key == ord('q'):

                    break

        cv.destroyAllWindows()

        if save_protocol:
            tp.ask_user(file_path, "end")
            tp.log_time_summary(file_path, times_sec)

if __name__ == "__main__":
    run()
 