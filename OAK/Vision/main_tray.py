import cv2 as cv
import numpy as np
import depthai as dai
import time
from updated_communication import Communication
import camera_setup as cs
import test_protocol as tp
import tray_camera as tc
import threading
#==================================== THREADING SETUP ====================================
global busy, times_sec, count
busy = False
count = 0
times_sec = []

lock = threading.Lock()

def parse_file(path):
    R_list = []
    t_list = []

    with open(path, "r") as f:
        block = []
        for line in f:
            line = line.strip()
            if not line:
                continue
            block.append(line)

            if len(block) == 4:
                R = []
                for i in range(3):
                    R.append([float(x) for x in block[i].replace("[","").replace("]","").split()])
                R = np.array(R)
                t = np.array([float(x) for x in block[3].replace("[","").replace("]","").split()])
                R_list.append(R)
                t_list.append(t)
                block = []

    return np.array(R_list), np.array(t_list)


def average_rotations(Rs):
    M = np.zeros((3, 3))
    for R in Rs:
        M += R
    M /= len(Rs)

    # Project back to closest proper rotation matrix
    U, _, Vt = np.linalg.svd(M)
    R_avg = U @ Vt
    return R_avg


def average_translations(ts):
    return np.mean(ts, axis=0)


def save_result(path, R_avg, t_avg):
    with open(path, "w") as f:
        # Same style as your input: 3 lines for R, 1 line for t
        for row in R_avg:
            f.write("[" + " ".join(f"{v:.8f}" for v in row) + "]\n")
        f.write("[" + " ".join(f"{v:.8f}" for v in t_avg) + "]\n")


def build_homogeneous(rotation_matrix, translation_vector):
    T_camera_to_base_effector = np.eye(4)
    T_camera_to_base_effector[:3, :3] = rotation_matrix
    T_camera_to_base_effector[:3, 3] = translation_vector.reshape(3)
    return T_camera_to_base_effector

# Right-arm place target after handover.
# "fixed"  — one robot-frame point, computer camera stays closed.
# "camera" — tray_camera scans the rack and returns a free slot.
SLOT_SOURCE = "fixed"
FIXED_SLOT_NAME = "slot_1"
FIXED_SLOT_COORDS = [302.0, -508, 248.0]  # mm, robot frame; same fixed slot as 1005

_nxt = None


def init_slot():
    """Initialize next free slot."""
    global _nxt
    _nxt = _nxt or tc.reserve()
    return _nxt or (None, None)


def setup():
    """Calibrate or load, initialize."""
    saved = bool(tc.boxes())
    if not saved:
        print("[TRAY] No saved calibration.")
    if not saved or input("\nCalibrate slots? (y/n): ").strip().lower() == "y":
        if not tc.calibrate() and not saved:
            return False
    tc.check()
    if init_slot()[0] is None:
        print("[TRAY] No free slot. Empty the tray or calibrate again.")
        return False
    return True


def end_slot():
    """Clear slot, prepare next."""
    global _nxt
    if SLOT_SOURCE != "camera":
        return
    if _nxt:
        tc.drop(_nxt[0])
        _nxt = None
    init_slot()


def select_placement_slot():
    """Return (slot_name, [x, y, z]) for the right arm, or (None, None)."""
    match SLOT_SOURCE:
        case "fixed":
            coords = [float(v) for v in FIXED_SLOT_COORDS]
            print(f"[TRAY] Source=fixed  slot '{FIXED_SLOT_NAME}' @ {coords}")
            return FIXED_SLOT_NAME, coords
        case "camera":
            chosen, coords = init_slot()
            if chosen is not None:
                print(f"[TRAY] Source=camera  slot '{chosen}' @ {coords}")
            return chosen, coords
        case _:
            print(f"[TRAY] ERROR: unknown SLOT_SOURCE '{SLOT_SOURCE}'.")
            return None, None


def confirm_placement(slot_name):
    """Check the rack after a place. Fixed mode does not open the 2D camera."""
    match SLOT_SOURCE:
        case "fixed":
            print(f"[TRAY] Source=fixed  placement check skipped for '{slot_name}'.")
            return None
        case "camera":
            return tc.verify(slot_name)
        case _:
            print(f"[TRAY] ERROR: unknown SLOT_SOURCE '{SLOT_SOURCE}'.")
            return None

# def select_target(detections, labels):
#     """Leftmost valid upright cup, matching vision_test.select_target."""
#     selected_target = None
#     for det in detections:
#         if not 0 <= det.label < len(labels):
#             continue
#         if labels[det.label] != "upright":
#             continue
#         coords = det.spatialCoordinates
#         if not np.isfinite([coords.x, coords.y, coords.z]).all() or coords.z <= 0:
#             continue
#         if selected_target is None or coords.x < selected_target.spatialCoordinates.x:
#             selected_target = det
#     return selected_target

# def select_leftmost_target(detections, labels):
#     """Leftmost detection with valid depth. Mug pose is not used."""
#     selected_target = None
#     for det in detections:
#         if not 0 <= det.label < len(labels):
#             continue
#         if labels[det.label] == "handle":
#             continue
#         coords = det.spatialCoordinates
#         if not np.isfinite([coords.x, coords.y, coords.z]).all() or coords.z <= 0:
#             continue
#         if selected_target is None or coords.x < selected_target.spatialCoordinates.x:
#             selected_target = det
#     return selected_target

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

            try:

                time.sleep(0.5)

                placed_ok = False
                while not placed_ok:
                    placed_ok = confirm_placement(
                        chosen_slot
                    )
                    if placed_ok is None:
                        break
                    if placed_ok:
                        print(
                            f"[TRAY] Placement verified "
                            f"in slot '{chosen_slot}'."
                        )
                        break
                    print(
                        "[TRAY] Waiting for mug "
                        "to appear in dishwasher..."
                    )
                    time.sleep(0.5)

            except Exception as exc:

                print(
                    f"[TRAY] ERROR during "
                    f"placement verification: {exc}"
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

            end_slot()

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
    cam_coords = 'saved_coordinates.txt' # Path to camera coordinates .txt file
    robot_file = 'robo_coords.txt' # Path to robot coordinates .txt file
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

    if SLOT_SOURCE == "camera" and not setup():
        tc.close()
        return

    if client.connectV2():

        homogeneous, syncNN, pipeline, labels, rotation_matrix, translation_vector, camera_points, robot_points = cs.camera_setup(cam_coords, robot_file, model_choice)

        #rotation_matrix = np.array([
        #    [0.007144178280750622, -0.40899573676986206, 0.9125083276446139],
        #    [-0.9999562095426, 0.0025942603063787294, 0.008991596669013288],
        #    [-0.006044808838028906, -0.9125326060571975, -0.40895929280133886],
        #], dtype=np.float64)
        #translation_vector = np.array(
        #    [-59.28905853814081, -6.733211959000464, 323.56564041743616],
        #    dtype=np.float64,
        #)
        # rotation_matrix = np.array([
        #     [-0.052720905575654821, -0.40009299202319931, 0.91495688633356154],
        #     [-0.99805997511225542, -0.009274843734204832, -0.061565114737399375],
        #     [0.033117853103816257, -0.91642761580368592, -0.3988278235005222],
        # ], dtype=np.float64)
        # translation_vector = np.array(
        #     [-59.916837258921817, 34.30217408063033, 332.29792664183373],
        #     dtype=np.float64,
        # )
        # HT1002, funkade bäst
        rotation_matrix = np.array([
            [0.0411876989, 0.3951223237, 0.9177047035],
            [-0.9991006428, 0.0070268024, 0.0418154225],
            [-0.0100736773, 0.9186016402, -0.3950563854],
        ], dtype=np.float64)
        translation_vector = np.array(
            [-77.3523678182, -12.9968405542, 328.7301604122],
            dtype=np.float64,
        )


        # HT1005, 16 points used of 61
        # rotation_matrix = np.array([
        #     [0.0177970357, -0.4078098903, 0.9128933995],
        #     [-0.9998374913, -0.0098828949, 0.0150771177],
        #     [0.0028734318, -0.9130133743, -0.4079195039],
        # ], dtype=np.float64)
        # translation_vector = np.array(
        #     [-121.5552664800, 16.0664362279, 414.6434804753],
        #     dtype=np.float64,
        # )

        homogeneous = build_homogeneous(rotation_matrix, translation_vector)

        # get avg of matrices
        #input_file = "AveragedOutput.txt"
        # input_file = "RT.txt"
        # R_list, t_list = parse_file(input_file)

        # R_avg = average_rotations(R_list)
        # t_avg = average_translations(t_list)

        #homogeneous = build_homogeneous(R_avg, t_avg)
        #rotation_matrix = R_avg
        #translation_vector = t_avg
        #==================================== TEST PROTOCOL SETUP ====================================


        save_protocol = input("Do you want to save the test protocol? (y/n): ").lower() == 'y'
        if save_protocol:
            file_path = tp.create_today_textfile()
            tp.ask_user(file_path, "start")
            tp.fill_meta_data(file_path)
            rms_error = cs.rms_alignment_error(camera_points, robot_points, rotation_matrix, translation_vector)
            tp.log_rms_error(file_path, rms_error)



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


            while True:
                if SLOT_SOURCE == "camera":
                    tray_img = tc.view()
                    if tray_img is not None:
                        cv.imshow("Tray Status (2D)", tray_img)
                in_rgb   = q_rgb.get() # latest RGB frame
                in_depth = q_depth.get() # latest depth frame (aligned to RGB)
                in_dets  = q_det.get() # latest detection results


                frame = in_rgb.getCvFrame() # OpenCV BGR frame from color camera
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

                        # Camera XYZ stays white. Robot XYZ from the current HT is purple.
                        coords = det.spatialCoordinates  # Spatial coordinates relative to camera
                        cv.putText(frame, f'X: {int(coords.x)} mm', (x1+5, y1+35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                        cv.putText(frame, f'Y: {int(coords.y)} mm', (x1+5, y1+50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                        cv.putText(frame, f'Z: {int(coords.z)} mm', (x1+5, y1+65), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                        robot_xyz = cs.convert_coordinates(coords.x, coords.y, coords.z, homogeneous)
                        cv.putText(frame, f'R: {robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}', (x1+5, y1+80), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                        if det is selected_target and grasp_samples:
                            median_xyz = median_grasp_xyz(grasp_samples)
                            median_robot = cs.convert_coordinates(
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
                    target_xyz = cs.convert_coordinates(
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
                    print(f'[HT] robot XYZ mm: {target_xyz}')
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
        tc.close()

        if save_protocol:
            tp.ask_user(file_path, "end")
            tp.log_time_summary(file_path, times_sec)

if __name__ == "__main__":
    run()