import cv2 as cv
import numpy as np
import depthai as dai
import time
import threading
from updated_communication import Communication
import camera_setup as cs
import test_protocol as tp
import tray_camera_yolo as tc
try:
    import yumi_capture
except ImportError:
    yumi_capture = None
global busy, times_sec, count
busy = False
count = 0
times_sec = []
lock = threading.RLock()
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
                    R.append([float(x) for x in block[i].replace("[", "").replace("]", "").split()])
                R = np.array(R)
                t = np.array([float(x) for x in block[3].replace("[", "").replace("]", "").split()])
                R_list.append(R)
                t_list.append(t)
                block = []
    return np.array(R_list), np.array(t_list)
def average_rotations(Rs):
    M = np.zeros((3, 3))
    for R in Rs:
        M += R
    M /= len(Rs)
    U, _, Vt = np.linalg.svd(M)
    R_avg = U @ Vt
    return R_avg
def average_translations(ts):
    return np.mean(ts, axis=0)
def save_result(path, R_avg, t_avg):
    with open(path, "w") as f:
        for row in R_avg:
            f.write("[" + " ".join(f"{v:.8f}" for v in row) + "]\n")
        f.write("[" + " ".join(f"{v:.8f}" for v in t_avg) + "]\n")
def build_homogeneous(rotation_matrix, translation_vector):
    T_camera_to_base_effector = np.eye(4, dtype=np.float64)
    T_camera_to_base_effector[:3, :3] = rotation_matrix
    T_camera_to_base_effector[:3, 3] = translation_vector.reshape(3)
    return T_camera_to_base_effector
def x_compensate(x):
    return 1.4018216887159423 * x - 198.52114324095436
def y_compensate(y):
    return 0.9383537849847798 * y - 8.3682309221822742
def z_compensate(z):
    return -0.01013848696276861 * z + 41.08061364523784
def robot_xyz_from_relations(x, y, z, homogeneous):
    point = np.array([x, y, z, 1.0], dtype=np.float64)
    ht_xyz = homogeneous.dot(point)[:3]
    robot_x = x_compensate(ht_xyz[0])
    robot_y = y_compensate(ht_xyz[1])
    robot_z = z_compensate(ht_xyz[2])
    return np.around([robot_x, robot_y, robot_z], 2).tolist()
SLOT_SOURCE = "camera"
FIXED_SLOT_NAME = "slot_1"
FIXED_SLOT_COORDS = [302.0, -508.0, 248.0]
_cur = None
def setup():
    saved = bool(tc.boxes())
    if not saved:
        print("[TRAY] No saved calibration found.")
    if not saved or input("\nCalibrate slots? (y/n): ").strip().lower() == "y":
        if not tc.calibrate() and not saved:
            return False
    tc.report()
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
    [-254.3162, -147.0899, 357.3541],
    dtype=np.float64,
)
def select_nearest_target(detections, labels):
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
GRASP_FRAME_COUNT = 3
SAME_CUP_DISTANCE_MM = 40.0
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
            try:
                chosen_slot, coords = select_placement_slot()
                if chosen_slot is None or coords is None:
                    print("[TRAY] ERROR: No placement slot available from YOLO tray monitor.")
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
            if not finished:
                end_slot(chosen_slot, started)
            with lock:
                busy = False
                client.FreeSlot = None
def choose_detection_model():
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
    normalized_vector = [0, 0, 1]
    orientation_map = {
        'Back': [0.0, 0.0, 1.0],
        'Front': [0.0, 0.0, -1.0],
        'left_side': [1.0, 0.0, 0.0],
        'right_side': [-1.0, 0.0, 0.0],
        'upright': [0.0, 1.0, 0.0],
        'upside_down': [0.0, -1.0, 0.0],
        'Gripper': [0.0, 0.0, -1.0],
    }
    quaternion = [1, 0, 0, 0]
    cam_coords = 'saved_coordinates.txt'
    robot_file = 'robo_coords.txt'
    client = Communication()
    def find_empty_slot_in_dishwasher():
        try:
            if client.FreeSlot is not None:
                print(f"[DYNAMIC TRAY] Returning pre-cached slot coordinates: {client.FreeSlot}")
                return list(client.FreeSlot)
            chosen_slot, coords = select_placement_slot()
            if coords is None:
                print("[DYNAMIC TRAY] WARNING: No placement slot available.")
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
        homogeneous, syncNN, pipeline, labels, rotation_matrix, translation_vector, camera_points, robot_points = cs.camera_setup(cam_coords, robot_file, model_choice)
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
        save_protocol = input("Do you want to save the test protocol? (y/n): ").lower() == 'y'
        if save_protocol:
            file_path = tp.create_today_textfile()
            tp.ask_user(file_path, "start")
            tp.fill_meta_data(file_path)
            rms_error = cs.rms_alignment_error(camera_points, robot_points, rotation_matrix, translation_vector)
            tp.log_rms_error(file_path, rms_error)
        first_run = True
        grasp_samples = []
        with dai.Device(pipeline) as device:
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            cv.namedWindow('RGB', cv.WINDOW_NORMAL)
            while True:
                if SLOT_SOURCE == "camera":
                    tray_img = tc.view()
                    if tray_img is not None:
                        cv.imshow("Tray Status (YOLO)", tray_img)
                in_rgb = q_rgb.get()
                in_depth = q_depth.get()
                in_dets = q_det.get()
                frame = in_rgb.getCvFrame()
                detections = in_dets.detections
                selected_target = select_nearest_target(detections, labels)
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
                        print(f'[FRAME] Target changed. {GRASP_FRAME_COUNT}-frame window restarted.')
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
                        print(f'[FRAME] {GRASP_FRAME_COUNT}/{GRASP_FRAME_COUNT} ready. Automatic pick will send the median.')
                for det in detections:
                    label = str(det.label)
                    if det.label < len(labels):
                        label = labels[det.label]
                    if label != 'handle':
                        conf = int(det.confidence * 100)
                        x1 = int(det.xmin * frame.shape[1])
                        y1 = int(det.ymin * frame.shape[0])
                        x2 = int(det.xmax * frame.shape[1])
                        y2 = int(det.ymax * frame.shape[0])
                        box_color = (0, 0, 255) if det is selected_target else (0, 255, 0)
                        cv.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                        cv.putText(frame, f'{label} ({conf}%)', (x1 + 5, y1 + 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        coords = det.spatialCoordinates
                        cv.putText(frame, f'X: {int(coords.x)} mm', (x1 + 5, y1 + 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv.putText(frame, f'Y: {int(coords.y)} mm', (x1 + 5, y1 + 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv.putText(frame, f'Z: {int(coords.z)} mm', (x1 + 5, y1 + 65), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        robot_xyz = robot_xyz_from_relations(coords.x, coords.y, coords.z, homogeneous)
                        if det is selected_target:
                            with client._mutex_variable:
                                client.MugCoordinates = robot_xyz
                        cv.putText(frame, f'R: {robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}', (x1 + 5, y1 + 80), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                        if det is selected_target and grasp_samples:
                            median_xyz = median_grasp_xyz(grasp_samples)
                            median_robot = robot_xyz_from_relations(float(median_xyz[0]), float(median_xyz[1]), float(median_xyz[2]), homogeneous)
                            cv.putText(
                                frame,
                                f'M{len(grasp_samples)}: {median_robot[0]:.1f}, {median_robot[1]:.1f}, {median_robot[2]:.1f}',
                                (x1 + 5, y1 + 95),
                                cv.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 255),
                                1,
                            )
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
                if yumi_capture is not None and hasattr(yumi_capture, "handle"):
                    yumi_capture.handle(frame)
                key = cv.waitKey(1) & 0xFF
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
                    print(f'[AUTO] Next mug from {GRASP_FRAME_COUNT} frames ({label}, {conf}%)')
                    print(f'[HT] median camera XYZ mm: {median_xyz[0]:.1f}, {median_xyz[1]:.1f}, {median_xyz[2]:.1f}')
                    print(f'[HT] {GRASP_FRAME_COUNT}-frame range mm: {spread[0]:.1f}, {spread[1]:.1f}, {spread[2]:.1f}')
                    print(f'[HT] robot XYZ mm (compensated): {target_xyz}')
                    try:
                        normalized_vector = orientation_map.get(label, [0.0, 0.0, 1.0])
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
                        threading.Thread(
                            target=local_move,
                            args=(quaternion, client, target_xyz, [float(norm[0]), float(norm[1]), float(norm[2])], save_protocol, file_path, conf, label),
                            daemon=True,
                        ).start()
                        time.sleep(0.5)
                    except Exception as e:
                        print(f"Error {e}")
                if key == ord('r'):
                    client.connect()
                if key == ord('q'):
                    break
        cv.destroyAllWindows()
        tc.close()
        if save_protocol:
            tp.ask_user(file_path, "end")
            tp.log_time_summary(file_path, times_sec)
if __name__ == "__main__":
    run()
