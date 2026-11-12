import os
import cv2 as cv
import numpy as np
import depthai as dai
import time
import threading
from updated_communication import Communication
import camera_setup as cs
import test_protocol as tp
import tray_camera
import yumi_capture
from config_loader import load_config

# ==================================== CONFIG & SETUP ====================================
_CFG = load_config()
SLOT_SOURCE = _CFG.get("placement", {}).get("slot_source", "camera")
FIXED_SLOT_NAME = _CFG.get("placement", {}).get("fixed_slot_name", "slot_1")
FIXED_SLOT_COORDS = _CFG.get("placement", {}).get("fixed_slot_coords", [302.0, -508.0, 248.0])
Z_UPWARD = float(_CFG.get("mug_heights", {}).get("upward_z", 55.0))
Z_DOWNWARD = float(_CFG.get("mug_heights", {}).get("downward_z", 45.0))

# ==================================== THREADING SETUP ====================================
busy = False
count = 0
times_sec = []
lock = threading.Lock()


def build_homogeneous(rotation_matrix, translation_vector):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation_vector.reshape(3)
    return T


def x_compensate(x):
    """HT X in, compensated X out (fitted from calibration cups)."""
    return 1.4018216887159423 * x - 198.52114324095436


def robot_xyz_from_relations(x, y, z, homogeneous, label="upright"):
    """Camera XYZ -> HT -> x_compensate -> fixed Z (55 for upward, 45 for downward)."""
    point = np.array([x, y, z, 1.0], dtype=np.float64)
    ht_xyz = homogeneous.dot(point)[:3]
    robot_x = x_compensate(ht_xyz[0])
    robot_y = ht_xyz[1]
    is_upward = (str(label).strip().lower() == "upright")
    robot_z = Z_UPWARD if is_upward else Z_DOWNWARD
    return np.around([robot_x, robot_y, robot_z], 2).tolist()


def select_placement_slot():
    """Return (slot_name, [x, y, z]) for the right arm, or (None, None)."""
    match SLOT_SOURCE:
        case "fixed":
            coords = [float(v) for v in FIXED_SLOT_COORDS]
            print(f"[TRAY] Source=fixed  slot '{FIXED_SLOT_NAME}' @ {coords}")
            return FIXED_SLOT_NAME, coords
        case "camera":
            states, _frame = tray_camera.get_free_slots(rescan=True)
            print(f"[TRAY] Source=camera  states: {states}")
            known_coords = tray_camera._coords()
            chosen = next(
                (name for name, state in states.items() if state == "free" and name in known_coords),
                None,
            )
            if chosen is None:
                print("[TRAY] No free calibrated dishwasher slot.")
                return None, None
            coords = known_coords[chosen]
            print(f"[TRAY] Source=camera  slot '{chosen}' @ {coords}")
            return chosen, list(coords)
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
            return tray_camera.verify_placement(slot_name)
        case _:
            print(f"[TRAY] ERROR: unknown SLOT_SOURCE '{SLOT_SOURCE}'.")
            return None


# Camera XYZ of pSafeEntryNormal trans [182, 255, 55] mm inverted with active HT.
LEFT_ENTRY_CAMERA_XYZ_MM = np.array(
    [-254.3162, -147.0899, 357.3541],
    dtype=np.float64,
)

GRASP_FRAME_COUNT = 5
SAME_CUP_DISTANCE_MM = 40.0


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
        if not (0 <= det.label < len(labels)):
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


def camera_xyz_of(detection):
    coords = detection.spatialCoordinates
    return np.array([coords.x, coords.y, coords.z], dtype=np.float64)


def grasp_sample_mismatch_reason(samples, label, camera_xyz):
    """Return None when detection matches the current window, else 'label' or 'distance'."""
    if not samples:
        return None
    if label != samples[0]["label"]:
        return "label"
    center = np.median(np.vstack([sample["xyz"] for sample in samples]), axis=0)
    if float(np.linalg.norm(camera_xyz - center)) > SAME_CUP_DISTANCE_MM:
        return "distance"
    return None


def grasp_sample_matches(samples, label, camera_xyz):
    """True when this detection is still the cup already in the window."""
    return grasp_sample_mismatch_reason(samples, label, camera_xyz) is None


def median_grasp_xyz(samples):
    return np.median(np.vstack([sample["xyz"] for sample in samples]), axis=0)


def local_move(
    orient,
    client,
    target_xyz,
    normalized_vector,
    save_protocol,
    file_path,
    conf=None,
    label=None,
    report_logger=None,
    lock_info=None,
):
    """
    Perform pick-and-place transaction in background thread.
    Robot cycle time is measured strictly from when the robot starts moving
    toward the mug until it has left the mug in the dishwasher tray.
    """
    global busy, times_sec, count

    with lock:
        if busy:
            return
        busy = True

    verify_polls = 0
    placed_ok = None
    chosen_slot = None
    coords = None

    try:
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

        # ======================================================================
        # PURE ROBOT CYCLE TIME: Start of robot movement -> Mug left in tray
        # ======================================================================
        print("[ROBOT] Starting PickUpSequence...")
        t_robot_start = time.perf_counter()
        client.PickUpSequence(target_xyz, orient, normalized_vector)
        robot_cycle_time_s = time.perf_counter() - t_robot_start
        print(f"[ROBOT] PickUpSequence completed. Robot Cycle Time: {robot_cycle_time_s:.2f} s")

        # Only check the 2D dishwasher camera if handover actually happened!
        # If all grasp attempts failed, the robot aborted handover and returned home,
        # so skipping verification avoids the 5-8s delay of polling an empty slot.
        handover_happened = getattr(client, "last_handover_timestamp", None) is not None
        if handover_happened:
            try:
                time.sleep(0.5)
                placed_ok = False
                while not placed_ok:
                    verify_polls += 1
                    placed_ok = confirm_placement(chosen_slot)
                    if placed_ok is None:
                        break
                    if placed_ok:
                        print(f"[TRAY] Placement verified in slot '{chosen_slot}'.")
                        break
                    print("[TRAY] Waiting for mug to appear in dishwasher...")
                    time.sleep(0.5)
            except Exception as exc:
                print(f"[TRAY] ERROR during placement verification: {exc}")
        else:
            print("[TRAY] Grasp failed (handover aborted) - skipping dishwasher verification.")
            placed_ok = False

        count += 1
        times_sec.append(robot_cycle_time_s)

        if save_protocol and file_path is not None:
            tp.cup_information(
                file_path,
                count,
                target_xyz,
                robot_cycle_time_s,
                confidence=conf,
                label=label,
            )

        if report_logger is not None:
            report_logger.log_cup_event(
                cup_no=count,
                lock_info=lock_info,
                chosen_slot=chosen_slot,
                slot_coords=coords,
                normal_vec=normalized_vector,
                pick_attempts=getattr(client, "pick_attempt_count", 1),
                retry_history=getattr(client, "retry_history", []),
                placed_ok=placed_ok,
                verify_polls=verify_polls,
                left_arm_time_s=getattr(client, "last_left_arm_time_s", robot_cycle_time_s),
                right_arm_time_s=getattr(client, "last_right_arm_time_s", 0.0),
                robot_cycle_time_s=robot_cycle_time_s,
            )

    except Exception as exc:
        print(f"[ROBOT] ERROR in local_move: {exc}")
    finally:
        with lock:
            busy = False
        client.FreeSlot = None


def choose_detection_model():
    """Ask once which copied detection package to load."""
    model_by_input = {
        "5": "v5",
        "v5": "v5",
        "5s": "v5s",
        "v5s": "v5s",
        "8": "v8",
        "v8": "v8",
        "11": "v11z",
        "11z": "v11z",
        "11s": "v11z",
        "v11": "v11z",
        "v11z": "v11z",
        "v11s": "v11z",
    }
    while True:
        choice = input("Run detection model v5, v5s, v8, or v11Z? Enter 5, 5s, 8, or 11: ").strip().lower()
        model = model_by_input.get(choice)
        if model is not None:
            print(f"[MODEL] Selected {model}.")
            return model


def run():
    global busy, times_sec
    model_choice = choose_detection_model()
    file_path = None
    quaternion = [1, 0, 0, 0]
    cam_coords = 'saved_coordinates.txt'
    robot_file = 'robo_coords.txt'
    client = Communication()

    orientation_map = {
        'Back': [0.0, 0.0, 1.0],
        'Front': [0.0, 0.0, -1.0],
        'left_side': [1.0, 0.0, 0.0],
        'right_side': [-1.0, 0.0, 0.0],
        'upright': [0.0, 1.0, 0.0],
        'upside_down': [0.0, -1.0, 0.0],
        'Gripper': [0.0, 0.0, -1.0],
    }

    def find_empty_slot_in_dishwasher():
        """Dynamic callback triggered when RAPID sends 'Ask_LeavePosition'."""
        try:
            if client.FreeSlot is not None:
                print(f"[DYNAMIC TRAY] Returning pre-cached slot coordinates: {client.FreeSlot}")
                return list(client.FreeSlot)
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
        if SLOT_SOURCE == "camera":
            try:
                tray_camera.load_calibrated_positions("slot_positions.json")
            except Exception as exc:
                print(f"[TRAY] WARNING: could not load slot_positions.json: {exc}.")

        homogeneous, syncNN, pipeline, labels, _, _, _, _ = cs.camera_setup(
            cam_coords, robot_file, model_choice
        )
        model_info = cs.load_detection_model(model_choice)

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
        print(f"[HT] active {active_ht_name}. Y unchanged.")
        print(f"[HT] x_compensate: y = 1.401822 * x - 198.521")
        print(f"[HT] Z Heights: Upward={Z_UPWARD} mm, Downward={Z_DOWNWARD} mm")

        # ==================================== REPORT & TEST PROTOCOL SETUP ====================================
        cal_metrics = tp.compute_calibration_metrics(x_comp_fn=x_compensate)

        tested_variable = ""
        save_protocol = input("Do you want to save the test protocol? (y/n): ").lower() == 'y'
        if save_protocol:
            tested_variable = input("What variable was tested? ")

        report_logger = tp.ReportLogger(
            model_name=model_choice,
            model_info=model_info,
            active_ht_name=active_ht_name,
            slot_source=SLOT_SOURCE,
            z_upward=Z_UPWARD,
            z_downward=Z_DOWNWARD,
            grasp_frame_count=GRASP_FRAME_COUNT,
            same_cup_distance_mm=SAME_CUP_DISTANCE_MM,
            cal_metrics=cal_metrics,
            tested_variable=tested_variable,
        )
        print(f"[REPORT] Logging all metrics to: {report_logger.report_txt_path}")

        # ==================================== MAIN LOOP ====================================
        first_run = True
        grasp_samples = []
        prev_dev_ts = None

        try:
            with dai.Device(pipeline) as device:
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

                cv.namedWindow('RGB', cv.WINDOW_NORMAL)

                while True:
                    loop_t0 = time.perf_counter()

                    if SLOT_SOURCE == "camera":
                        _states, tray_img = tray_camera.get_free_slots()
                        if tray_img is not None:
                            cv.imshow("Tray Status (2D)", tray_img)

                    in_rgb = q_rgb.get()
                    in_depth = q_depth.get()
                    in_dets = q_det.get()

                    # Measure OAK-D on-chip inference time & Camera->PC latency for the report log
                    try:
                        cam_to_pc_latency_ms = (
                            dai.Clock.now() - in_dets.getTimestamp()
                        ).total_seconds() * 1000.0
                    except Exception:
                        cam_to_pc_latency_ms = None

                    on_chip_inference_ms = None
                    try:
                        dev_ts = in_dets.getTimestampDevice()
                        if prev_dev_ts is not None:
                            dt_ms = (dev_ts - prev_dev_ts).total_seconds() * 1000.0
                            if 5.0 <= dt_ms <= 300.0:
                                on_chip_inference_ms = dt_ms
                        prev_dev_ts = dev_ts
                    except Exception:
                        pass

                    frame = in_rgb.getCvFrame()
                    detections = in_dets.detections
                    selected_target = select_nearest_target(detections, labels)

                    accept_grasp_sample = not first_run
                    if first_run:
                        time.sleep(1)
                        first_run = False
                        prev_dev_ts = None

                    if busy:
                        grasp_samples = []
                    elif accept_grasp_sample and selected_target is not None:
                        camera_xyz = camera_xyz_of(selected_target)
                        sample_label = labels[selected_target.label]
                        previous_count = len(grasp_samples)
                        mismatch_reason = grasp_sample_mismatch_reason(
                            grasp_samples, sample_label, camera_xyz
                        )
                        if grasp_samples and mismatch_reason is not None:
                            print('[FRAME] Target changed. 5-frame window restarted.')
                            report_logger.record_window_reset(mismatch_reason)
                            grasp_samples = []
                            previous_count = 0

                        if not grasp_samples:
                            report_logger.mark_window_started()

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

                    # Draw bounding boxes and labels
                    for det in detections:
                        label = str(det.label)
                        if det.label < len(labels):
                            label = labels[det.label]

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

                        robot_xyz = robot_xyz_from_relations(
                            coords.x, coords.y, coords.z, homogeneous, label=label
                        )
                        if det is selected_target:
                            with client._mutex_variable:
                                client.LatestMugCoordinates = robot_xyz
                                norm_v = orientation_map.get(label, [0.0, 1.0, 0.0])
                                norm_calc = np.matmul(rotation_matrix, norm_v)
                                if (abs(norm_v[1]) < abs(norm_v[2])) or (abs(norm_v[1]) < abs(norm_v[0])):
                                    norm_calc -= [0, 0, np.dot(norm_calc, [0, 0, 1])]
                                    norm_calc = norm_calc / np.sqrt(np.dot(norm_calc, norm_calc))
                                else:
                                    norm_calc[2] = norm_v[1] / abs(norm_v[1])
                                    norm_calc[1] = 0
                                    norm_calc[0] = 0
                                client.LatestMugNormal = [float(norm_calc[0]), float(norm_calc[1]), float(norm_calc[2])]
                                if not busy:
                                    client.MugCoordinates = robot_xyz

                        cv.putText(frame, f'R: {robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}', (x1 + 5, y1 + 80), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

                        if det is selected_target and grasp_samples:
                            median_xyz = median_grasp_xyz(grasp_samples)
                            median_label = grasp_samples[-1]['label']
                            median_robot = robot_xyz_from_relations(
                                float(median_xyz[0]),
                                float(median_xyz[1]),
                                float(median_xyz[2]),
                                homogeneous,
                                label=median_label,
                            )
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
                    yumi_capture.handle(frame)
                    key = cv.waitKey(1) & 0xFF

                    loop_ms = (time.perf_counter() - loop_t0) * 1000.0
                    if accept_grasp_sample:
                        report_logger.record_frame(
                            detections=detections,
                            labels=labels,
                            selected_target=selected_target,
                            on_chip_inference_ms=on_chip_inference_ms,
                            cam_to_pc_latency_ms=cam_to_pc_latency_ms,
                            loop_ms=loop_ms,
                            is_busy=busy,
                        )

                    # ==========================================
                    # AUTOMATIC PICK-AND-PLACE TRIGGER
                    # ==========================================
                    if not busy and len(grasp_samples) >= GRASP_FRAME_COUNT:
                        samples_snapshot = list(grasp_samples)
                        stacked = np.vstack([sample['xyz'] for sample in samples_snapshot])
                        median_xyz = np.median(stacked, axis=0)
                        spread = np.max(stacked, axis=0) - np.min(stacked, axis=0)
                        label = samples_snapshot[-1]['label']
                        conf = int(np.median([sample['conf'] for sample in samples_snapshot]))
                        grasp_samples = []

                        target_xyz = robot_xyz_from_relations(
                            float(median_xyz[0]),
                            float(median_xyz[1]),
                            float(median_xyz[2]),
                            homogeneous,
                            label=label,
                        )

                        lock_info = report_logger.record_target_lock(
                            grasp_samples=samples_snapshot,
                            median_xyz=median_xyz,
                            spread_xyz=spread,
                            target_robot_xyz=target_xyz,
                            label=label,
                            median_conf=conf,
                        )

                        print(f'[AUTO] Next mug from {GRASP_FRAME_COUNT} frames ({label}, {conf}%)')
                        print(f'[HT] median camera XYZ mm: {median_xyz[0]:.1f}, {median_xyz[1]:.1f}, {median_xyz[2]:.1f}')
                        print(f'[HT] 5-frame range mm: {spread[0]:.1f}, {spread[1]:.1f}, {spread[2]:.1f}')
                        print(f'[HT] robot XYZ mm (compensated X, fixed Z={target_xyz[2]}): {target_xyz}')

                        try:
                            normalized_vector = orientation_map.get(label, [0.0, 1.0, 0.0])
                            norm = np.matmul(rotation_matrix, normalized_vector)
                            if (abs(normalized_vector[1]) < abs(normalized_vector[2])) or (abs(normalized_vector[1]) < abs(normalized_vector[0])):
                                norm -= [0, 0, np.dot(norm, [0, 0, 1])]
                                norm = norm / np.sqrt(np.dot(norm, norm))
                            else:
                                norm[2] = normalized_vector[1] / abs(normalized_vector[1])
                                norm[1] = 0
                                norm[0] = 0

                            threading.Thread(
                                target=local_move,
                                args=(
                                    quaternion,
                                    client,
                                    target_xyz,
                                    [float(norm[0]), float(norm[1]), float(norm[2])],
                                    False,
                                    file_path,
                                    conf,
                                    label,
                                    report_logger,
                                    lock_info,
                                ),
                                daemon=True,
                            ).start()
                            time.sleep(0.5)
                        except Exception as e:
                            print(f"Error {e}")

                    if key == ord('r'):
                        client.connect()
                    if key == ord('q'):
                        break
        finally:
            cv.destroyAllWindows()
            tray_camera.release_camera()
            report_logger.finalize_report()


if __name__ == "__main__":
    run()
