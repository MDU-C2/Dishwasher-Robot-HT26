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
import tray_camera_yolo as tc_yolo
try:
    import yumi_capture
except ImportError:
    yumi_capture = None
from config_loader import load_config

# ==================================== CONFIG & SETUP ====================================
_CFG = load_config()
SLOT_SOURCE = _CFG.get("placement", {}).get("slot_source", "camera")
DEFAULT_TRAY_MODE = _CFG.get("placement", {}).get("tray_mode", "yolo")
FIXED_SLOT_NAME = _CFG.get("placement", {}).get("fixed_slot_name", "slot_1")
FIXED_SLOT_COORDS = _CFG.get("placement", {}).get("fixed_slot_coords", [302.0, -508.0, 248.0])
Z_UPWARD = float(_CFG.get("mug_heights", {}).get("upward_z", 55.0))
Z_DOWNWARD = float(_CFG.get("mug_heights", {}).get("downward_z", 45.0))

# ==================================== THREADING SETUP ====================================
busy = False
count = 0
times_sec = []
lock = threading.RLock()
_slot_lock = threading.RLock()
_cur = None
ACTIVE_TRAY_MODE = DEFAULT_TRAY_MODE


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


def setup_tray(tray_mode=None):
    """Initialize the selected 2D tray detection backend ('yolo' or 'threshold')."""
    mode = (tray_mode or ACTIVE_TRAY_MODE).strip().lower()
    if SLOT_SOURCE != "camera":
        return True

    if mode == "yolo":
        try:
            tc_yolo.load_calibrated_positions("slot_positions.json")
        except Exception as exc:
            print(f"[TRAY] WARNING: could not load slot_positions.json: {exc}.")
        saved = bool(tc_yolo.boxes())
        if not saved:
            print("[TRAY] No saved calibration found.")
        from_gui = os.environ.get("YUMI_GUI") == "1"
        if not saved:
            if not from_gui:
                if not tc_yolo.calibrate() and not saved:
                    return False
            else:
                print("[TRAY WARNING] No slot_config.json found. Use 'CALIBRATE TRAY' in the GUI.")
        tc_yolo.report()
        if not tc_yolo.start():
            print("[TRAY WARNING] Initial 2D tray camera/YOLO scan failed; will retry in background.")
        return True
    else:
        try:
            tray_camera.load_calibrated_positions("slot_positions.json")
        except Exception as exc:
            print(f"[TRAY] WARNING: could not load slot_positions.json: {exc}.")
        tray_camera.start_live_scanner()
        return True


def select_placement_slot(tray_mode=None):
    """Return (slot_name, [x, y, z]) for the right arm, or (None, None)."""
    global _cur
    mode = (tray_mode or ACTIVE_TRAY_MODE).strip().lower()
    match SLOT_SOURCE:
        case "fixed":
            coords = [float(v) for v in FIXED_SLOT_COORDS]
            print(f"[TRAY] Source=fixed  slot '{FIXED_SLOT_NAME}' @ {coords}")
            return FIXED_SLOT_NAME, coords
        case "camera":
            if mode == "yolo":
                with _slot_lock:
                    selected = _cur
                if selected is None:
                    taken = tc_yolo.take()
                    with _slot_lock:
                        if _cur is None:
                            _cur = taken
                        selected = _cur
                if selected is None:
                    return None, None
                print(f"[TRAY] Source=camera (YOLO)  slot '{selected[0]}' @ {selected[1]}")
                return selected[0], list(selected[1])
            else:
                states, _frame = tray_camera.get_free_slots(rescan=True)
                reserved = tray_camera.reserved_slots()
                print(f"[TRAY] Source=camera (Threshold)  states: {states} (reserved: {sorted(reserved)})")
                known_coords = tray_camera._coords()
                chosen = next(
                    (
                        name
                        for name, state in states.items()
                        if state == "free" and name in known_coords and name not in reserved
                    ),
                    None,
                )
                if chosen is None:
                    print("[TRAY] No free calibrated dishwasher slot.")
                    return None, None
                tray_camera.reserve_slot(chosen)
                coords = known_coords[chosen]
                print(f"[TRAY] Source=camera (Threshold)  slot '{chosen}' @ {coords}")
                return chosen, list(coords)
        case _:
            print(f"[TRAY] ERROR: unknown SLOT_SOURCE '{SLOT_SOURCE}'.")
            return None, None


def has_cached_free_slot(tray_mode=None):
    """Fast non-blocking check before starting PickUpSequence to avoid moving on a full tray."""
    mode = (tray_mode or ACTIVE_TRAY_MODE).strip().lower()
    if SLOT_SOURCE != "camera":
        return True
    if mode == "yolo":
        with _slot_lock:
            if _cur is not None:
                return True
        return tc_yolo.has_cached_free_slot()
    return tray_camera.has_cached_free_slot()


def end_slot(name, check, tray_mode=None):
    """Finalize or release a reserved slot in YOLO mode."""
    global _cur
    with _slot_lock:
        _cur = None
    mode = (tray_mode or ACTIVE_TRAY_MODE).strip().lower()
    if SLOT_SOURCE != "camera" or name is None or mode != "yolo":
        return
    if check:
        tc_yolo.done(name)
    else:
        tc_yolo.free(name)


def confirm_placement(slot_name, tray_mode=None):
    """Check the rack after a place (used in synchronous 2D Thresholding mode)."""
    mode = (tray_mode or ACTIVE_TRAY_MODE).strip().lower()
    match SLOT_SOURCE:
        case "fixed":
            print(f"[TRAY] Source=fixed  placement check skipped for '{slot_name}'.")
            return None
        case "camera":
            if mode == "yolo":
                return tc_yolo.verify(slot_name)
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
    """Nearest valid cup detection to the left-entry reference. Mug pose is not used."""
    return cs.select_nearest_target(detections, labels, LEFT_ENTRY_CAMERA_XYZ_MM)


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
    tray_mode=None,
    _preclaimed=False,
):
    """
    Perform pick-and-place transaction in background thread.
    Robot cycle time is measured strictly from when the robot starts moving
    toward the mug until it has left the mug in the dishwasher tray.
    """
    global busy, times_sec, count
    mode = (tray_mode or ACTIVE_TRAY_MODE).strip().lower()

    if not _preclaimed:
        with lock:
            if busy:
                return
            busy = True

    verify_polls = 0
    placed_ok = None
    chosen_slot = None
    coords = None
    finished = False
    slot_ready_event = threading.Event()
    slot_worker = None
    slot_holder = {"slot": None, "coords": None, "error": None}

    def _resolve_slot_async():
        try:
            s_name, s_coords = select_placement_slot(mode)
            slot_holder["slot"] = s_name
            slot_holder["coords"] = s_coords
            if s_name is not None and s_coords is not None:
                client.SetFreeSlot(s_coords)
                print(f"[TRAY] FreeSlot stored: {client.FreeSlot}")
        except Exception as exc:
            slot_holder["error"] = exc
            print(f"[TRAY] ERROR selecting free slot: {exc}")
        finally:
            slot_ready_event.set()

    try:
        client.SetFreeSlot(None)
        client._fallback_slot_info = None
        if has_cached_free_slot(mode):
            client._pending_slot_ready = slot_ready_event
            slot_worker = threading.Thread(target=_resolve_slot_async, daemon=True)
            slot_worker.start()
        else:
            _resolve_slot_async()
            if slot_holder["slot"] is None or slot_holder["coords"] is None:
                print(f"[TRAY] ERROR: No placement slot available ({mode} tray monitor).")
                return

        # ======================================================================
        # PURE ROBOT CYCLE TIME: Start of robot movement -> Mug left in tray
        # ======================================================================
        print("[ROBOT] Starting PickUpSequence...")
        t_robot_start = time.perf_counter()
        client.PickUpSequence(target_xyz, orient, normalized_vector)
        robot_cycle_time_s = time.perf_counter() - t_robot_start
        print(f"[ROBOT] PickUpSequence completed. Robot Cycle Time: {robot_cycle_time_s:.2f} s")

        if slot_worker is not None:
            slot_worker.join(timeout=3.0)
        slot_ready_event.wait(timeout=0.5)
        chosen_slot = slot_holder["slot"]
        coords = slot_holder["coords"]
        if chosen_slot is None or coords is None:
            fb = getattr(client, "_fallback_slot_info", None)
            if fb is not None:
                chosen_slot, coords = fb

        # Only verify placement if handover actually happened!
        # If all 3 grasp attempts failed, the robot aborted handover and returned home.
        handover_happened = getattr(client, "last_handover_timestamp", None) is not None

        if mode == "yolo":
            if handover_happened:
                end_slot(chosen_slot, True, mode)
                finished = True
                placed_ok = True if SLOT_SOURCE == "camera" else None
                verify_polls = 1 if SLOT_SOURCE == "camera" else 0
            else:
                print("[TRAY] Grasp failed (handover aborted) - freeing reserved YOLO slot immediately.")
                end_slot(chosen_slot, False, mode)
                finished = True
                placed_ok = False
        else:
            if handover_happened:
                if SLOT_SOURCE == "camera":
                    placed_ok = True
                    verify_polls = 0
                    tray_camera.verify_async(
                        chosen_slot,
                        on_complete=(
                            lambda s_name, ok, polls, rl=report_logger: rl.update_slot_verification(
                                s_name, ok, verify_polls=polls
                            )
                        )
                        if report_logger is not None
                        else None,
                    )
                else:
                    placed_ok = None
                    verify_polls = 0
            else:
                print("[TRAY] Grasp failed (handover aborted) - freeing slot without verification.")
                if SLOT_SOURCE == "camera" and chosen_slot is not None:
                    tray_camera.drop(chosen_slot)
                placed_ok = False
            finished = True

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
        if slot_worker is not None and slot_worker.is_alive():
            slot_worker.join(timeout=2.0)
            chosen_slot = chosen_slot or slot_holder.get("slot")
        if chosen_slot is None:
            fb = getattr(client, "_fallback_slot_info", None)
            if fb is not None:
                chosen_slot = fb[0]
        if not finished and chosen_slot is not None:
            if mode == "yolo":
                end_slot(chosen_slot, False, mode)
            elif SLOT_SOURCE == "camera":
                tray_camera.drop(chosen_slot)
        with lock:
            busy = False
            client.SetFreeSlot(None)
            client._pending_slot_ready = None
            client._fallback_slot_info = None


def choose_detection_model():
    """Ask once which copied 3D OAK-D detection package to load."""
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


def choose_tray_mode(forced_mode=None):
    """Ask which 2D tray detection method to use ('yolo' or 'threshold')."""
    if forced_mode is not None:
        mode = str(forced_mode).strip().lower()
        print(f"[TRAY MODE] Using {mode.upper()} 2D tray detection.")
        return mode

    mode_by_input = {
        "": "yolo",
        "y": "yolo",
        "yolo": "yolo",
        "1": "yolo",
        "t": "threshold",
        "thresh": "threshold",
        "threshold": "threshold",
        "normal": "threshold",
        "diff": "threshold",
        "2d": "threshold",
        "2": "threshold",
    }
    while True:
        choice = input(
            "Run 2D tray detection with YOLO or Normal Thresholding? Enter y (yolo) or t (threshold) [default=yolo]: "
        ).strip().lower()
        mode = mode_by_input.get(choice)
        if mode is not None:
            label_str = "YOLO (best.pt)" if mode == "yolo" else "Normal 2D Thresholding"
            print(f"[TRAY MODE] Selected {label_str}.")
            return mode


def run(forced_tray_mode=None):
    global busy, times_sec, ACTIVE_TRAY_MODE
    model_choice = choose_detection_model()
    ACTIVE_TRAY_MODE = choose_tray_mode(forced_mode=forced_tray_mode)
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
    }

    def find_empty_slot_in_dishwasher():
        """Dynamic callback triggered when RAPID sends 'Ask_LeavePosition'."""
        try:
            pending_evt = getattr(client, "_pending_slot_ready", None)
            if pending_evt is not None:
                pending_evt.wait(timeout=5.0)
            with client._mutex_variable:
                cached = list(client.FreeSlot) if client.FreeSlot is not None else None
            if cached is not None:
                print(f"[DYNAMIC TRAY] Returning pre-cached slot coordinates: {cached}")
                return cached
            chosen_slot, coords = select_placement_slot(ACTIVE_TRAY_MODE)
            if coords is None:
                print("[DYNAMIC TRAY] WARNING: No placement slot available.")
                return None
            client._fallback_slot_info = (chosen_slot, list(coords))
            client.SetFreeSlot(coords)
            print(f"[DYNAMIC TRAY] Selected slot: '{chosen_slot}' @ {coords}")
            return coords
        except Exception as exc:
            print(f"[DYNAMIC TRAY ERROR] {exc}")
            return None

    client.get_free_slot_logic = find_empty_slot_in_dishwasher

    if SLOT_SOURCE == "camera" and not setup_tray(ACTIVE_TRAY_MODE):
        if ACTIVE_TRAY_MODE == "yolo":
            tc_yolo.close()
        else:
            tray_camera.release_camera()
        return

    if client.connectV2():
        client.MoveHome()
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

        tray_source_desc = (
            f"{SLOT_SOURCE} ({ACTIVE_TRAY_MODE.upper()})"
            if SLOT_SOURCE == "camera"
            else SLOT_SOURCE
        )
        report_logger = tp.ReportLogger(
            model_name=model_choice,
            model_info=model_info,
            active_ht_name=active_ht_name,
            slot_source=tray_source_desc,
            z_upward=Z_UPWARD,
            z_downward=Z_DOWNWARD,
            grasp_frame_count=GRASP_FRAME_COUNT,
            same_cup_distance_mm=SAME_CUP_DISTANCE_MM,
            cal_metrics=cal_metrics,
            tested_variable=tested_variable,
        )
        if ACTIVE_TRAY_MODE == "yolo":
            tc_yolo.on_verify_callback = report_logger.update_slot_verification
        print(f"[REPORT] Logging all metrics to: {report_logger.report_txt_path}")

        # ==================================== MAIN LOOP ====================================
        first_run = True
        grasp_samples = []
        prev_dev_ts = None
        tray_win_name = "Tray Status (YOLO)" if ACTIVE_TRAY_MODE == "yolo" else "Tray Status (2D)"

        try:
            with dai.Device(pipeline) as device:
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

                cv.namedWindow('RGB', cv.WINDOW_NORMAL)

                while True:
                    loop_t0 = time.perf_counter()

                    if SLOT_SOURCE == "camera":
                        tray_img = (
                            tc_yolo.view()
                            if ACTIVE_TRAY_MODE == "yolo"
                            else tray_camera.view()
                        )
                        if tray_img is not None:
                            cv.imshow(tray_win_name, tray_img)

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
                    detections = cs.extract_detections(in_dets, in_depth, device, model_info)
                    selected_target = select_nearest_target(detections, labels)

                    accept_grasp_sample = not first_run
                    if first_run:
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
                            print(f'[FRAME] Target changed. {GRASP_FRAME_COUNT}-frame window restarted.')
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
                            print(f'[FRAME] {GRASP_FRAME_COUNT}/{GRASP_FRAME_COUNT} ready. Automatic pick will send the median.')

                    # Draw bounding boxes and labels
                    for det in detections:
                        label = str(det.label)
                        if 0 <= det.label < len(labels):
                            label = labels[det.label]
                        if not cs.is_ignored_detection_label(label):
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
                    if yumi_capture is not None and hasattr(yumi_capture, "handle"):
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
                        print(f'[HT] {GRASP_FRAME_COUNT}-frame range mm: {spread[0]:.1f}, {spread[1]:.1f}, {spread[2]:.1f}')
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

                            claimed = False
                            with lock:
                                if not busy:
                                    busy = True
                                    claimed = True
                            if claimed:
                                try:
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
                                            ACTIVE_TRAY_MODE,
                                            True,
                                        ),
                                        daemon=True,
                                    ).start()
                                except Exception:
                                    with lock:
                                        busy = False
                                    raise
                        except Exception as e:
                            print(f"Error {e}")

                    if key == ord('r'):
                        client.connect()
                    if key == ord('q'):
                        break
        finally:
            cv.destroyAllWindows()
            if ACTIVE_TRAY_MODE == "yolo":
                tc_yolo.close()
            else:
                tray_camera.release_camera()
            report_logger.finalize_report()


if __name__ == "__main__":
    run()
