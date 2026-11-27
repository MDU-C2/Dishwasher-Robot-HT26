import os
from datetime import datetime
from statistics import mean, stdev


def create_today_textfile():
    today_str = datetime.now().strftime("%Y-%m-%d_%H.%M")
    folder = "Test_Protocols"
    os.makedirs(folder, exist_ok=True)
    filename = os.path.join(folder, f"{today_str}.txt")
    if not os.path.exists(filename):
        with open(filename, "w") as f:
            f.write(f"File created on {today_str}\n")
    return filename


def fill_meta_data(file_path, ht_name="fixed HT", rotation_matrix=None, translation_vector=None):
    """Record the transform actually used, without reading legacy point files."""
    with open(file_path, "a") as f:
        f.write("---------- Meta Data ----------\n")
        f.write(f"Coordinate transform: {ht_name} (fixed in main program)\n")
        if rotation_matrix is not None:
            f.write(f"Rotation matrix: {rotation_matrix.tolist()}\n")
        if translation_vector is not None:
            f.write(f"Translation vector (mm): {translation_vector.tolist()}\n")
        f.write("Robot X compensation: 1.4018216887159423 * X - 198.52114324095436\n")
        f.write("Legacy calibration-point RMS: not evaluated\n")


def ask_user(file_path, start_end):
    if start_end == "start":
        response = input("What variable was tested? ")
        with open(file_path, "a") as f:
            f.write("---------- Test Information ----------\n")
            f.write(f"Tested Variable: {response}\n")
    else:
        response = input("How did the test go? ")
        with open(file_path, "a") as f:
            f.write("---------- Test Conclusion ----------\n")
            f.write(f"Test Conclusion: {response}\n")


def cup_information(file_path, count, cam_coords, process_time, confidence=None, label=None):
    with open(file_path, "a") as f:
        f.write(f"------- Cup {count} Coordinates -------\n")
        if label is not None:
            f.write(f"Detection Label: {label}\n")
        if confidence is not None:
            f.write(f"Detection Confidence: {confidence:.1f}%\n")
        f.write(f"Camera Coordinates: {cam_coords}\n")
        f.write(f"Time taken for pick and place: {process_time:.3f} seconds\n\n")


def log_time_summary(file_path, times_sec):
    if not times_sec:
        return
    avg = mean(times_sec)
    sd  = stdev(times_sec) if len(times_sec) > 1 else 0.0
    with open(file_path, "a") as f:
        f.write("---------- Timing Summary ----------\n")
        f.write(f"Trials: {len(times_sec)}\n")
        f.write(f"Average pick/place time: {avg:.3f} s\n")
        f.write(f"Std dev: {sd:.3f} s\n\n")


def log_rms_error(file_path, rms_error):
    with open(file_path, "a") as f:
        f.write("---------- Calibration Accuracy ----------\n")
        f.write(f"RMS alignment error: {rms_error:.3f} mm\n\n")
