"""Small, hardware-independent 10-pair rolling buffer for ArUco sampling.

Pairs are *distinct captures*, not distinct XYZ values. Repeated XYZ is allowed.
This module does not talk to a camera or robot and can be unit tested offline.
"""

from collections import deque
import math

import numpy as np

WINDOW_SIZE = 10
MIN_VALID_FRAMES = 5
MAX_FRAME_AGE_S = 0.5


class RecentFrameBuffer:
    """Keep up to 10 latest distinct synchronized-pair packets (good or bad)."""

    def __init__(self, capacity=WINDOW_SIZE):
        self.capacity = capacity
        self._pairs = deque(maxlen=capacity)
        self._last_rgb = None
        self._last_depth = None

    def add(self, pair):
        """Reject repeated sensor frames; numeric XYZ repetition is irrelevant."""
        t = pair["timing"]
        rgb_id = (t["rgb_sequence"], t["rgb_device_timestamp_ms"])
        depth_id = (t["depth_sequence"], t["depth_device_timestamp_ms"])
        if rgb_id == self._last_rgb or depth_id == self._last_depth:
            return False
        # Replayed packets need not be adjacent. Reject reuse within this window.
        for buffered in self._pairs:
            old = buffered["timing"]
            old_rgb = (old["rgb_sequence"], old["rgb_device_timestamp_ms"])
            old_depth = (old["depth_sequence"], old["depth_device_timestamp_ms"])
            if rgb_id == old_rgb or depth_id == old_depth:
                return False
        self._last_rgb, self._last_depth = rgb_id, depth_id
        self._pairs.append(pair)
        return True

    def clear(self):
        """Discard the old window after a successful batch, retaining capture IDs."""
        self._pairs.clear()

    def snapshot(self):
        """The returned list cannot change when later frames arrive."""
        return list(self._pairs)

    def __len__(self):
        return len(self._pairs)


def assess_snapshot(pairs, now_monotonic, max_age_s=MAX_FRAME_AGE_S,
                    min_valid=MIN_VALID_FRAMES, target=WINDOW_SIZE):
    """Validate the newest window without waiting or capturing any new frames.

    Returns (summary, per_frame_records, usable_pairs). The caller can persist
    failed attempts, and must not fit a failed sample into HT calibration.
    """
    records = []
    good = []
    for index, pair in enumerate(pairs):
        capture_at = pair.get("capture_monotonic")
        age = None if capture_at is None else now_monotonic - capture_at
        if age is None or not math.isfinite(age) or age < 0 or age > max_age_s:
            reason = "stale_or_invalid_capture_timestamp"
        elif pair.get("measurement") is None:
            reason = pair.get("reason") or "invalid_camera_measurement"
        else:
            xyz = pair["measurement"]["camera_xyz_mm"]
            values = np.asarray([xyz["x"], xyz["y"], xyz["z"]], dtype=np.float64)
            reason = "ok" if np.isfinite(values).all() else "nonfinite_xyz"
        passed = reason == "ok"
        if passed:
            good.append(pair)
        records.append({
            "index_in_window": index,
            "valid": passed,
            "reason": reason,
            "age_at_snapshot_s": None if age is None or not math.isfinite(age) else float(age),
            "received_at": pair.get("received_at"),
            "timing": pair["timing"],
            "measurement": pair.get("measurement"),
            "aruco_selection": pair.get("selection"),
        })

    n = len(good)
    stats = {"mean_xyz_mm": None, "median_xyz_mm": None, "std_xyz_mm": None,
             "min_xyz_mm": None, "max_xyz_mm": None}
    if n:
        values = np.array([[p["measurement"]["camera_xyz_mm"][k] for k in ("x", "y", "z")]
                           for p in good], dtype=np.float64)
        stats = {
            "mean_xyz_mm": values.mean(axis=0).tolist(),
            "median_xyz_mm": np.median(values, axis=0).tolist(),
            "std_xyz_mm": values.std(axis=0, ddof=1).tolist() if n > 1 else None,
            "min_xyz_mm": values.min(axis=0).tolist(),
            "max_xyz_mm": values.max(axis=0).tolist(),
        }
    summary = {
        "status": "success" if n >= min_valid else "failed",
        "target_frames": target,
        "buffered_frames": len(pairs),
        "valid_frames": n,
        "invalid_frames": len(pairs) - n,
        "minimum_accepted_frames": min_valid,
        "capture_timeout_s": None,
        "measurement_method": "aruco_center_depth_backprojection",
        "std_definition": "sample standard deviation, ddof=1; XYZ axes separately",
        "note": "Independent received frames may legitimately have identical XYZ values.",
        **stats,
    }
    return summary, records, good
