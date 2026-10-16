"""Offline tests for sampling logic and disk persistence; no OAK needed."""
import importlib
import sys
import types
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parents[1]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from recent_frame_buffer import RecentFrameBuffer, assess_snapshot


def pair(i, xyz=(10.0, 20.0, 450.0), at=100.0, reason=None):
    m = (None if reason else {
        "camera_xyz_mm": dict(zip(("x", "y", "z"), xyz)),
        "pixel_u": 320, "pixel_v": 180, "valid_depth_count": 25,
        "depth_range_mm": 0.0,
    })
    return {
        "timing": {
            "rgb_sequence": i, "depth_sequence": i+100,
            "rgb_device_timestamp_ms": float(i*33.33),
            "depth_device_timestamp_ms": float(i*33.33+5),
            "device_timestamp_delta_ms": 5.0,
        },
        "measurement": m,
        "reason": reason or "ok",
        "capture_monotonic": at,
        "received_at": "2026-10-12T12:00:00+02:00",
        "selection": {"target_id": 23},
        "rgb": np.zeros((360, 640, 3), dtype=np.uint8),
        "depth": np.ones((360, 640), dtype=np.uint16)*450,
        "marked": np.zeros((360, 640, 3), dtype=np.uint8),
    }


def test_same_xyz_in_distinct_frames_is_valid():
    buffer = RecentFrameBuffer()
    for i in range(10):
        assert buffer.add(pair(i))
    summary, records, good = assess_snapshot(buffer.snapshot(), 100.1)
    assert summary["status"] == "success"
    assert summary["valid_frames"] == 10
    assert summary["mean_xyz_mm"] == [10, 20, 450]
    assert summary["std_xyz_mm"] == [0, 0, 0]
    assert len(good) == 10 and all(x["valid"] for x in records)


def test_duplicate_sensor_frame_is_rejected_even_if_xyz_changes():
    buffer = RecentFrameBuffer()
    assert buffer.add(pair(5))
    assert not buffer.add(pair(5, xyz=(0, 1, 2)))
    assert len(buffer) == 1


def test_rolling_latest_ten_and_reset_after_success():
    buffer = RecentFrameBuffer()
    for i in range(16):
        buffer.add(pair(i))
    assert len(buffer) == 10
    assert buffer.snapshot()[0]["timing"]["rgb_sequence"] == 6
    buffer.clear()
    assert len(buffer) == 0
    assert not buffer.add(pair(15))
    assert buffer.add(pair(16))


def test_five_to_ten_are_successes_and_four_fails():
    for n in [4, 5, 7, 9, 10]:
        buffer = RecentFrameBuffer()
        for i in range(10):
            buffer.add(pair(i, reason="no marker" if i >= n else None))
        summary, records, _ = assess_snapshot(buffer.snapshot(), 100.1)
        assert summary["valid_frames"] == n
        assert summary["status"] == ("success" if n >= 5 else "failed")
        assert summary["invalid_frames"] == 10 - n
        assert len(records) == 10


def test_stale_frames_excluded_even_with_valid_xyz():
    buffer = RecentFrameBuffer()
    for i in range(10):
        buffer.add(pair(i, at=99.0 if i < 7 else 100.0))
    summary, _, _ = assess_snapshot(buffer.snapshot(), 100.1)
    assert summary["valid_frames"] == 3
    assert summary["status"] == "failed"


def test_persistence_and_single_frame(tmp_path, monkeypatch):
    # Hardware-independent test for the real camera-script persistence functions.
    if "depthai" not in sys.modules:
        stub = types.ModuleType("depthai")
        stub.__version__ = "mock"
        monkeypatch.setitem(sys.modules, "depthai", stub)
    module = importlib.import_module("aruco_center_sampling")
    records_input = [pair(i, reason="no marker" if i == 2 else None) for i in range(10)]
    summary, records, good = assess_snapshot(records_input, 100.1)
    path = module.save_batch(tmp_path, records_input, summary, records, good, {"target_id":23}, "s1")
    assert path.exists()
    import json
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["point_label"] == "s1"
    assert data["summary"]["valid_frames"] == 9
    assert len(data["frames"]) == 10
    assert (path.parent/"representative_rgb.png").exists()
    assert (tmp_path/"batch_index.csv").exists()
    # Failed batches remain in the chronological index to preserve RAPID alignment.
    fail_pairs = [pair(i+50, reason="no marker" if i >= 3 else None) for i in range(10)]
    fail_summary, fail_records, fail_good = assess_snapshot(fail_pairs, 100.1)
    fail_path = module.save_batch(tmp_path, fail_pairs, fail_summary,
                                  fail_records, fail_good, {}, "s2")
    assert fail_path.parent.name == "failed_0001"
    assert ",s2,failed," in (tmp_path/"batch_index.csv").read_text()
    single_path = module.save_sample(tmp_path, records_input[0], {"target_id":23}, "s2")
    assert single_path.exists()
    assert (tmp_path/"index.csv").exists()


def test_five_frames_only_in_window_are_acceptable():
    window = [pair(i) for i in range(5)]
    summary, rows, good = assess_snapshot(window, 100.1)
    assert summary["status"] == "success"
    assert summary["buffered_frames"] == 5
    assert len(rows) == len(good) == 5


def test_marker_size_updated_in_rgb_module(monkeypatch):
    if "depthai" not in sys.modules:
        stub = types.ModuleType("depthai")
        stub.__version__ = "mock"
        monkeypatch.setitem(sys.modules, "depthai", stub)
    rgb = importlib.import_module("aruco_rgb")
    assert rgb.MARKER_LENGTH_MM == 23.5
    assert rgb.MARKER_LENGTH_IS_APPROXIMATE is False


def test_all_frame_images_optional(tmp_path, monkeypatch):
    if "depthai" not in sys.modules:
        stub = types.ModuleType("depthai")
        stub.__version__ = "mock"
        monkeypatch.setitem(sys.modules, "depthai", stub)
    module = importlib.import_module("aruco_center_sampling")
    window = [pair(i) for i in range(6)]
    summary, records, good = assess_snapshot(window, 100.1)
    outfile = module.save_batch(tmp_path, window, summary, records, good, {}, "s11", save_all_frames=True)
    assert outfile.exists()
    assert (outfile.parent/"frame_05_rgb.png").exists()
    assert (outfile.parent/"frame_05_depth.npy").exists()
    assert (outfile.parent/"representative_depth.npy").exists()


def test_nonadjacent_replayed_frames_are_not_counted():
    buf = RecentFrameBuffer()
    assert buf.add(pair(1))
    assert buf.add(pair(2))
    assert not buf.add(pair(1))
    assert len(buf) == 2
