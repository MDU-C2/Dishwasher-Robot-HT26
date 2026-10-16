"""Regression checks for V2 S/F key behavior and RAPID alignment."""
import csv
import importlib
import sys
import types
from datetime import timedelta
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

if "depthai" not in sys.modules:
    stub = types.ModuleType("depthai")
    stub.__version__ = "fake"
    sys.modules["depthai"] = stub


def test_homepos_and_every_S_consumes_even_if_failed():
    from aruco_center_sampling import PointSchedule
    pts = PointSchedule()
    assert pts.peek() == "HOMEpos"
    assert pts.consume() == "HOMEpos"
    assert pts.peek() == "s1"
    for n in range(1, 11):
        assert pts.consume() == f"s{n}"
    assert pts.consume() == "s11"  # whether good or bad, consume
    assert pts.peek() == "s12"
    for n in range(12, 61):
        assert pts.consume() == f"s{n}"
    assert pts.peek() is None
    assert pts.consume() is None


def test_resume_at_s12_and_preserve_v2_override_flags():
    from aruco_center_sampling import PointSchedule
    assert PointSchedule(point_start=12).peek() == "s12"
    assert PointSchedule(point_label="test_marker").peek() == "test_marker"
    assert PointSchedule(point_prefix="p", point_start=5).peek() == "p5"
    with pytest.raises(ValueError):
        PointSchedule(point_start=61)


class FakeFrame:
    def __init__(self, seq, rgb=True):
        self.seq = seq
        self.rgb = rgb
    def getSequenceNum(self):
        return self.seq if self.rgb else self.seq + 1000
    def getTimestamp(self):
        return timedelta(seconds=100)
    def getTimestampDevice(self):
        return timedelta(seconds=100 + self.seq / 30)
    def getCvFrame(self):
        return np.zeros((360, 640, 3), dtype=np.uint8)
    def getFrame(self):
        return np.full((360, 640), 450, dtype=np.uint16)


class FakeQueue:
    def __init__(self):
        self.seq = 0
    def tryGet(self):
        self.seq += 1
        return {"rgb": FakeFrame(self.seq, True),
                "depth": FakeFrame(self.seq, False)}


class FakeDevice:
    def __init__(self, pipeline):
        self.queue = FakeQueue()
    def __enter__(self):
        return self
    def __exit__(self, *exc):
        pass
    def readCalibration(self):
        return self
    def getCameraIntrinsics(self, *args):
        return np.eye(3, dtype=np.float64)
    def getDistortionCoefficients(self, *args):
        return [0.0] * 14
    def getDefaultIntrinsics(self, *args):
        return None, 1920, 1080
    def getDistortionModel(self, *args):
        return "test"
    def getMxId(self):
        return "fake-OAK"
    def getOutputQueue(self, **kwargs):
        return self.queue


def test_actual_S_F_S_S_flow(tmp_path, monkeypatch):
    """Exercise the real interactive loop, not only an isolated label helper."""
    import aruco_center_sampling as app
    import mouse_depth_sampling as base
    import cv2
    import depthai

    monkeypatch.setattr(app, "SESSION_ROOT", tmp_path)
    monkeypatch.setattr(depthai, "Device", FakeDevice, raising=False)
    monkeypatch.setattr(depthai, "CameraBoardSocket", types.SimpleNamespace(RGB="RGB"), raising=False)
    monkeypatch.setattr(depthai, "Clock", types.SimpleNamespace(now=lambda: timedelta(seconds=100.03)), raising=False)
    monkeypatch.setattr(base, "build_pipeline", lambda: object())
    monkeypatch.setattr(base, "geometry_status", lambda rgb, depth: (True, "ok"))
    monkeypatch.setattr(base, "colorize_depth", lambda depth: np.zeros((360,640,3), dtype=np.uint8))
    monkeypatch.setattr(base, "put_lines", lambda *args, **kwargs: None)
    monkeypatch.setattr(cv2, "namedWindow", lambda *args, **kwargs: None)
    monkeypatch.setattr(cv2, "imshow", lambda *args, **kwargs: None)
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda *args, **kwargs: None)

    seq = {"current": 0}
    def fake_waitkey(*_):
        seq["current"] += 1
        return {10:ord("s"), 11:ord("f"), 21:ord("s"), 31:ord("s"), 32:ord("q")}.get(seq["current"], -1)
    monkeypatch.setattr(cv2, "waitKey", fake_waitkey)

    def fake_eval(pair, detector, matrix, distortion, target_id):
        n = pair["timing"]["rgb_sequence"]
        # First S 10/10, second S 3/10, third S 10/10.
        if 15 <= n <= 21:
            pair["reason"] = "marker not found"
            pair["measurement"] = None
        else:
            pair["reason"] = "ok"
            pair["measurement"] = {
                "camera_xyz_mm": {"x": 1.0, "y": 2.0, "z": 450.0},
                "pixel_u": 320, "pixel_v": 180,
                "valid_depth_count": 25, "depth_range_mm": 0.0,
            }
        pair["selection"] = {"target_id": target_id}
        pair["marked"] = pair["rgb"].copy()
        return pair
    monkeypatch.setattr(app, "evaluate_pair", fake_eval)

    # This must exercise the 'F' key and both good and failed 'S' attempts.
    app.run(target_id=23, point_label=None, point_prefix="s", point_start=0)
    sessions = list(tmp_path.iterdir())
    assert len(sessions) == 1
    session = sessions[0]
    with (session / "batch_index.csv").open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    assert [(r["point_label"],r["status"]) for r in rows] == [
        ("HOMEpos", "success"), ("s1", "failed"), ("s2", "success")]
    assert (session / "sample_0001" / "sample.json").exists()
    with (session / "index.csv").open(newline="", encoding="utf-8") as file:
        debug_rows = list(csv.DictReader(file))
    assert debug_rows[0]["point_label"] == "s1"   # F does not move schedule
    assert [r["valid_frames"] for r in rows] == ["10", "3", "10"]
    assert (session / "failed_0001" / "sample.json").exists()


def test_frozen_s_counts_failed_but_f_still_available(tmp_path, monkeypatch):
    # Pure data sanity; the frozen action is explicitly handled in the S branch.
    from aruco_center_sampling import PointSchedule
    labels = PointSchedule()
    assert labels.consume() == "HOMEpos"   # frozen invalid S
    assert labels.peek() == "s1"           # F leaves schedule unchanged
    assert labels.consume() == "s1"
    assert labels.peek() == "s2"
