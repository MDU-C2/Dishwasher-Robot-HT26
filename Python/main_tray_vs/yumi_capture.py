"""
yumi_capture.py - snapshot / recording / live-view helper for main_tray.py, driven by the YuMi control panel.

In main_tray.py add three things:
    import yumi_capture                      # with the other imports at the top
    yumi_capture.handle(frame)               # right after   cv.imshow('RGB', frame)
    yumi_capture.check_stop()                # in the "no frames yet" branch that replaces the blocking q.get() calls

The panel's buttons just create small flag files in yumi_captures/. This module sees them and saves the real
(annotated, full-resolution) frame:
    snap_YYYYmmdd_HHMMSS.png     one picture per Snapshot click
    rec_YYYYmmdd_HHMMSS.avi      video while Record is on (MJPG: survives an unclean exit)
    live.jpg                     small JPEG refreshed ~8x/second, ONLY while the phone page is being watched
    .heartbeat                   touched once a second so the panel can tell when the camera loop freezes
Stop request: when the panel wants vision to quit (restart / stop) it creates .stop; handle() then raises
SystemExit, which leaves the `with dai.Device(...)` block cleanly so the camera is released.
"""
import datetime
import os
import time

import cv2 as cv

_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "yumi_captures")
_SNAP = os.path.join(_DIR, ".snapshot_request")
_REC = os.path.join(_DIR, ".recording")
_LIVE_FLAG = os.path.join(_DIR, ".live")
_LIVE_JPG = os.path.join(_DIR, "live.jpg")
_STOP = os.path.join(_DIR, ".stop")
_BEAT = os.path.join(_DIR, ".heartbeat")

FPS = 15.0                      # video frame rate (writes are throttled to this)
LIVE_WIDTH = 640                # phone stream width in pixels (smaller = faster over Wi-Fi)
LIVE_QUALITY = 60               # phone stream JPEG quality 1-100
LIVE_INTERVAL = 0.12            # seconds between live frames (0.12 = ~8 fps)

_writer = None
_last_write = 0.0
_last_live = 0.0
_last_beat = 0.0
_failed = False


def _stamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def _watched():
    """True while the web server has touched .live within the last 3 seconds."""
    try:
        return time.time() - os.path.getmtime(_LIVE_FLAG) < 3.0
    except OSError:
        return False


def check_stop():
    """Raise SystemExit if the panel asked vision to quit. Call it wherever the loop may be waiting for frames,
    so a stalled camera can still be shut down cleanly (leaving the `with dai.Device(...)` block releases it)."""
    global _writer
    if os.path.exists(_STOP):
        try:
            os.remove(_STOP)
        except OSError:
            pass
        if _writer is not None:
            try:
                _writer.release()
            except Exception:
                pass
            _writer = None
        print("[CAPTURE] stop requested by the panel - closing the camera cleanly", flush=True)
        raise SystemExit(0)


def handle(frame):
    """Call once per displayed frame. Never raises, so it cannot break the vision loop."""
    global _writer, _last_write, _last_live, _last_beat, _failed
    try:
        now = time.time()

        # --- clean shutdown requested by the panel (SystemExit is not caught by "except Exception" below) ---
        check_stop()

        # --- heartbeat: lets the panel notice a frozen camera loop ---
        if now - _last_beat >= 1.0:
            _last_beat = now
            try:
                with open(_BEAT, "w"):
                    pass
            except OSError:
                pass

        # --- live view for the phone page ---
        if now - _last_live >= LIVE_INTERVAL and _watched():
            _last_live = now
            small = frame
            h, w = frame.shape[:2]
            if w > LIVE_WIDTH:
                small = cv.resize(frame, (LIVE_WIDTH, int(h * LIVE_WIDTH / w)), interpolation=cv.INTER_AREA)
            ok, buf = cv.imencode(".jpg", small, [int(cv.IMWRITE_JPEG_QUALITY), LIVE_QUALITY])
            if ok:
                tmp = _LIVE_JPG + ".tmp"
                with open(tmp, "wb") as fh:
                    fh.write(buf.tobytes())
                try:
                    os.replace(tmp, _LIVE_JPG)      # may fail for a moment while the server reads it: skip frame
                except OSError:
                    pass

        # --- snapshot ---
        if os.path.exists(_SNAP):
            try:
                os.remove(_SNAP)
            except OSError:
                pass
            path = os.path.join(_DIR, f"snap_{_stamp()}.png")
            cv.imwrite(path, frame)
            print(f"[CAPTURE] snapshot saved: {path}", flush=True)

        # --- recording ---
        if os.path.exists(_REC):
            if _writer is None and not _failed:
                h, w = frame.shape[:2]
                path = os.path.join(_DIR, f"rec_{_stamp()}.avi")
                _writer = cv.VideoWriter(path, cv.VideoWriter_fourcc(*"MJPG"), FPS, (w, h))
                if _writer.isOpened():
                    print(f"[CAPTURE] recording to {path}", flush=True)
                else:
                    print("[CAPTURE] ERROR: could not open video writer", flush=True)
                    _writer, _failed = None, True
            if _writer is not None and now - _last_write >= 1.0 / FPS:
                _writer.write(frame)
                _last_write = now
        else:
            _failed = False
            if _writer is not None:
                _writer.release()
                _writer = None
                print("[CAPTURE] recording saved", flush=True)
    except Exception as exc:
        print(f"[CAPTURE] error: {exc}", flush=True)
