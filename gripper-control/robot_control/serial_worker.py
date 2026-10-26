"""Serial I/O runs in a QThread so the desktop window stays responsive."""

from __future__ import annotations

import queue
import threading
import time

import serial
from PySide6.QtCore import QThread, Signal

from .config import BAUD_RATE


class SerialWorker(QThread):
    port_opened = Signal(str)
    ready = Signal()
    received = Signal(str)
    failed = Signal(str)

    def __init__(self, port: str, parent=None):
        super().__init__(parent)
        self.port = port
        self._running = threading.Event()
        self._running.set()
        self._outbox: queue.Queue[str] = queue.Queue()

    def send(self, command: str) -> None:
        """Thread-safe; the worker owns the actual serial device."""
        if self._running.is_set():
            self._outbox.put(command)

    def stop(self, final_commands: tuple[str, ...] = ()) -> None:
        # Discard stale queued movement before adding the final safety stop.
        # The run loop below drains these final commands before closing the
        # port, even though the normal running flag is cleared.
        with self._outbox.mutex:
            self._outbox.queue.clear()
        for command in final_commands:
            self._outbox.put(command)
        self._running.clear()

    def run(self) -> None:
        try:
            with serial.Serial(
                self.port,
                BAUD_RATE,
                timeout=0.05,
                write_timeout=0.25,
            ) as device:
                self.port_opened.emit(self.port)
                is_ready = False
                last_hello = 0.0
                while self._running.is_set() or not self._outbox.empty():
                    # Opening the USB serial port may reset an Arduino.
                    # Keep requesting READY until the board has booted.
                    now = time.monotonic()
                    if not is_ready and now - last_hello >= 0.8:
                        device.write(b"HELLO\n")
                        last_hello = now

                    try:
                        while True:
                            message = self._outbox.get_nowait()
                            if is_ready:
                                device.write(message.encode("ascii"))
                    except queue.Empty:
                        pass

                    if not self._running.is_set() and self._outbox.empty():
                        break
                    raw = device.readline() if self._running.is_set() else b""
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    self.received.emit(line)
                    if line.startswith("READY,"):
                        is_ready = True
                        self.ready.emit()
        except (serial.SerialException, OSError) as exc:
            if self._running.is_set():
                self.failed.emit(str(exc))
        finally:
            self._running.clear()
