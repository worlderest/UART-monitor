from __future__ import annotations

import csv
import queue
import threading
import time
from pathlib import Path
from typing import Literal

from python_host.protocol import AlignedFrame, CSV_HEADER

StorageEventType = Literal["raw", "frame", "stop"]


class BackgroundStorageWriter:
    def __init__(self, out_dir: Path) -> None:
        self.out_dir = out_dir
        self.raw_path = self.out_dir / "raw.bin"
        self.csv_path = self.out_dir / "aligned.csv"
        self._queue: queue.Queue[tuple[StorageEventType, bytes | AlignedFrame | None]] = queue.Queue()
        self._thread = threading.Thread(target=self._worker, name="storage-writer", daemon=True)
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self._thread.start()
        self._started = True

    def enqueue_raw(self, raw_packet: bytes) -> None:
        self._queue.put(("raw", raw_packet))

    def enqueue_frame(self, frame: AlignedFrame) -> None:
        self._queue.put(("frame", frame))

    def close(self) -> None:
        if not self._started:
            return
        self._queue.put(("stop", None))
        self._thread.join()
        self._started = False

    def _worker(self) -> None:
        flush_deadline = time.monotonic() + 0.5
        with self.raw_path.open("ab") as raw_file, self.csv_path.open("a", newline="", encoding="utf-8") as csv_file:
            csv_writer = csv.writer(csv_file)
            if csv_file.tell() == 0:
                csv_writer.writerow(CSV_HEADER)

            while True:
                timeout = max(0.0, flush_deadline - time.monotonic())
                try:
                    event_type, payload = self._queue.get(timeout=timeout)
                except queue.Empty:
                    raw_file.flush()
                    csv_file.flush()
                    flush_deadline = time.monotonic() + 0.5
                    continue

                if event_type == "stop":
                    raw_file.flush()
                    csv_file.flush()
                    break

                if event_type == "raw":
                    assert isinstance(payload, bytes)
                    raw_file.write(payload)
                elif event_type == "frame":
                    assert isinstance(payload, AlignedFrame)
                    csv_writer.writerow(payload.csv_row())

                if time.monotonic() >= flush_deadline:
                    raw_file.flush()
                    csv_file.flush()
                    flush_deadline = time.monotonic() + 0.5
