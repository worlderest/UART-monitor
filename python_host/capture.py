from __future__ import annotations

import argparse
import logging
import socket
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import serial

from python_host.alignment import PacketAligner
from python_host.protocol import SERIAL_PACKET_SIZE, SerialPacketStreamParser, pack_score_event, pack_udp_frame
from python_host.scoring import DEFAULT_SCORING_CONFIG_PATH, ScorePipeline, load_scoring_config
from python_host.storage import BackgroundStorageWriter


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture STM32 quaternion packets and forward them over UDP.")
    parser.add_argument("--port", required=True, help="Serial port name, for example COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--udp-host", default="127.0.0.1", help="UDP target host")
    parser.add_argument("--udp-port", type=int, default=5005, help="UDP target port")
    parser.add_argument("--score-udp-port", type=int, default=5006, help="UDP target port for score events")
    parser.add_argument(
        "--scoring-config",
        type=Path,
        default=DEFAULT_SCORING_CONFIG_PATH,
        help="Scoring JSON config path",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Capture output directory. Defaults to captures/<start_timestamp>",
    )
    return parser


def default_output_dir() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return Path("captures") / timestamp


@dataclass(slots=True)
class CaptureStats:
    serial_packets: int = 0
    invalid_chunks: int = 0
    aligned_frames: int = 0
    udp_frames: int = 0
    score_events: int = 0


class CaptureService:
    def __init__(
        self,
        port: str,
        baud: int,
        udp_host: str,
        udp_port: int,
        score_udp_port: int,
        out_dir: Path,
        scoring_config_path: Path = DEFAULT_SCORING_CONFIG_PATH,
    ) -> None:
        self.port = port
        self.baud = baud
        self.udp_target = (udp_host, udp_port)
        self.score_udp_target = (udp_host, score_udp_port)
        self.out_dir = out_dir
        self.stats = CaptureStats()
        self.error: str | None = None
        self.stop_event = threading.Event()

        self._parser = SerialPacketStreamParser()
        self._aligner = PacketAligner(stale_timeout_ms=40)
        self._storage = BackgroundStorageWriter(out_dir=out_dir)
        self._score_pipeline = ScorePipeline(load_scoring_config(scoring_config_path))
        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._serial_thread = threading.Thread(target=self._serial_loop, name="serial-capture", daemon=True)

    def start(self) -> None:
        self._storage.start()
        self._serial_thread.start()

    def wait(self) -> None:
        while self._serial_thread.is_alive():
            self._serial_thread.join(timeout=0.5)

    def stop(self) -> None:
        self.stop_event.set()
        if self._serial_thread.is_alive():
            self._serial_thread.join(timeout=2.0)
        self._udp_socket.close()
        self._storage.close()

    def _serial_loop(self) -> None:
        logging.info("Opening serial port %s at %d baud", self.port, self.baud)
        try:
            with serial.Serial(self.port, self.baud, timeout=0.01) as serial_port:
                logging.info("Writing captures into %s", self.out_dir.resolve())
                logging.info("Forwarding aligned frames to udp://%s:%d", *self.udp_target)
                logging.info("Forwarding score events to udp://%s:%d", *self.score_udp_target)
                while not self.stop_event.is_set():
                    chunk = serial_port.read(serial_port.in_waiting or SERIAL_PACKET_SIZE)
                    if not chunk:
                        self._handle_ready_frames(self._aligner.poll())
                        continue

                    received_monotonic = time.monotonic()
                    received_timestamp_ms = time.time_ns() // 1_000_000
                    packets = self._parser.feed(chunk, received_monotonic, received_timestamp_ms)

                    for packet in packets:
                        self.stats.serial_packets += 1
                        self._storage.enqueue_raw(packet.raw_bytes)
                        self._handle_ready_frames(self._aligner.ingest(packet))
        except serial.SerialException as exc:
            self.error = str(exc)
            logging.exception("Serial capture stopped because the port could not be read")
            self.stop_event.set()
        except Exception as exc:
            self.error = str(exc)
            logging.exception("Unexpected error in serial capture loop")
            self.stop_event.set()

    def _handle_ready_frames(self, frames) -> None:
        for frame in frames:
            self.stats.aligned_frames += 1
            self._storage.enqueue_frame(frame)
            self._udp_socket.sendto(pack_udp_frame(frame), self.udp_target)
            self.stats.udp_frames += 1
            for score_event in self._score_pipeline.ingest(frame):
                self._storage.enqueue_score(score_event)
                self._udp_socket.sendto(pack_score_event(score_event), self.score_udp_target)
                self.stats.score_events += 1


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    out_dir = args.out_dir if args.out_dir is not None else default_output_dir()
    service = CaptureService(
        port=args.port,
        baud=args.baud,
        udp_host=args.udp_host,
        udp_port=args.udp_port,
        score_udp_port=args.score_udp_port,
        out_dir=out_dir,
        scoring_config_path=args.scoring_config,
    )

    service.start()
    try:
        service.wait()
    except KeyboardInterrupt:
        logging.info("Stopping capture...")
    finally:
        service.stop()
        logging.info(
            "Stopped. serial_packets=%d aligned_frames=%d udp_frames=%d score_events=%d",
            service.stats.serial_packets,
            service.stats.aligned_frames,
            service.stats.udp_frames,
            service.stats.score_events,
        )
    return 1 if service.error else 0


if __name__ == "__main__":
    raise SystemExit(main())
