from __future__ import annotations

import argparse
import json
import logging
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import serial.tools.list_ports

from python_host.capture import CaptureService, default_output_dir

DEFAULT_CONFIG_PATH = Path("config") / "launcher.json"
DEFAULT_BAUD = 460800
DEFAULT_UDP_HOST = "127.0.0.1"
DEFAULT_UDP_PORT = 5005
DEFAULT_SCORE_UDP_PORT = 5006
DEFAULT_VISUALIZER = Path("build") / "upper_monitor_visualizer.exe"
DEFAULT_SCORING_CONFIG = Path("config") / "scoring.json"
BAUD_CHOICES = (115200, 230400, 460800, 921600)


@dataclass(slots=True)
class LauncherConfig:
    port: str
    baud: int
    udp_host: str
    udp_port: int
    score_udp_port: int
    visualizer: Path
    out_dir: Path | None
    scoring_config: Path


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Start the serial capture and Raylib visualizer together.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="Launcher JSON config path")
    parser.add_argument("--port", default=None, help="Serial port name, for example COM5")
    parser.add_argument("--baud", type=int, default=None, help="Serial baud rate")
    parser.add_argument("--choose-baud", action="store_true", help="Choose baud rate from an interactive menu")
    parser.add_argument("--udp-host", default=None, help="UDP target host")
    parser.add_argument("--udp-port", type=int, default=None, help="UDP target port")
    parser.add_argument("--score-udp-port", type=int, default=None, help="UDP target port for score events")
    parser.add_argument("--out-dir", type=Path, default=None, help="Capture output directory")
    parser.add_argument("--scoring-config", type=Path, default=None, help="Scoring JSON config path")
    parser.add_argument("--visualizer", type=Path, default=None, help="Raylib visualizer executable path")
    parser.add_argument("--no-visualizer", action="store_true", help="Run capture only without opening Raylib")
    return parser


def load_config_file(config_path: Path) -> dict[str, Any]:
    if not config_path.exists():
        logging.warning("Config file %s does not exist; using defaults", config_path)
        return {}

    try:
        with config_path.open("r", encoding="utf-8") as config_file:
            data = json.load(config_file)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {config_path}: {exc}") from exc

    if not isinstance(data, dict):
        raise ValueError(f"Config file {config_path} must contain a JSON object")
    return data


def _config_str(config_data: dict[str, Any], key: str, default: str) -> str:
    value = config_data.get(key, default)
    if value is None:
        return default
    return str(value)


def _config_int(config_data: dict[str, Any], key: str, default: int) -> int:
    value = config_data.get(key, default)
    if value in (None, ""):
        return default
    return int(value)


def merge_config(args: argparse.Namespace, config_data: dict[str, Any]) -> LauncherConfig:
    port = args.port if args.port is not None else _config_str(config_data, "port", "")
    baud = args.baud if args.baud is not None else _config_int(config_data, "baud", DEFAULT_BAUD)
    udp_host = args.udp_host if args.udp_host is not None else _config_str(config_data, "udp_host", DEFAULT_UDP_HOST)
    udp_port = args.udp_port if args.udp_port is not None else _config_int(config_data, "udp_port", DEFAULT_UDP_PORT)
    score_udp_port = (
        args.score_udp_port if args.score_udp_port is not None else _config_int(config_data, "score_udp_port", DEFAULT_SCORE_UDP_PORT)
    )

    visualizer_value = args.visualizer if args.visualizer is not None else Path(_config_str(config_data, "visualizer", str(DEFAULT_VISUALIZER)))
    scoring_config_value = (
        args.scoring_config if args.scoring_config is not None else Path(_config_str(config_data, "scoring_config", str(DEFAULT_SCORING_CONFIG)))
    )
    out_dir_value = args.out_dir
    if out_dir_value is None:
        config_out_dir = _config_str(config_data, "out_dir", "")
        out_dir_value = Path(config_out_dir) if config_out_dir else None

    return LauncherConfig(
        port=port.strip(),
        baud=baud,
        udp_host=udp_host.strip(),
        udp_port=udp_port,
        score_udp_port=score_udp_port,
        visualizer=visualizer_value,
        out_dir=out_dir_value,
        scoring_config=scoring_config_value,
    )


def choose_serial_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports detected. Connect the STM32 board or pass --port COMx.")

    print("Available serial ports:")
    for index, port_info in enumerate(ports, start=1):
        description = port_info.description or "no description"
        hardware_id = port_info.hwid or "no hwid"
        print(f"  {index}. {port_info.device} - {description} ({hardware_id})")

    while True:
        choice = input(f"Select port [1-{len(ports)}]: ").strip()
        if not choice:
            selected = ports[0].device
            print(f"Using default port: {selected}")
            return selected
        if choice.isdigit():
            index = int(choice)
            if 1 <= index <= len(ports):
                return ports[index - 1].device
        print("Invalid selection, please try again.")


def choose_baud_rate(current_baud: int) -> int:
    print("Available baud rates:")
    for index, baud in enumerate(BAUD_CHOICES, start=1):
        marker = " (recommended)" if baud == DEFAULT_BAUD else ""
        current = " [current]" if baud == current_baud else ""
        print(f"  {index}. {baud}{marker}{current}")

    while True:
        choice = input(f"Select baud [default {current_baud}]: ").strip()
        if not choice:
            return current_baud
        if choice.isdigit():
            index = int(choice)
            if 1 <= index <= len(BAUD_CHOICES):
                return BAUD_CHOICES[index - 1]
        print("Invalid selection, please try again.")


def resolve_runtime_config(args: argparse.Namespace) -> LauncherConfig:
    config_data = load_config_file(args.config)
    config = merge_config(args, config_data)

    if not config.port:
        config.port = choose_serial_port()
    if args.choose_baud:
        config.baud = choose_baud_rate(config.baud)
    if config.out_dir is None:
        config.out_dir = default_output_dir()

    return config


def start_visualizer(visualizer_path: Path, udp_port: int, score_udp_port: int) -> subprocess.Popen[bytes]:
    if not visualizer_path.exists():
        raise FileNotFoundError(
            f"Visualizer executable not found: {visualizer_path}. "
            "Run: cmake --build build"
        )

    logging.info("Starting visualizer: %s", visualizer_path)
    return subprocess.Popen(
        [
            str(visualizer_path),
            "--udp-port",
            str(udp_port),
            "--score-udp-port",
            str(score_udp_port),
        ]
    )


def stop_visualizer(process: subprocess.Popen[bytes] | None) -> None:
    if process is None or process.poll() is not None:
        return

    logging.info("Stopping visualizer...")
    process.terminate()
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


def run_capture(config: LauncherConfig) -> int:
    service = CaptureService(
        port=config.port,
        baud=config.baud,
        udp_host=config.udp_host,
        udp_port=config.udp_port,
        score_udp_port=config.score_udp_port,
        out_dir=config.out_dir if config.out_dir is not None else default_output_dir(),
        scoring_config_path=config.scoring_config,
    )

    logging.info("Starting capture on %s at %d baud", config.port, config.baud)
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


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    visualizer_process: subprocess.Popen[bytes] | None = None
    try:
        config = resolve_runtime_config(args)
        logging.info(
            "Launcher config: port=%s baud=%d udp=%s:%d score_udp=%d out_dir=%s scoring_config=%s",
            config.port,
            config.baud,
            config.udp_host,
            config.udp_port,
            config.score_udp_port,
            config.out_dir,
            config.scoring_config,
        )

        if not args.no_visualizer:
            visualizer_process = start_visualizer(config.visualizer, config.udp_port, config.score_udp_port)
            time.sleep(0.5)

        return run_capture(config)
    except KeyboardInterrupt:
        logging.info("Interrupted by user")
        return 0
    except Exception as exc:
        logging.error("%s", exc)
        return 1
    finally:
        stop_visualizer(visualizer_process)


if __name__ == "__main__":
    raise SystemExit(main())
