from __future__ import annotations

import struct
from dataclasses import dataclass

SERIAL_HEADER = 0xAA
SERIAL_HEADER_BYTES = bytes([SERIAL_HEADER])
FRAME_GROUP_SIZE = 5
FRAME_INTERVAL_MS = 4
WINDOW_PERIOD_MS = FRAME_GROUP_SIZE * FRAME_INTERVAL_MS

SERIAL_PACKET_STRUCT = struct.Struct("<BBB20fB")
SERIAL_PACKET_SIZE = SERIAL_PACKET_STRUCT.size

UDP_MAGIC = b"UMON"
UDP_VERSION = 1
UDP_FRAME_STRUCT = struct.Struct("<4sBBBBQ4f4f4fBBBBff")
UDP_FRAME_SIZE = UDP_FRAME_STRUCT.size

NODE_ID_TO_NAME = {
    0x01: "upper_arm",
    0x02: "forearm",
    0x03: "hand_palm",
}
NODE_NAME_TO_ID = {name: node_id for node_id, name in NODE_ID_TO_NAME.items()}
NODE_ORDER = ("upper_arm", "forearm", "hand_palm")
IDENTITY_QUATERNION = (1.0, 0.0, 0.0, 0.0)

Quaternion = tuple[float, float, float, float]

CSV_HEADER = [
    "timestamp_ms",
    "seq_cnt",
    "sample_idx",
    "upper_arm_w",
    "upper_arm_x",
    "upper_arm_y",
    "upper_arm_z",
    "forearm_w",
    "forearm_x",
    "forearm_y",
    "forearm_z",
    "hand_palm_w",
    "hand_palm_x",
    "hand_palm_y",
    "hand_palm_z",
    "upper_arm_stale",
    "forearm_stale",
    "hand_palm_stale",
    "hand_palm_pitch_deg",
    "hand_palm_pitch_dps",
]


@dataclass(slots=True)
class SerialNodePacket:
    node_id: int
    node_name: str
    seq_cnt: int
    quaternions: tuple[Quaternion, ...]
    checksum: int
    raw_bytes: bytes
    received_monotonic: float
    received_timestamp_ms: int


@dataclass(slots=True)
class AlignedFrame:
    timestamp_ms: int
    seq_cnt: int
    sample_idx: int
    upper_arm: Quaternion
    forearm: Quaternion
    hand_palm: Quaternion
    upper_arm_stale: bool
    forearm_stale: bool
    hand_palm_stale: bool
    hand_palm_pitch_deg: float
    hand_palm_pitch_dps: float

    def csv_row(self) -> list[object]:
        return [
            self.timestamp_ms,
            self.seq_cnt,
            self.sample_idx,
            *self.upper_arm,
            *self.forearm,
            *self.hand_palm,
            int(self.upper_arm_stale),
            int(self.forearm_stale),
            int(self.hand_palm_stale),
            self.hand_palm_pitch_deg,
            self.hand_palm_pitch_dps,
        ]


def compute_checksum(payload: bytes) -> int:
    checksum = 0
    for value in payload:
        checksum ^= value
    return checksum


def is_valid_serial_packet(raw_packet: bytes) -> bool:
    if len(raw_packet) != SERIAL_PACKET_SIZE:
        return False
    if raw_packet[0] != SERIAL_HEADER:
        return False
    return compute_checksum(raw_packet[:-1]) == raw_packet[-1]


def unpack_serial_packet(
    raw_packet: bytes,
    received_monotonic: float,
    received_timestamp_ms: int,
) -> SerialNodePacket:
    if len(raw_packet) != SERIAL_PACKET_SIZE:
        raise ValueError(f"Expected {SERIAL_PACKET_SIZE} bytes, got {len(raw_packet)}")

    header, node_id, seq_cnt, *values = SERIAL_PACKET_STRUCT.unpack(raw_packet)
    checksum = values.pop()

    if header != SERIAL_HEADER:
        raise ValueError(f"Invalid packet header 0x{header:02X}")
    if compute_checksum(raw_packet[:-1]) != checksum:
        raise ValueError("Serial packet checksum mismatch")

    node_name = NODE_ID_TO_NAME.get(node_id)
    if node_name is None:
        raise ValueError(f"Unsupported node_id 0x{node_id:02X}")

    quaternions = tuple(
        (values[index], values[index + 1], values[index + 2], values[index + 3])
        for index in range(0, len(values), 4)
    )
    if len(quaternions) != FRAME_GROUP_SIZE:
        raise ValueError("Expected exactly 5 quaternion frames per serial packet")

    return SerialNodePacket(
        node_id=node_id,
        node_name=node_name,
        seq_cnt=seq_cnt,
        quaternions=quaternions,
        checksum=checksum,
        raw_bytes=raw_packet,
        received_monotonic=received_monotonic,
        received_timestamp_ms=received_timestamp_ms,
    )


def pack_udp_frame(frame: AlignedFrame) -> bytes:
    return UDP_FRAME_STRUCT.pack(
        UDP_MAGIC,
        UDP_VERSION,
        frame.seq_cnt,
        frame.sample_idx,
        0,
        frame.timestamp_ms,
        *frame.upper_arm,
        *frame.forearm,
        *frame.hand_palm,
        int(frame.upper_arm_stale),
        int(frame.forearm_stale),
        int(frame.hand_palm_stale),
        0,
        frame.hand_palm_pitch_deg,
        frame.hand_palm_pitch_dps,
    )


class SerialPacketStreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(
        self,
        chunk: bytes,
        received_monotonic: float,
        received_timestamp_ms: int,
    ) -> list[SerialNodePacket]:
        packets: list[SerialNodePacket] = []
        if not chunk:
            return packets

        self._buffer.extend(chunk)
        while True:
            header_index = self._buffer.find(SERIAL_HEADER_BYTES)
            if header_index < 0:
                self._buffer.clear()
                break

            if header_index > 0:
                del self._buffer[:header_index]

            if len(self._buffer) < SERIAL_PACKET_SIZE:
                break

            candidate = bytes(self._buffer[:SERIAL_PACKET_SIZE])
            if not is_valid_serial_packet(candidate):
                del self._buffer[0]
                continue

            packets.append(
                unpack_serial_packet(
                    candidate,
                    received_monotonic=received_monotonic,
                    received_timestamp_ms=received_timestamp_ms,
                )
            )
            del self._buffer[:SERIAL_PACKET_SIZE]

        return packets
