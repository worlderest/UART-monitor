from __future__ import annotations

import time
from dataclasses import dataclass, field

from python_host.math3d import pitch_deg, shortest_angle_delta_deg
from python_host.protocol import (
    AlignedFrame,
    FRAME_GROUP_SIZE,
    FRAME_INTERVAL_MS,
    IDENTITY_QUATERNION,
    NODE_ORDER,
    Quaternion,
    SerialNodePacket,
    WINDOW_PERIOD_MS,
)


@dataclass(slots=True)
class PendingWindow:
    seq_cnt: int
    first_seen_monotonic: float
    packets: dict[str, SerialNodePacket] = field(default_factory=dict)

    def is_complete(self) -> bool:
        return all(node_name in self.packets for node_name in NODE_ORDER)


def _seq_distance(base_seq: int, target_seq: int) -> int:
    return (target_seq - base_seq) & 0xFF


def _is_seq_behind(seq_cnt: int, reference_seq: int) -> bool:
    distance = (reference_seq - seq_cnt) & 0xFF
    return 1 <= distance <= 127


class PacketAligner:
    def __init__(self, stale_timeout_ms: int = 40) -> None:
        self._stale_timeout_s = stale_timeout_ms / 1000.0
        self._pending: dict[int, PendingWindow] = {}
        self._expected_seq: int | None = None
        self._last_known_quats: dict[str, tuple[Quaternion, ...]] = {
            node_name: tuple(IDENTITY_QUATERNION for _ in range(FRAME_GROUP_SIZE))
            for node_name in NODE_ORDER
        }
        self._last_pitch_deg: float | None = None
        self._last_pitch_timestamp_ms: int | None = None
        self._last_window_anchor_ms: int | None = None

    def ingest(self, packet: SerialNodePacket) -> list[AlignedFrame]:
        if self._expected_seq is None:
            self._expected_seq = packet.seq_cnt
        elif _is_seq_behind(packet.seq_cnt, self._expected_seq):
            return []

        window = self._pending.get(packet.seq_cnt)
        if window is None:
            window = PendingWindow(seq_cnt=packet.seq_cnt, first_seen_monotonic=packet.received_monotonic)
            self._pending[packet.seq_cnt] = window

        window.packets[packet.node_name] = packet
        return self._drain_ready()

    def poll(self) -> list[AlignedFrame]:
        return self._drain_ready()

    def _drain_ready(self) -> list[AlignedFrame]:
        ready_frames: list[AlignedFrame] = []

        while self._expected_seq is not None:
            expected_seq = self._expected_seq
            now_monotonic = time.monotonic()
            window = self._pending.get(expected_seq)

            if window is not None:
                age_seconds = now_monotonic - window.first_seen_monotonic
                if window.is_complete():
                    ready_frames.extend(self._finalize_window(expected_seq, window, now_ms=self._next_anchor_ms()))
                    continue
                if age_seconds >= self._stale_timeout_s:
                    ready_frames.extend(self._finalize_window(expected_seq, window, now_ms=self._next_anchor_ms()))
                    continue
                break

            future_window = self._nearest_future_window(expected_seq)
            if future_window is None:
                break

            future_age_seconds = now_monotonic - future_window.first_seen_monotonic
            if future_age_seconds < self._stale_timeout_s:
                break

            ready_frames.extend(self._finalize_window(expected_seq, None, now_ms=self._next_anchor_ms()))

        return ready_frames

    def _nearest_future_window(self, expected_seq: int) -> PendingWindow | None:
        nearest_window: PendingWindow | None = None
        nearest_distance: int | None = None

        for seq_cnt, window in self._pending.items():
            distance = _seq_distance(expected_seq, seq_cnt)
            if distance == 0 or distance > 127:
                continue
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_window = window

        return nearest_window

    def _next_anchor_ms(self) -> int:
        current_ms = time.time_ns() // 1_000_000
        if self._last_window_anchor_ms is None:
            self._last_window_anchor_ms = current_ms
            return current_ms

        anchor_ms = max(current_ms, self._last_window_anchor_ms + WINDOW_PERIOD_MS)
        self._last_window_anchor_ms = anchor_ms
        return anchor_ms

    def _finalize_window(
        self,
        seq_cnt: int,
        window: PendingWindow | None,
        now_ms: int,
    ) -> list[AlignedFrame]:
        packets = window.packets if window is not None else {}
        node_quats: dict[str, tuple[Quaternion, ...]] = {}
        stale_flags: dict[str, bool] = {}

        for node_name in NODE_ORDER:
            packet = packets.get(node_name)
            if packet is None:
                node_quats[node_name] = self._last_known_quats[node_name]
                stale_flags[node_name] = True
                continue

            node_quats[node_name] = packet.quaternions
            self._last_known_quats[node_name] = packet.quaternions
            stale_flags[node_name] = False

        frames: list[AlignedFrame] = []
        for sample_idx in range(FRAME_GROUP_SIZE):
            timestamp_ms = now_ms - (FRAME_GROUP_SIZE - 1 - sample_idx) * FRAME_INTERVAL_MS
            hand_palm_quat = node_quats["hand_palm"][sample_idx]
            hand_palm_pitch = pitch_deg(hand_palm_quat)

            if self._last_pitch_deg is None or self._last_pitch_timestamp_ms is None:
                hand_palm_pitch_dps = 0.0
            else:
                dt_seconds = (timestamp_ms - self._last_pitch_timestamp_ms) / 1000.0
                if dt_seconds <= 0.0:
                    hand_palm_pitch_dps = 0.0
                else:
                    hand_palm_pitch_dps = shortest_angle_delta_deg(self._last_pitch_deg, hand_palm_pitch) / dt_seconds

            frames.append(
                AlignedFrame(
                    timestamp_ms=timestamp_ms,
                    seq_cnt=seq_cnt,
                    sample_idx=sample_idx,
                    upper_arm=node_quats["upper_arm"][sample_idx],
                    forearm=node_quats["forearm"][sample_idx],
                    hand_palm=hand_palm_quat,
                    upper_arm_stale=stale_flags["upper_arm"],
                    forearm_stale=stale_flags["forearm"],
                    hand_palm_stale=stale_flags["hand_palm"],
                    hand_palm_pitch_deg=hand_palm_pitch,
                    hand_palm_pitch_dps=hand_palm_pitch_dps,
                )
            )
            self._last_pitch_deg = hand_palm_pitch
            self._last_pitch_timestamp_ms = timestamp_ms

        self._pending.pop(seq_cnt, None)
        self._expected_seq = (seq_cnt + 1) & 0xFF
        return frames
