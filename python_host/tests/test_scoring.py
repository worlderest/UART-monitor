from __future__ import annotations

import math
import unittest

from python_host.protocol import AlignedFrame
from python_host.scoring import ScorePipeline, ScoringConfig

FRAME_INTERVAL_MS = 4


def quat_x_deg(angle_deg: float) -> tuple[float, float, float, float]:
    radians = math.radians(angle_deg) * 0.5
    return (math.cos(radians), math.sin(radians), 0.0, 0.0)


def make_frame(
    timestamp_ms: int,
    hand_pitch_deg: float,
    hand_pitch_dps: float,
    upper_arm_pitch_deg: float = 0.0,
    forearm_pitch_deg: float = 0.0,
    upper_arm_stale: bool = False,
    forearm_stale: bool = False,
    hand_palm_stale: bool = False,
) -> AlignedFrame:
    return AlignedFrame(
        timestamp_ms=timestamp_ms,
        seq_cnt=(timestamp_ms // FRAME_INTERVAL_MS) & 0xFF,
        sample_idx=(timestamp_ms // FRAME_INTERVAL_MS) % 5,
        upper_arm=quat_x_deg(upper_arm_pitch_deg),
        forearm=quat_x_deg(forearm_pitch_deg),
        hand_palm=quat_x_deg(hand_pitch_deg),
        upper_arm_stale=upper_arm_stale,
        forearm_stale=forearm_stale,
        hand_palm_stale=hand_palm_stale,
        hand_palm_pitch_deg=hand_pitch_deg,
        hand_palm_pitch_dps=hand_pitch_dps,
    )


class ScorePipelineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.pipeline = ScorePipeline(ScoringConfig())

    def test_standard_shot_scores_once(self) -> None:
        frames, trigger_timestamp = standard_shot_frames()

        events = collect_events(self.pipeline, frames)

        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(event.shot_timestamp_ms, trigger_timestamp)
        self.assertGreaterEqual(event.total_score, 0.0)
        self.assertLessEqual(event.total_score, 100.0)
        self.assertGreater(event.max_muzzle_jump_deg, 0.0)
        self.assertLess(event.recovery_time_ms, 800.0)

    def test_raise_arm_is_not_misdetected_as_shot(self) -> None:
        frames = raising_arm_frames()

        events = collect_events(self.pipeline, frames)

        self.assertEqual(events, [])

    def test_single_recoil_only_counts_once(self) -> None:
        frames, _ = standard_shot_frames(include_extra_spike=True)

        events = collect_events(self.pipeline, frames)

        self.assertEqual(len(events), 1)

    def test_unrecovered_shot_uses_timeout_score(self) -> None:
        frames, _ = standard_shot_frames(recover=False)

        events = collect_events(self.pipeline, frames)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].recovery_time_ms, 800.0)

    def test_trigger_timestamp_matches_peak_frame(self) -> None:
        frames, trigger_timestamp = standard_shot_frames()

        events = collect_events(self.pipeline, frames)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].shot_timestamp_ms, trigger_timestamp)


def collect_events(pipeline: ScorePipeline, frames: list[AlignedFrame]) -> list:
    events = []
    for frame in frames:
        events.extend(pipeline.ingest(frame))
    return events


def standard_shot_frames(include_extra_spike: bool = False, recover: bool = True) -> tuple[list[AlignedFrame], int]:
    frames: list[AlignedFrame] = []
    timestamp_ms = 0

    for _ in range(50):
        frames.append(make_frame(timestamp_ms, hand_pitch_deg=0.0, hand_pitch_dps=0.0))
        timestamp_ms += FRAME_INTERVAL_MS

    shot_pitches = [2.0, 6.0, 9.5, 12.0, 9.0, 6.0, 3.0]
    shot_dps = [230.0, 280.0, 140.0, 80.0, 35.0, 18.0, 8.0]
    trigger_timestamp = timestamp_ms + FRAME_INTERVAL_MS
    for pitch_deg, pitch_dps in zip(shot_pitches, shot_dps):
        frames.append(make_frame(timestamp_ms, hand_pitch_deg=pitch_deg, hand_pitch_dps=pitch_dps))
        timestamp_ms += FRAME_INTERVAL_MS

    if include_extra_spike:
        extra_pitches = [11.0, 10.0, 8.0]
        extra_dps = [250.0, 130.0, 40.0]
        for pitch_deg, pitch_dps in zip(extra_pitches, extra_dps):
            frames.append(make_frame(timestamp_ms, hand_pitch_deg=pitch_deg, hand_pitch_dps=pitch_dps))
            timestamp_ms += FRAME_INTERVAL_MS

    if recover:
        recovery_profile = [
            (4.0, 30.0),
            (2.0, 12.0),
            (1.0, 6.0),
            (0.6, 3.0),
            (0.2, 2.0),
        ]
        for pitch_deg, pitch_dps in recovery_profile:
            frames.append(make_frame(timestamp_ms, hand_pitch_deg=pitch_deg, hand_pitch_dps=pitch_dps))
            timestamp_ms += FRAME_INTERVAL_MS
        for _ in range(60):
            frames.append(make_frame(timestamp_ms, hand_pitch_deg=0.0, hand_pitch_dps=0.0))
            timestamp_ms += FRAME_INTERVAL_MS
    else:
        for _ in range(220):
            frames.append(make_frame(timestamp_ms, hand_pitch_deg=10.0, hand_pitch_dps=0.0))
            timestamp_ms += FRAME_INTERVAL_MS

    return frames, trigger_timestamp


def raising_arm_frames() -> list[AlignedFrame]:
    frames: list[AlignedFrame] = []
    timestamp_ms = 0

    for _ in range(50):
        frames.append(make_frame(timestamp_ms, hand_pitch_deg=0.0, hand_pitch_dps=0.0))
        timestamp_ms += FRAME_INTERVAL_MS

    for step in range(60):
        upper_pitch = step * 1.5
        forearm_pitch = step * 1.5
        hand_pitch = step * 1.5
        frames.append(
            make_frame(
                timestamp_ms,
                hand_pitch_deg=hand_pitch,
                hand_pitch_dps=260.0,
                upper_arm_pitch_deg=upper_pitch,
                forearm_pitch_deg=forearm_pitch,
            )
        )
        timestamp_ms += FRAME_INTERVAL_MS

    for _ in range(40):
        frames.append(
            make_frame(
                timestamp_ms,
                hand_pitch_deg=90.0,
                hand_pitch_dps=0.0,
                upper_arm_pitch_deg=90.0,
                forearm_pitch_deg=90.0,
            )
        )
        timestamp_ms += FRAME_INTERVAL_MS

    return frames


if __name__ == "__main__":
    unittest.main()
