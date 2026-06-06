from __future__ import annotations

import unittest

from python_host.protocol import (
    SCORE_EVENT_SIZE,
    SCORES_CSV_HEADER,
    ShotScoreEvent,
    pack_score_event,
    unpack_score_event,
)


class ProtocolTests(unittest.TestCase):
    def test_score_event_pack_unpack_round_trip(self) -> None:
        event = ShotScoreEvent(
            shot_timestamp_ms=1_717_171_717_000,
            total_score=88.5,
            stability_jitter_deg=1.25,
            max_muzzle_jump_deg=7.5,
            recovery_time_ms=212.0,
        )

        payload = pack_score_event(event)

        self.assertEqual(SCORE_EVENT_SIZE, 32)
        self.assertEqual(len(payload), 32)
        unpacked = unpack_score_event(payload)
        self.assertEqual(unpacked.shot_timestamp_ms, event.shot_timestamp_ms)
        self.assertAlmostEqual(unpacked.total_score, event.total_score, places=5)
        self.assertAlmostEqual(unpacked.stability_jitter_deg, event.stability_jitter_deg, places=5)
        self.assertAlmostEqual(unpacked.max_muzzle_jump_deg, event.max_muzzle_jump_deg, places=5)
        self.assertAlmostEqual(unpacked.recovery_time_ms, event.recovery_time_ms, places=5)

    def test_scores_csv_header_is_stable(self) -> None:
        self.assertEqual(
            SCORES_CSV_HEADER,
            [
                "shot_timestamp_ms",
                "total_score",
                "stability_jitter_deg",
                "max_muzzle_jump_deg",
                "recovery_time_ms",
            ],
        )


if __name__ == "__main__":
    unittest.main()
