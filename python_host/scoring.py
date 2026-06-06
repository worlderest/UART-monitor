from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque

from python_host.math3d import (
    angle_between_vectors_deg,
    angular_speed_dps,
    rotate_vector,
)
from python_host.protocol import FRAME_INTERVAL_MS, AlignedFrame, Quaternion, ShotScoreEvent

Vector3 = tuple[float, float, float]
DEFAULT_SCORING_CONFIG_PATH = Path("config") / "scoring.json"


@dataclass(slots=True, frozen=True)
class ScoreWeights:
    stability: float = 0.40
    muzzle_jump: float = 0.35
    recovery: float = 0.25


@dataclass(slots=True, frozen=True)
class ScoreThresholds:
    stability_jitter_deg: float
    max_muzzle_jump_deg: float
    recovery_time_ms: float


@dataclass(slots=True, frozen=True)
class ScoringConfig:
    stable_window_ms: int = 160
    candidate_window_ms: int = 80
    recoil_window_ms: int = 160
    recovery_hold_ms: int = 120
    recovery_timeout_ms: int = 800
    refractory_ms: int = 300
    hand_stable_speed_dps: float = 45.0
    forearm_stable_speed_dps: float = 45.0
    upper_arm_stable_speed_dps: float = 35.0
    trigger_hand_pitch_dps: float = 220.0
    raise_guard_forearm_dps: float = 100.0
    raise_guard_upper_arm_dps: float = 80.0
    recovery_band_deg: float = 2.5
    weights: ScoreWeights = field(default_factory=ScoreWeights)
    good_thresholds: ScoreThresholds = field(
        default_factory=lambda: ScoreThresholds(
            stability_jitter_deg=0.8,
            max_muzzle_jump_deg=6.0,
            recovery_time_ms=180.0,
        )
    )
    bad_thresholds: ScoreThresholds = field(
        default_factory=lambda: ScoreThresholds(
            stability_jitter_deg=4.0,
            max_muzzle_jump_deg=18.0,
            recovery_time_ms=800.0,
        )
    )

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> ScoringConfig:
        weights_data = data.get("weights", {})
        good_thresholds_data = data.get("good_thresholds", {})
        bad_thresholds_data = data.get("bad_thresholds", {})
        if not isinstance(weights_data, dict) or not isinstance(good_thresholds_data, dict) or not isinstance(bad_thresholds_data, dict):
            raise ValueError("weights, good_thresholds, and bad_thresholds must be JSON objects")

        defaults = cls()
        return cls(
            stable_window_ms=int(data.get("stable_window_ms", defaults.stable_window_ms)),
            candidate_window_ms=int(data.get("candidate_window_ms", defaults.candidate_window_ms)),
            recoil_window_ms=int(data.get("recoil_window_ms", defaults.recoil_window_ms)),
            recovery_hold_ms=int(data.get("recovery_hold_ms", defaults.recovery_hold_ms)),
            recovery_timeout_ms=int(data.get("recovery_timeout_ms", defaults.recovery_timeout_ms)),
            refractory_ms=int(data.get("refractory_ms", defaults.refractory_ms)),
            hand_stable_speed_dps=float(data.get("hand_stable_speed_dps", defaults.hand_stable_speed_dps)),
            forearm_stable_speed_dps=float(data.get("forearm_stable_speed_dps", defaults.forearm_stable_speed_dps)),
            upper_arm_stable_speed_dps=float(data.get("upper_arm_stable_speed_dps", defaults.upper_arm_stable_speed_dps)),
            trigger_hand_pitch_dps=float(data.get("trigger_hand_pitch_dps", defaults.trigger_hand_pitch_dps)),
            raise_guard_forearm_dps=float(data.get("raise_guard_forearm_dps", defaults.raise_guard_forearm_dps)),
            raise_guard_upper_arm_dps=float(data.get("raise_guard_upper_arm_dps", defaults.raise_guard_upper_arm_dps)),
            recovery_band_deg=float(data.get("recovery_band_deg", defaults.recovery_band_deg)),
            weights=ScoreWeights(
                stability=float(weights_data.get("stability", defaults.weights.stability)),
                muzzle_jump=float(weights_data.get("muzzle_jump", defaults.weights.muzzle_jump)),
                recovery=float(weights_data.get("recovery", defaults.weights.recovery)),
            ),
            good_thresholds=ScoreThresholds(
                stability_jitter_deg=float(
                    good_thresholds_data.get("stability_jitter_deg", defaults.good_thresholds.stability_jitter_deg)
                ),
                max_muzzle_jump_deg=float(
                    good_thresholds_data.get("max_muzzle_jump_deg", defaults.good_thresholds.max_muzzle_jump_deg)
                ),
                recovery_time_ms=float(
                    good_thresholds_data.get("recovery_time_ms", defaults.good_thresholds.recovery_time_ms)
                ),
            ),
            bad_thresholds=ScoreThresholds(
                stability_jitter_deg=float(
                    bad_thresholds_data.get("stability_jitter_deg", defaults.bad_thresholds.stability_jitter_deg)
                ),
                max_muzzle_jump_deg=float(
                    bad_thresholds_data.get("max_muzzle_jump_deg", defaults.bad_thresholds.max_muzzle_jump_deg)
                ),
                recovery_time_ms=float(
                    bad_thresholds_data.get("recovery_time_ms", defaults.bad_thresholds.recovery_time_ms)
                ),
            ),
        )


def load_scoring_config(path: Path) -> ScoringConfig:
    with path.open("r", encoding="utf-8") as config_file:
        data = json.load(config_file)
    if not isinstance(data, dict):
        raise ValueError(f"Scoring config {path} must contain a JSON object")
    return ScoringConfig.from_dict(data)


@dataclass(slots=True, frozen=True)
class DerivedFrameSample:
    timestamp_ms: int
    hand_pitch_deg: float
    hand_pitch_dps: float
    hand_angular_speed_dps: float
    forearm_angular_speed_dps: float
    upper_arm_angular_speed_dps: float
    aim_axis_world: Vector3
    any_stale: bool


@dataclass(slots=True)
class PendingShot:
    trigger_timestamp_ms: int


class AutoTriggerDetector:
    def __init__(self, config: ScoringConfig) -> None:
        self._config = config
        self._state = "idle"
        self._stable_run_start_ms: int | None = None
        self._candidate_start_ms: int | None = None
        self._candidate_peak_sample: DerivedFrameSample | None = None
        self._recovery_trigger_ms: int | None = None
        self._recovery_stable_start_ms: int | None = None
        self._refractory_start_ms: int | None = None

    def feed(self, sample: DerivedFrameSample) -> DerivedFrameSample | None:
        if self._state == "candidate":
            return self._handle_candidate(sample)
        if self._state == "recovering":
            self._handle_recovering(sample)
            return None
        if self._state == "refractory":
            self._handle_refractory(sample)
            return None
        if self._state == "armed":
            return self._handle_armed(sample)
        return self._handle_idle(sample)

    def _handle_idle(self, sample: DerivedFrameSample) -> DerivedFrameSample | None:
        if self._is_stable(sample):
            if self._stable_run_start_ms is None:
                self._stable_run_start_ms = sample.timestamp_ms
            if self._covered_duration_ms(self._stable_run_start_ms, sample.timestamp_ms) >= self._config.stable_window_ms:
                self._state = "armed"
        else:
            self._stable_run_start_ms = None
        return None

    def _handle_armed(self, sample: DerivedFrameSample) -> DerivedFrameSample | None:
        if self._should_start_candidate(sample):
            self._state = "candidate"
            self._candidate_start_ms = sample.timestamp_ms
            self._candidate_peak_sample = sample
            return None

        if not self._is_stable(sample):
            self._state = "idle"
            self._stable_run_start_ms = None
        return None

    def _handle_candidate(self, sample: DerivedFrameSample) -> DerivedFrameSample | None:
        assert self._candidate_start_ms is not None
        assert self._candidate_peak_sample is not None

        if sample.any_stale or not self._passes_raise_guard(sample):
            self._reset_to_idle()
            return None

        if sample.hand_pitch_dps > self._candidate_peak_sample.hand_pitch_dps:
            self._candidate_peak_sample = sample

        elapsed_ms = self._covered_duration_ms(self._candidate_start_ms, sample.timestamp_ms)
        if sample.hand_pitch_dps <= self._candidate_peak_sample.hand_pitch_dps * 0.60 or elapsed_ms >= self._config.candidate_window_ms:
            trigger_sample = self._candidate_peak_sample
            self._state = "recovering"
            self._recovery_trigger_ms = trigger_sample.timestamp_ms
            self._recovery_stable_start_ms = None
            self._candidate_start_ms = None
            self._candidate_peak_sample = None
            self._stable_run_start_ms = None
            return trigger_sample
        return None

    def _handle_recovering(self, sample: DerivedFrameSample) -> None:
        assert self._recovery_trigger_ms is not None
        elapsed_ms = self._covered_duration_ms(self._recovery_trigger_ms, sample.timestamp_ms)

        if self._is_stable(sample):
            if self._recovery_stable_start_ms is None:
                self._recovery_stable_start_ms = sample.timestamp_ms
            if self._covered_duration_ms(self._recovery_stable_start_ms, sample.timestamp_ms) >= self._config.recovery_hold_ms:
                self._enter_refractory(sample.timestamp_ms)
                return
        else:
            self._recovery_stable_start_ms = None

        if elapsed_ms >= self._config.recovery_timeout_ms:
            self._enter_refractory(sample.timestamp_ms)

    def _handle_refractory(self, sample: DerivedFrameSample) -> None:
        assert self._refractory_start_ms is not None
        elapsed_ms = self._covered_duration_ms(self._refractory_start_ms, sample.timestamp_ms)
        if elapsed_ms < self._config.refractory_ms:
            return

        self._state = "idle"
        self._refractory_start_ms = None
        self._recovery_trigger_ms = None
        self._recovery_stable_start_ms = None
        self._stable_run_start_ms = sample.timestamp_ms if self._is_stable(sample) else None

    def _enter_refractory(self, timestamp_ms: int) -> None:
        self._state = "refractory"
        self._refractory_start_ms = timestamp_ms
        self._recovery_stable_start_ms = None
        self._recovery_trigger_ms = None

    def _reset_to_idle(self) -> None:
        self._state = "idle"
        self._stable_run_start_ms = None
        self._candidate_start_ms = None
        self._candidate_peak_sample = None
        self._recovery_trigger_ms = None
        self._recovery_stable_start_ms = None
        self._refractory_start_ms = None

    def _should_start_candidate(self, sample: DerivedFrameSample) -> bool:
        return sample.hand_pitch_dps >= self._config.trigger_hand_pitch_dps and self._passes_raise_guard(sample)

    def _passes_raise_guard(self, sample: DerivedFrameSample) -> bool:
        return (
            sample.forearm_angular_speed_dps < self._config.raise_guard_forearm_dps
            and sample.upper_arm_angular_speed_dps < self._config.raise_guard_upper_arm_dps
        )

    def _is_stable(self, sample: DerivedFrameSample) -> bool:
        return (
            not sample.any_stale
            and sample.hand_angular_speed_dps <= self._config.hand_stable_speed_dps
            and sample.forearm_angular_speed_dps <= self._config.forearm_stable_speed_dps
            and sample.upper_arm_angular_speed_dps <= self._config.upper_arm_stable_speed_dps
        )

    @staticmethod
    def _covered_duration_ms(start_ms: int, end_ms: int) -> int:
        return end_ms - start_ms + FRAME_INTERVAL_MS


class ShotScorer:
    def __init__(self, config: ScoringConfig) -> None:
        self._config = config

    def try_score(
        self,
        samples: tuple[DerivedFrameSample, ...],
        trigger_timestamp_ms: int,
        current_timestamp_ms: int,
    ) -> ShotScoreEvent | None:
        pre_samples = self._window_samples(
            samples,
            start_ms=trigger_timestamp_ms - self._config.stable_window_ms,
            end_ms=trigger_timestamp_ms,
            include_end=False,
        )
        if not pre_samples:
            return None

        recoil_window_end_ms = trigger_timestamp_ms + self._config.recoil_window_ms
        if current_timestamp_ms < recoil_window_end_ms:
            return None

        recoil_samples = self._window_samples(
            samples,
            start_ms=trigger_timestamp_ms,
            end_ms=recoil_window_end_ms,
            include_end=True,
        )
        if not recoil_samples:
            return None

        baseline_pitch_deg = sum(sample.hand_pitch_deg for sample in pre_samples) / len(pre_samples)
        baseline_axis = _average_unit_vector(sample.aim_axis_world for sample in pre_samples)

        stability_jitter_deg = math.sqrt(
            sum(angle_between_vectors_deg(sample.aim_axis_world, baseline_axis) ** 2 for sample in pre_samples) / len(pre_samples)
        )
        max_muzzle_jump_deg = max(0.0, max(sample.hand_pitch_deg - baseline_pitch_deg for sample in recoil_samples))
        recovery_time_ms = self._find_recovery_time_ms(samples, trigger_timestamp_ms, baseline_axis, current_timestamp_ms)
        if recovery_time_ms is None:
            return None

        total_score = self._total_score(
            stability_jitter_deg=stability_jitter_deg,
            max_muzzle_jump_deg=max_muzzle_jump_deg,
            recovery_time_ms=recovery_time_ms,
        )
        return ShotScoreEvent(
            shot_timestamp_ms=trigger_timestamp_ms,
            total_score=total_score,
            stability_jitter_deg=stability_jitter_deg,
            max_muzzle_jump_deg=max_muzzle_jump_deg,
            recovery_time_ms=recovery_time_ms,
        )

    def _find_recovery_time_ms(
        self,
        samples: tuple[DerivedFrameSample, ...],
        trigger_timestamp_ms: int,
        baseline_axis: Vector3,
        current_timestamp_ms: int,
    ) -> float | None:
        timeout_end_ms = trigger_timestamp_ms + self._config.recovery_timeout_ms
        hold_start_ms: int | None = None
        for sample in samples:
            if sample.timestamp_ms <= trigger_timestamp_ms:
                continue
            if sample.timestamp_ms > min(current_timestamp_ms, timeout_end_ms):
                break

            recovered = self._is_recovered(sample, baseline_axis)
            if recovered:
                if hold_start_ms is None:
                    hold_start_ms = sample.timestamp_ms
                if sample.timestamp_ms - hold_start_ms + FRAME_INTERVAL_MS >= self._config.recovery_hold_ms:
                    return float(sample.timestamp_ms - trigger_timestamp_ms)
            else:
                hold_start_ms = None

        if current_timestamp_ms >= timeout_end_ms:
            return float(self._config.recovery_timeout_ms)
        return None

    def _is_recovered(self, sample: DerivedFrameSample, baseline_axis: Vector3) -> bool:
        aim_error_deg = angle_between_vectors_deg(sample.aim_axis_world, baseline_axis)
        return (
            not sample.any_stale
            and aim_error_deg <= self._config.recovery_band_deg
            and sample.hand_angular_speed_dps <= self._config.hand_stable_speed_dps
            and sample.forearm_angular_speed_dps <= self._config.forearm_stable_speed_dps
            and sample.upper_arm_angular_speed_dps <= self._config.upper_arm_stable_speed_dps
        )

    def _total_score(
        self,
        stability_jitter_deg: float,
        max_muzzle_jump_deg: float,
        recovery_time_ms: float,
    ) -> float:
        weights = self._config.weights
        thresholds_good = self._config.good_thresholds
        thresholds_bad = self._config.bad_thresholds
        stability_score = _normalized_subscore(
            stability_jitter_deg,
            thresholds_good.stability_jitter_deg,
            thresholds_bad.stability_jitter_deg,
        )
        jump_score = _normalized_subscore(
            max_muzzle_jump_deg,
            thresholds_good.max_muzzle_jump_deg,
            thresholds_bad.max_muzzle_jump_deg,
        )
        recovery_score = _normalized_subscore(
            recovery_time_ms,
            thresholds_good.recovery_time_ms,
            thresholds_bad.recovery_time_ms,
        )
        total_weight = weights.stability + weights.muzzle_jump + weights.recovery
        if total_weight <= 0.0:
            return 0.0
        return (
            stability_score * weights.stability
            + jump_score * weights.muzzle_jump
            + recovery_score * weights.recovery
        ) / total_weight

    @staticmethod
    def _window_samples(
        samples: tuple[DerivedFrameSample, ...],
        start_ms: int,
        end_ms: int,
        include_end: bool,
    ) -> list[DerivedFrameSample]:
        if include_end:
            return [sample for sample in samples if start_ms <= sample.timestamp_ms <= end_ms]
        return [sample for sample in samples if start_ms <= sample.timestamp_ms < end_ms]


class ScorePipeline:
    def __init__(self, config: ScoringConfig) -> None:
        self._config = config
        self._detector = AutoTriggerDetector(config)
        self._scorer = ShotScorer(config)
        self._samples: Deque[DerivedFrameSample] = deque()
        self._pending_shot: PendingShot | None = None
        self._previous_frame: AlignedFrame | None = None
        self._history_window_ms = (
            config.stable_window_ms
            + config.recovery_timeout_ms
            + config.recovery_hold_ms
            + config.refractory_ms
            + config.candidate_window_ms
            + 200
        )

    def ingest(self, frame: AlignedFrame) -> list[ShotScoreEvent]:
        sample = self._derive_sample(frame)
        self._samples.append(sample)
        self._trim_history(sample.timestamp_ms)

        trigger_sample = self._detector.feed(sample)
        if trigger_sample is not None and self._pending_shot is None:
            self._pending_shot = PendingShot(trigger_timestamp_ms=trigger_sample.timestamp_ms)

        if self._pending_shot is None:
            return []

        event = self._scorer.try_score(tuple(self._samples), self._pending_shot.trigger_timestamp_ms, sample.timestamp_ms)
        if event is None:
            return []

        self._pending_shot = None
        return [event]

    def _derive_sample(self, frame: AlignedFrame) -> DerivedFrameSample:
        dt_seconds = 0.0
        if self._previous_frame is not None:
            dt_ms = frame.timestamp_ms - self._previous_frame.timestamp_ms
            if dt_ms > 0:
                dt_seconds = dt_ms / 1000.0

        hand_angular_speed_dps = _segment_angular_speed(
            self._previous_frame.hand_palm if self._previous_frame is not None else None,
            frame.hand_palm,
            dt_seconds,
        )
        forearm_angular_speed_dps = _segment_angular_speed(
            self._previous_frame.forearm if self._previous_frame is not None else None,
            frame.forearm,
            dt_seconds,
        )
        upper_arm_angular_speed_dps = _segment_angular_speed(
            self._previous_frame.upper_arm if self._previous_frame is not None else None,
            frame.upper_arm,
            dt_seconds,
        )
        aim_axis_world = rotate_vector(frame.hand_palm, (0.0, -1.0, 0.0))
        self._previous_frame = frame

        return DerivedFrameSample(
            timestamp_ms=frame.timestamp_ms,
            hand_pitch_deg=frame.hand_palm_pitch_deg,
            hand_pitch_dps=frame.hand_palm_pitch_dps,
            hand_angular_speed_dps=hand_angular_speed_dps,
            forearm_angular_speed_dps=forearm_angular_speed_dps,
            upper_arm_angular_speed_dps=upper_arm_angular_speed_dps,
            aim_axis_world=aim_axis_world,
            any_stale=frame.upper_arm_stale or frame.forearm_stale or frame.hand_palm_stale,
        )

    def _trim_history(self, latest_timestamp_ms: int) -> None:
        cutoff_ms = latest_timestamp_ms - self._history_window_ms
        while self._samples and self._samples[0].timestamp_ms < cutoff_ms:
            self._samples.popleft()


def _segment_angular_speed(previous: Quaternion | None, current: Quaternion, dt_seconds: float) -> float:
    if previous is None or dt_seconds <= 0.0:
        return 0.0
    return angular_speed_dps(previous, current, dt_seconds)


def _average_unit_vector(vectors: list[Vector3] | tuple[Vector3, ...] | object) -> Vector3:
    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0
    count = 0
    for vector in vectors:
        sum_x += vector[0]
        sum_y += vector[1]
        sum_z += vector[2]
        count += 1
    if count == 0:
        return (0.0, -1.0, 0.0)

    length = math.sqrt(sum_x * sum_x + sum_y * sum_y + sum_z * sum_z)
    if length <= 1e-9:
        return (0.0, -1.0, 0.0)
    return (sum_x / length, sum_y / length, sum_z / length)


def _normalized_subscore(value: float, good: float, bad: float) -> float:
    if bad <= good:
        return 0.0
    return max(0.0, min(100.0, 100.0 * (bad - value) / (bad - good)))
