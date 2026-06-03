from __future__ import annotations

import math

from python_host.protocol import Quaternion


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quaternion_to_euler_deg(quaternion: Quaternion) -> tuple[float, float, float]:
    w, x, y, z = quaternion

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(_clamp(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def shortest_angle_delta_deg(previous_deg: float, current_deg: float) -> float:
    delta = (current_deg - previous_deg + 180.0) % 360.0 - 180.0
    return delta


def pitch_deg(quaternion: Quaternion) -> float:
    return quaternion_to_euler_deg(quaternion)[1]
