from __future__ import annotations

import math

from python_host.protocol import Quaternion

Vector3 = tuple[float, float, float]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def quaternion_to_euler_deg(quaternion: Quaternion) -> tuple[float, float, float]:
    w, x, y, z = _normalize_quaternion(quaternion)

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


def quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def relative_quaternion(current: Quaternion, reference: Quaternion) -> Quaternion:
    relative = quaternion_multiply(_normalize_quaternion(current), quaternion_conjugate(_normalize_quaternion(reference)))
    return _normalize_quaternion(relative)


def rotation_angle_deg(quaternion: Quaternion) -> float:
    normalized = _normalize_quaternion(quaternion)
    w = _clamp(abs(normalized[0]), 0.0, 1.0)
    return math.degrees(2.0 * math.acos(w))


def angular_speed_dps(previous: Quaternion, current: Quaternion, dt_seconds: float) -> float:
    if dt_seconds <= 0.0:
        return 0.0
    return rotation_angle_deg(relative_quaternion(current, previous)) / dt_seconds


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    normalized = _normalize_quaternion(quaternion)
    vector_quaternion: Quaternion = (0.0, vector[0], vector[1], vector[2])
    rotated = quaternion_multiply(
        quaternion_multiply(normalized, vector_quaternion),
        quaternion_conjugate(normalized),
    )
    return (rotated[1], rotated[2], rotated[3])


def angle_between_vectors_deg(lhs: Vector3, rhs: Vector3) -> float:
    lhs_length = math.sqrt(lhs[0] * lhs[0] + lhs[1] * lhs[1] + lhs[2] * lhs[2])
    rhs_length = math.sqrt(rhs[0] * rhs[0] + rhs[1] * rhs[1] + rhs[2] * rhs[2])
    if lhs_length <= 1e-9 or rhs_length <= 1e-9:
        return 0.0

    dot = lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]
    cosine = _clamp(dot / (lhs_length * rhs_length), -1.0, 1.0)
    return math.degrees(math.acos(cosine))


def pitch_deg(quaternion: Quaternion) -> float:
    return quaternion_to_euler_deg(quaternion)[0]
