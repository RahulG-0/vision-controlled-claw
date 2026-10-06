"""
Finger joint angles from hand landmarks.

Each finger is a chain of four landmarks anchored at the wrist. At every
interior joint we measure the angle between the two bone segments that meet
there: 180 degrees is a straight finger, smaller is more curled. Averaging a
finger's joint angles gives one curl value per finger, which is the number a
servo needs to mirror the hand.

This module has no MediaPipe dependency, so it can be tested with plain
coordinate lists.
"""

import math

Landmark = tuple[float, float, float]

WRIST = 0
FINGERS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}
JOINT_NAMES = {
    "thumb": ("cmc", "mcp", "ip"),
    "default": ("mcp", "pip", "dip"),
}


def _sub(a: Landmark, b: Landmark) -> Landmark:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def angle_at(prev_pt: Landmark, joint: Landmark, next_pt: Landmark) -> float:
    """Angle in degrees at `joint` between the segments to its two neighbours."""
    v1, v2 = _sub(prev_pt, joint), _sub(next_pt, joint)
    n1 = math.sqrt(sum(c * c for c in v1))
    n2 = math.sqrt(sum(c * c for c in v2))
    if n1 == 0 or n2 == 0:
        return 180.0
    cos = sum(a * b for a, b in zip(v1, v2)) / (n1 * n2)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def finger_joint_angles(landmarks: list[Landmark], finger: str) -> dict[str, float]:
    """Angle at each of the three interior joints of one finger."""
    chain = [landmarks[WRIST]] + [landmarks[i] for i in FINGERS[finger]]
    names = JOINT_NAMES.get(finger, JOINT_NAMES["default"])
    return {
        name: round(angle_at(chain[i], chain[i + 1], chain[i + 2]), 1)
        for i, name in enumerate(names)
    }


def all_finger_angles(landmarks: list[Landmark]) -> dict[str, dict[str, float]]:
    return {finger: finger_joint_angles(landmarks, finger) for finger in FINGERS}


def curl(joint_angles: dict[str, float]) -> float:
    """Collapse a finger's joint angles to 0.0 (fully curled) .. 1.0 (straight)."""
    return round(sum(joint_angles.values()) / (180.0 * len(joint_angles)), 3)


def servo_angle(curl_value: float, closed_deg: float = 0.0, open_deg: float = 180.0,
                curl_min: float = 0.55, curl_max: float = 0.97) -> float:
    """
    Map a curl value onto a servo angle.

    A real hand never reaches curl 0.0, so `curl_min` and `curl_max` are the
    measured values for a closed fist and an open hand. Values outside that
    range are clamped.
    """
    span = curl_max - curl_min
    t = 0.0 if span <= 0 else (curl_value - curl_min) / span
    t = max(0.0, min(1.0, t))
    return round(closed_deg + t * (open_deg - closed_deg), 1)
