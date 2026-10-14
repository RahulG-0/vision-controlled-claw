"""
Send the tracked hand to the Gazebo simulation in sim/.

Each frame becomes one line of JSON on a TCP connection to the simulation's
teleop node: where the hand is in the image, how big it looks (a stand-in for
its distance from the camera), how far it is turned, and the servo angle for
each finger.

    {"hand": true, "pos": [u, v], "size": s, "roll": rad, "servo": {"thumb": deg, ...}}
    {"hand": false}

Plain TCP keeps this side free of ROS, so the tracker can run wherever the
camera is. With the simulation in WSL2 and the tracker on Windows, the
default address 127.0.0.1 reaches it.
"""

import json
import math
import socket
import time

from finger_angles import WRIST, all_finger_angles, curl, servo_angle

Landmark = tuple[float, float, float]

DEFAULT_PORT = 8765
# Wrist and the four finger knuckles: the rigid part of the hand, so its
# centre and size do not change when the fingers curl.
PALM = (WRIST, 5, 9, 13, 17)
RETRY_SECONDS = 1.0


def hand_pose(landmarks: list[Landmark], aspect: float = 1.0) -> tuple[float, float, float]:
    """
    Palm centre (u, v) in 0..1 image coordinates and the palm's apparent size.

    `aspect` is image width / height. Size is in units of image height: the
    larger of the palm's length (wrist to middle knuckle) and width (index to
    pinky knuckle), which holds up better than either alone as the hand tilts.
    """
    u = sum(landmarks[i][0] for i in PALM) / len(PALM)
    v = sum(landmarks[i][1] for i in PALM) / len(PALM)

    def span(a: int, b: int) -> float:
        return math.hypot((landmarks[a][0] - landmarks[b][0]) * aspect, landmarks[a][1] - landmarks[b][1])

    return u, v, max(span(WRIST, 9), span(5, 17))


def hand_roll(landmarks: list[Landmark], aspect: float = 1.0) -> float:
    """
    How far the hand is turned in the image, in radians.

    Zero with the fingers pointing straight up, positive as the hand leans to
    the right of the image, measured along the line from the wrist to the
    middle knuckle.
    """
    dx = (landmarks[9][0] - landmarks[WRIST][0]) * aspect
    dy = landmarks[9][1] - landmarks[WRIST][1]
    return math.atan2(dx, -dy)


def hand_message(landmarks: list[Landmark] | None, aspect: float = 1.0) -> dict:
    if landmarks is None:
        return {"hand": False}
    u, v, size = hand_pose(landmarks, aspect)
    servo = {finger: servo_angle(curl(angles)) for finger, angles in all_finger_angles(landmarks).items()}
    return {"hand": True, "pos": [round(u, 4), round(v, 4)], "size": round(size, 4),
            "roll": round(hand_roll(landmarks, aspect), 4), "servo": servo}


def parse_address(text: str) -> tuple[str, int]:
    """'host', 'host:port' or ':port'; missing parts fall back to 127.0.0.1 and DEFAULT_PORT."""
    host, _, port = text.partition(":")
    return host or "127.0.0.1", int(port) if port else DEFAULT_PORT


class SimLink:
    """Connection to the simulation. Frames are dropped, not queued, while it is unreachable."""

    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT):
        self._address = (host, port)
        self._socket = None
        self._next_try = 0.0

    def send(self, landmarks: list[Landmark] | None, aspect: float = 1.0) -> bool:
        """Send one frame's hand (None when no hand is visible). Returns whether it went out."""
        if self._socket is None and not self._connect():
            return False
        try:
            self._socket.sendall((json.dumps(hand_message(landmarks, aspect)) + "\n").encode())
            return True
        except OSError:
            print(f"Lost the simulation at {self._address[0]}:{self._address[1]}; will retry.")
            self.close()
            return False

    def _connect(self) -> bool:
        if time.monotonic() < self._next_try:
            return False
        try:
            self._socket = socket.create_connection(self._address, timeout=0.5)
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"Connected to the simulation at {self._address[0]}:{self._address[1]}.")
            return True
        except OSError:
            if self._next_try == 0.0:
                print(f"Simulation not reachable at {self._address[0]}:{self._address[1]} yet; will keep trying.")
            self._next_try = time.monotonic() + RETRY_SECONDS
            return False

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
