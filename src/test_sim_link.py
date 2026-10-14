"""Tests for the simulation link: the hand pose maths and the wire format."""

import json
import socket

import math

from sim_link import SimLink, hand_message, hand_pose, hand_roll, parse_address


def open_hand(cx=0.5, cy=0.5, scale=1.0):
    """A flat upright hand: wrist at the bottom, straight fingers pointing up the image."""
    landmarks = [(0.0, 0.0, 0.0)] * 21
    landmarks[0] = (cx, cy + 0.10 * scale, 0.0)
    for base, dx in ((1, -0.08), (5, -0.04), (9, 0.0), (13, 0.04), (17, 0.08)):
        for i in range(4):
            landmarks[base + i] = (cx + dx * scale, cy - (0.10 + 0.05 * i) * scale, 0.0)
    return landmarks


def test_pose_follows_the_hand():
    u, v, _ = hand_pose(open_hand(0.5, 0.5))
    u2, v2, _ = hand_pose(open_hand(0.7, 0.4))
    assert abs((u2 - u) - 0.2) < 1e-9
    assert abs((v2 - v) + 0.1) < 1e-9


def test_size_scales_with_the_hand_and_ignores_curl():
    small = hand_pose(open_hand(scale=1.0))[2]
    assert abs(hand_pose(open_hand(scale=2.0))[2] - 2 * small) < 1e-9
    fist = open_hand()
    for tip in (8, 12, 16, 20):
        fist[tip] = fist[0]
    assert hand_pose(fist)[2] == small


def turned(landmarks, degrees, cx=0.5, cy=0.5):
    """The same hand turned clockwise on screen (image y points down) about (cx, cy)."""
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    return [(cx + (x - cx) * c - (y - cy) * s, cy + (x - cx) * s + (y - cy) * c, z) for x, y, z in landmarks]


def test_roll_is_zero_upright_and_positive_leaning_right():
    assert abs(hand_roll(open_hand())) < 1e-9
    assert abs(hand_roll(turned(open_hand(), 30)) - math.radians(30)) < 1e-9
    assert abs(hand_roll(turned(open_hand(), -90)) + math.radians(90)) < 1e-9
    assert hand_message(turned(open_hand(), 30))["roll"] == round(math.radians(30), 4)


def test_message_for_an_open_hand_opens_every_finger():
    message = hand_message(open_hand())
    assert message["hand"] is True
    assert set(message["servo"]) == {"thumb", "index", "middle", "ring", "pinky"}
    assert message["servo"]["middle"] == 180.0
    # The outer fingers splay from the wrist, so they read as very slightly bent.
    assert all(message["servo"][f] > 150.0 for f in ("index", "ring", "pinky"))
    assert hand_message(None) == {"hand": False}


def test_parse_address():
    assert parse_address("127.0.0.1") == ("127.0.0.1", 8765)
    assert parse_address("10.0.0.5:9000") == ("10.0.0.5", 9000)
    assert parse_address(":9000") == ("127.0.0.1", 9000)


def test_link_sends_one_json_line_per_frame():
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    link = SimLink("127.0.0.1", server.getsockname()[1])
    assert link.send(open_hand())
    assert link.send(None)
    link.close()
    connection, _ = server.accept()
    lines = connection.makefile().read().splitlines()
    assert [json.loads(line)["hand"] for line in lines] == [True, False]
    connection.close()
    server.close()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  ", name)
