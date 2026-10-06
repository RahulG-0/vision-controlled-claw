"""Tests for the joint-angle math, using hand-written landmark coordinates."""

from finger_angles import all_finger_angles, angle_at, curl, finger_joint_angles, servo_angle


def hand_with_index(mcp, pip, dip, tip):
    landmarks = [(0.0, 0.0, 0.0)] * 21
    landmarks[0] = (0.5, 1.0, 0.0)  # wrist
    landmarks[5], landmarks[6], landmarks[7], landmarks[8] = mcp, pip, dip, tip
    return landmarks


STRAIGHT = hand_with_index((0.5, 0.8, 0), (0.5, 0.6, 0), (0.5, 0.4, 0), (0.5, 0.2, 0))
CURLED = hand_with_index((0.5, 0.8, 0), (0.5, 0.6, 0), (0.6, 0.6, 0), (0.6, 0.8, 0))


def test_right_angle():
    assert abs(angle_at((1, 0, 0), (0, 0, 0), (0, 1, 0)) - 90.0) < 1e-6


def test_straight_finger_is_180_at_every_joint():
    angles = finger_joint_angles(STRAIGHT, "index")
    assert set(angles) == {"mcp", "pip", "dip"}
    assert all(abs(a - 180.0) < 0.1 for a in angles.values())


def test_curled_finger_bends_at_pip_and_dip():
    angles = finger_joint_angles(CURLED, "index")
    assert abs(angles["pip"] - 90.0) < 0.1
    assert abs(angles["dip"] - 90.0) < 0.1


def test_curl_is_lower_for_a_curled_finger():
    straight = curl(finger_joint_angles(STRAIGHT, "index"))
    curled = curl(finger_joint_angles(CURLED, "index"))
    assert straight == 1.0
    assert curled < straight


def test_thumb_uses_its_own_joint_names():
    assert set(all_finger_angles(STRAIGHT)["thumb"]) == {"cmc", "mcp", "ip"}


def test_degenerate_points_do_not_crash():
    assert angle_at((0, 0, 0), (0, 0, 0), (0, 0, 0)) == 180.0


def test_servo_mapping_is_clamped_and_linear():
    assert servo_angle(0.2) == 0.0
    assert servo_angle(1.0) == 180.0
    assert servo_angle(0.76) == 90.0


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  ", name)
