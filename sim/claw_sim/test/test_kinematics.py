"""Tests for the URDF kinematics and the image-to-world mapping. No ROS or Gazebo needed beyond xacro."""

from pathlib import Path

import numpy as np
import pytest

from claw_sim.mapping import Workspace
from claw_sim.urdf_chain import Chain, revolute_limits

xacro = pytest.importorskip("xacro")

URDF = xacro.process_file(str(Path(__file__).parent.parent / "urdf" / "claw_arm.urdf.xacro")).toxml()
SEED = np.array([0.0, 0.4, 0.0, 1.3, 0.0, -0.13, 0.0])


@pytest.fixture(scope="module")
def chain():
    return Chain(URDF, "world", "grasp_point")


def test_chain_has_seven_revolute_joints(chain):
    assert chain.names == [f"joint{i}" for i in range(1, 8)]


def test_straight_arm_points_up(chain):
    pose, _ = chain.forward(np.zeros(7))
    # Flange at 0.34 + 0.40 + 0.40 + 0.125, then the grasp point's own offset.
    assert np.allclose(pose[:3, 3], [-0.0, 0.047, 1.265 + 0.068], atol=1e-6)


def test_seed_posture_holds_the_hand_level(chain):
    rotation = chain.forward(SEED)[0][:3, :3]
    assert np.allclose(rotation, np.eye(3), atol=1e-3)


def test_jacobian_matches_finite_differences(chain):
    q = np.array([0.3, 0.5, -0.4, 1.1, 0.2, -0.6, 0.7])
    pose, jacobian = chain.forward(q)
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = 1e-6
        moved = chain.forward(q + dq)[0]
        assert np.allclose((moved[:3, 3] - pose[:3, 3]) / 1e-6, jacobian[:3, i], atol=1e-4)


def test_ik_reaches_every_corner_of_the_workspace(chain):
    workspace = Workspace()
    rest = chain.solve(SEED, np.array(workspace.centre), np.eye(3), SEED)
    for x in workspace.x_range:
        for y in workspace.y_range:
            for z in workspace.z_range:
                target = np.array([x, y, z])
                pose = chain.forward(chain.solve(rest, target, np.eye(3), rest))[0]
                assert np.linalg.norm(pose[:3, 3] - target) < 2e-3, target
                assert np.allclose(pose[:3, :3], np.eye(3), atol=2e-2), target


def test_ik_reaches_the_workspace_at_full_roll(chain):
    workspace = Workspace()
    rest = chain.solve(SEED, np.array(workspace.centre), np.eye(3), SEED)
    for roll in (-workspace.max_roll, workspace.max_roll):
        c, s = np.cos(roll), np.sin(roll)
        rotation = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        for x in workspace.x_range:
            for y in workspace.y_range:
                for z in (workspace.floor(roll), workspace.z_range[1]):
                    target = np.array([x, y, z])
                    pose = chain.forward(chain.solve(rest, target, rotation, rest))[0]
                    assert np.linalg.norm(pose[:3, 3] - target) < 2e-3, (roll, target)
                    assert np.allclose(pose[:3, :3], rotation, atol=2e-2), (roll, target)


def test_table_floor_follows_the_roll():
    workspace = Workspace()
    assert abs(workspace.floor(0.0) - workspace.z_range[0]) < 1e-9
    # Palm down, the back of the hand is on top and the open thumb hangs lowest.
    assert abs(workspace.floor(-np.pi / 2) - (0.315 + 0.030)) < 1e-9
    # Palm up, the back of the hand is underneath.
    assert abs(workspace.floor(np.pi / 2) - (0.315 + 0.075)) < 1e-9


def test_finger_joints_are_named_after_their_finger():
    fingers = {name.split("_")[0] for name in revolute_limits(URDF) if not name.startswith("joint")}
    assert fingers == {"thumb", "index", "middle", "ring", "pinky"}


def test_mapping_round_trip():
    workspace = Workspace()
    point = (0.65, 0.20, 0.40)
    assert np.allclose(workspace.image_to_world(*workspace.world_to_image(*point)), point)


def test_mapping_directions():
    workspace = Workspace()
    x0, y0, z0 = workspace.image_to_world(0.5, 0.5, 1.0)
    assert workspace.image_to_world(0.7, 0.5, 1.0)[1] < y0  # hand right -> claw right (-y)
    assert workspace.image_to_world(0.5, 0.3, 1.0)[2] > z0  # hand up -> claw up
    assert workspace.image_to_world(0.5, 0.5, 1.5)[0] > x0  # hand closer -> claw forward
