"""
Drive the simulated arm and claw from the hand tracker.

The tracker (src/main.py --sim) connects over TCP and sends one JSON object
per line for each camera frame:

    {"hand": true, "pos": [u, v], "size": s, "roll": rad, "servo": {"thumb": deg, ...}}
    {"hand": false}

`pos`, `size` and `roll` steer the arm (see mapping.py); `servo` holds the
angle the real claw's servo would get for each finger, 0 closed to 180 open.
`roll` may be left out, which keeps the claw thumb-up. While no hand is
visible the arm and claw hold their last pose.

TCP rather than a ROS topic so the tracker can run outside ROS, including on
Windows next to the webcam while the simulation runs in WSL2.
"""

import json
import socket
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from claw_sim.mapping import Workspace
from claw_sim.urdf_chain import Chain, revolute_limits

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
RATE_HZ = 50.0
# Frames averaged to fix the reference hand size when a hand first appears.
SIZE_REFERENCE_FRAMES = 10
MAX_ROLL_SPEED = 2.5  # rad/s


def roll_matrix(angle: float) -> np.ndarray:
    """Rotation about world x, the direction the claw points."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


class Teleop(Node):
    def __init__(self):
        super().__init__("claw_teleop")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("base_link", "world")
        self.declare_parameter("tip_link", "grasp_point")
        self.declare_parameter("port", 8765)
        # A posture near the wanted one, with the hand level and pointing
        # forward. The solver starts from it and keeps the arm close to it.
        self.declare_parameter("seed", [0.0, 0.4, 0.0, 1.3, 0.0, -0.13, 0.0])
        self.declare_parameter("max_speed", 0.6)  # of the claw, m/s

        urdf = self.get_parameter("robot_description").value
        if not urdf:
            raise RuntimeError("the robot_description parameter is empty")
        self.chain = Chain(urdf, self.get_parameter("base_link").value,
                           self.get_parameter("tip_link").value)
        self.workspace = Workspace()
        self.max_speed = self.get_parameter("max_speed").value

        seed = np.array(self.get_parameter("seed").value)
        # The claw's orientation in the seed posture is its thumb-up pose;
        # the hand's roll is applied on top of it.
        self.rotation = self.chain.forward(seed)[0][:3, :3]
        self.roll = 0.0
        self.roll_goal = 0.0
        self.position = np.array(self.workspace.centre)
        self.goal = self.position.copy()
        self.rest = self.chain.solve(seed, self.position, self.rotation, seed)
        self.q = self.rest.copy()

        # Each finger's joints, with the angle they reach when fully curled.
        self.finger_joints = {
            name: upper for name, (_, upper) in revolute_limits(urdf).items()
            if name not in self.chain.names and name.split("_")[0] in FINGERS
        }
        self.curl = dict.fromkeys(FINGERS, 0.0)
        self.curl_goal = dict(self.curl)

        self.publishers_ = {
            name: self.create_publisher(Float64, f"/claw/{name}/cmd", 10)
            for name in [*self.chain.names, *self.finger_joints]
        }

        self._lock = threading.Lock()
        self._message = None
        self._size_samples = []
        self._size_reference = None
        threading.Thread(target=self._serve, daemon=True).start()
        self.create_timer(1.0 / RATE_HZ, self._tick)

    def _serve(self) -> None:
        port = self.get_parameter("port").value
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", port))
        server.listen(1)
        self.get_logger().info(f"waiting for the hand tracker on TCP port {port}")
        while True:
            connection, address = server.accept()
            self.get_logger().info(f"hand tracker connected from {address[0]}")
            with connection, connection.makefile("r", encoding="utf-8") as lines:
                try:
                    for line in lines:
                        try:
                            message = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        with self._lock:
                            self._message = message
                except OSError:
                    pass
            self.get_logger().info("hand tracker disconnected; holding pose")

    def _apply(self, message: dict) -> None:
        """Turn one tracker message into goals for the claw position and finger curls."""
        if not message.get("hand"):
            return
        size = float(message["size"])
        if self._size_reference is None:
            self._size_samples.append(size)
            if len(self._size_samples) < SIZE_REFERENCE_FRAMES:
                return
            self._size_reference = float(np.mean(self._size_samples))
            self.get_logger().info("hand found; its current distance is now the middle of the reach")
        u, v = message["pos"]
        self.roll_goal = self.workspace.clamp_roll(float(message.get("roll", 0.0)))
        self.goal = np.array(self.workspace.image_to_world(u, v, size / self._size_reference))
        for finger, degrees in message.get("servo", {}).items():
            if finger in self.curl_goal:
                self.curl_goal[finger] = 1.0 - min(max(float(degrees), 0.0), 180.0) / 180.0

    def _tick(self) -> None:
        with self._lock:
            message, self._message = self._message, None
        if message is not None:
            self._apply(message)

        # Ease towards the goal, which hides tracker jitter, under a speed cap.
        step = 0.2 * (self.goal - self.position)
        distance = np.linalg.norm(step)
        limit = self.max_speed / RATE_HZ
        if distance > limit:
            step *= limit / distance
        self.position += step
        turn = 0.2 * (self.roll_goal - self.roll)
        self.roll += float(np.clip(turn, -MAX_ROLL_SPEED / RATE_HZ, MAX_ROLL_SPEED / RATE_HZ))
        # Rolling swings the edge of the hand down, so the table limit moves with it.
        self.position = np.array(self.workspace.clamp(*self.position, self.roll))
        rotation = roll_matrix(self.roll) @ self.rotation
        for _ in range(2):
            self.q = self.chain.ik_step(self.q, self.position, rotation, self.rest)
        for name, angle in zip(self.chain.names, self.q):
            self.publishers_[name].publish(Float64(data=float(angle)))

        for finger in FINGERS:
            self.curl[finger] += 0.35 * (self.curl_goal[finger] - self.curl[finger])
        for name, upper in self.finger_joints.items():
            self.publishers_[name].publish(Float64(data=self.curl[name.split("_")[0]] * upper))


def main() -> None:
    rclpy.init()
    try:
        rclpy.spin(Teleop())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
