"""
Scripted pick-up, for trying the simulation without a camera.

It plays the part of the hand tracker: it connects to the teleop node and
sends the same messages a real hand would produce while reaching beside an
object, closing on it, lifting it, carrying it and setting it down.

    ros2 run claw_sim demo            # picks up the can
    ros2 run claw_sim demo block
"""

import argparse
import json
import socket
import time

from claw_sim.mapping import Workspace

# Where the objects stand in worlds/pick.sdf.
OBJECTS = {
    "can": (0.68, 0.20, 0.37),
    "block": (0.68, -0.02, 0.37),
    "bottle": (0.68, -0.26, 0.37),
}
OPEN, CLOSED = 180.0, 0.0
REFERENCE_SIZE = 0.2
RATE_HZ = 30.0


def plan(x: float, y: float, z: float) -> list[tuple[float, tuple[float, float, float], float]]:
    """(seconds, claw position, servo angle) steps. The palm faces +y, so the claw comes in from -y."""
    centre = Workspace().centre
    beside = (x, y - 0.10, z)
    lifted = (x, y, z + 0.18)
    carried = (x - 0.12, y, z + 0.18)
    put_down = (x - 0.12, y, z + 0.005)
    clear = (put_down[0], put_down[1] - 0.10, put_down[2])
    return [
        (1.5, centre, OPEN),
        (2.0, (centre[0], y - 0.10, centre[2]), OPEN),
        (2.5, beside, OPEN),
        (2.0, (x, y, z), OPEN),
        (1.5, (x, y, z), CLOSED),
        (2.0, lifted, CLOSED),
        (2.5, carried, CLOSED),
        (2.0, carried, CLOSED),
        (2.5, put_down, CLOSED),
        (1.5, put_down, OPEN),
        (2.0, clear, OPEN),
        # Rise before heading back, so the open hand does not sweep the object over.
        (1.5, (clear[0], clear[1], centre[2] + 0.1), OPEN),
        (2.5, centre, OPEN),
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("object", nargs="?", default="can", choices=OBJECTS)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    workspace = Workspace()
    steps = plan(*OBJECTS[args.object])
    position, servo = steps[0][1], steps[0][2]
    with socket.create_connection((args.host, args.port)) as connection:
        for seconds, target, target_servo in steps:
            frames = max(1, round(seconds * RATE_HZ))
            for i in range(1, frames + 1):
                t = i / frames
                now = [a + t * (b - a) for a, b in zip(position, target)]
                u, v, ratio = workspace.world_to_image(*now)
                degrees = servo + t * (target_servo - servo)
                message = {"hand": True, "pos": [u, v], "size": REFERENCE_SIZE * ratio,
                           "servo": dict.fromkeys(("thumb", "index", "middle", "ring", "pinky"), degrees)}
                connection.sendall((json.dumps(message) + "\n").encode())
                time.sleep(1.0 / RATE_HZ)
            position, servo = target, target_servo


if __name__ == "__main__":
    main()
