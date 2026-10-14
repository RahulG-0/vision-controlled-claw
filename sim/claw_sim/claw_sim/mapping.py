"""
How the hand in the camera image maps onto the claw's position in the world.

The tracker reports where the hand is in the (mirrored) image and how big it
looks. Seen from behind the robot, which is where the Gazebo camera starts:

    hand left / right in the image  ->  claw left / right   (world y)
    hand up / down in the image     ->  claw up / down      (world z)
    hand towards the camera         ->  claw reaches forward (world x)
    hand turned in the image        ->  claw rolls the same way about the
                                        direction it points

Distance is judged from the apparent size of the palm, relative to the size
it had when the hand first appeared. Size is inversely proportional to
distance, so the log of the ratio gives equal travel for equal relative moves.
"""

import math
from dataclasses import dataclass


@dataclass
class Workspace:
    # Claw position for a hand in the middle of the image at its reference size.
    centre: tuple[float, float, float] = (0.62, 0.0, 0.50)
    # Claw travel for a hand crossing the full image width / height, metres.
    span_y: float = 0.8
    span_z: float = 0.45
    # Forward travel when the hand looks e (2.7x) times bigger, metres.
    depth_gain: float = 0.35
    # The claw is kept inside this box. The z floor is what stops the hand
    # being driven into the table; it is the floor for an unrolled claw, and
    # `floor` raises or lowers it as the claw rolls.
    x_range: tuple[float, float] = (0.45, 0.78)
    y_range: tuple[float, float] = (-0.36, 0.36)
    z_range: tuple[float, float] = (0.37, 0.66)
    # The claw may roll this far either way from thumb-up, radians. A little
    # past a quarter turn, so palm-down is comfortably inside the range.
    max_roll: float = 1.75
    table_top: float = 0.30
    table_clearance: float = 0.015
    # The open hand's outline around the grasp point, in the hand's own y
    # (out of the palm) and z (across the knuckles), from urdf/hand.xacro:
    # back of the palm to the tip of the open thumb, pinky edge to index edge.
    hand_y: tuple[float, float] = (-0.075, 0.030)
    hand_z: tuple[float, float] = (-0.055, 0.055)

    def clamp_roll(self, roll: float) -> float:
        return min(max(roll, -self.max_roll), self.max_roll)

    def floor(self, roll: float = 0.0) -> float:
        """Lowest height of the grasp point that keeps a claw rolled by `roll` off the table."""
        lowest = min(y * math.sin(roll) + z * math.cos(roll) for y in self.hand_y for z in self.hand_z)
        return self.table_top + self.table_clearance - lowest

    def clamp(self, x: float, y: float, z: float, roll: float = 0.0) -> tuple[float, float, float]:
        return (min(max(x, self.x_range[0]), self.x_range[1]),
                min(max(y, self.y_range[0]), self.y_range[1]),
                min(max(z, self.floor(roll)), self.z_range[1]))

    def image_to_world(self, u: float, v: float, size_ratio: float) -> tuple[float, float, float]:
        """`u`, `v` in 0..1 from the top-left of the mirrored image; size relative to the reference."""
        cx, cy, cz = self.centre
        return self.clamp(cx + self.depth_gain * math.log(max(size_ratio, 1e-3)),
                          cy - (u - 0.5) * self.span_y,
                          cz - (v - 0.5) * self.span_z)

    def world_to_image(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Inverse of `image_to_world` for points inside the box. Used by the scripted demo."""
        cx, cy, cz = self.centre
        return (0.5 - (y - cy) / self.span_y,
                0.5 - (z - cz) / self.span_z,
                math.exp((x - cx) / self.depth_gain))
