"""
Plot the tracked hand as a 3D skeleton, with each finger's curl and the servo
angle it maps to.

    python plot_hand.py hand.jpg                 # opens a window
    python plot_hand.py hand.jpg --save plot.png

This is a stand-in for the physical claw while the hardware side is in
progress: it shows exactly what would be sent to the servos.
"""

import argparse

import cv2
import matplotlib

from finger_angles import FINGERS, WRIST, all_finger_angles, curl, servo_angle
from hand_tracker import HandTracker

COLORS = {
    "thumb": "#e45756",
    "index": "#f58518",
    "middle": "#54a24b",
    "ring": "#4c78a8",
    "pinky": "#b279a2",
}


def plot_hand(hand, ax_3d, ax_bar) -> None:
    lm = hand.landmarks
    curls = {f: curl(a) for f, a in all_finger_angles(lm).items()}

    # Image y points down and z points away from the camera, so flip both to
    # get a hand that looks upright in the plot.
    def xyz(i):
        x, y, z = lm[i]
        return x, -z, -y

    for finger, joints in FINGERS.items():
        chain = [WRIST, *joints]
        xs, ys, zs = zip(*(xyz(i) for i in chain))
        ax_3d.plot(xs, ys, zs, color=COLORS[finger], linewidth=3, marker="o", markersize=5)
    # Knuckle line across the palm.
    xs, ys, zs = zip(*(xyz(i) for i in (5, 9, 13, 17)))
    ax_3d.plot(xs, ys, zs, color="#888888", linewidth=2)

    ax_3d.set_title(f"{hand.handedness} hand: tracked landmarks")
    ax_3d.set_xticklabels([]); ax_3d.set_yticklabels([]); ax_3d.set_zticklabels([])
    ax_3d.set_box_aspect((1, 0.6, 1))
    ax_3d.view_init(elev=15, azim=-75)

    names = list(curls)
    angles = [servo_angle(curls[f]) for f in names]
    bars = ax_bar.barh(names, angles, color=[COLORS[f] for f in names])
    for bar, finger in zip(bars, names):
        ax_bar.text(bar.get_width() + 3, bar.get_y() + bar.get_height() / 2,
                    f"{bar.get_width():.0f} deg  (curl {curls[finger]:.2f})", va="center", fontsize=9)
    ax_bar.set_xlim(0, 250)
    ax_bar.set_xlabel("servo angle (0 = closed, 180 = open)")
    ax_bar.set_title("Servo command per finger")
    ax_bar.invert_yaxis()
    ax_bar.spines[["top", "right"]].set_visible(False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", help="photo containing a hand")
    parser.add_argument("--save", help="write the plot to this file instead of opening a window")
    args = parser.parse_args()

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    frame = cv2.imread(args.image)
    if frame is None:
        raise SystemExit(f"Could not read image: {args.image}")

    tracker = HandTracker(max_hands=1)
    hands = tracker.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    tracker.close()
    if not hands:
        raise SystemExit("No hand detected.")

    fig = plt.figure(figsize=(10, 4.5))
    ax_3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax_bar = fig.add_subplot(1, 2, 2)
    plot_hand(hands[0], ax_3d, ax_bar)
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=130)
        print(f"Saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
