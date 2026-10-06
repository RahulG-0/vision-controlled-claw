"""
Hand tracking demo: camera or video in, per-finger curl and servo angles out.

    python main.py                    # webcam 0
    python main.py clip.mp4           # a video file
    python main.py hand.jpg           # a single image
    python main.py clip.mp4 --no-window --save out.mp4

Press q to quit the preview window.
"""

import argparse
from pathlib import Path

import cv2

from finger_angles import all_finger_angles, curl, servo_angle
from hand_tracker import HandTracker

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def annotate(frame, tracker: HandTracker, hands) -> list[dict[str, float]]:
    """Draw each hand and its finger curls; return the curls per hand."""
    all_curls = []
    for n, hand in enumerate(hands):
        tracker.draw(frame, hand)
        curls = {f: curl(a) for f, a in all_finger_angles(hand.landmarks).items()}
        all_curls.append(curls)
        x = 10 + n * 260
        cv2.putText(frame, hand.handedness, (x, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        for i, (finger, value) in enumerate(curls.items()):
            text = f"{finger:<6} {value:.2f} -> {servo_angle(value):5.1f} deg"
            cv2.putText(frame, text, (x, 56 + i * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    return all_curls


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", nargs="?", default="0", help="camera index, video file, or image file")
    parser.add_argument("--hands", type=int, default=1, help="maximum number of hands to track")
    parser.add_argument("--save", help="write the annotated video or image to this path")
    parser.add_argument("--no-window", action="store_true", help="run without a preview window")
    args = parser.parse_args()

    tracker = HandTracker(max_hands=args.hands)

    if Path(args.source).suffix.lower() in IMAGE_SUFFIXES:
        frame = cv2.imread(args.source)
        if frame is None:
            raise SystemExit(f"Could not read image: {args.source}")
        hands = tracker.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        for hand, curls in zip(hands, annotate(frame, tracker, hands)):
            print(hand.handedness, curls)
        if not hands:
            print("No hand detected.")
        if args.save:
            cv2.imwrite(args.save, frame)
        tracker.close()
        return

    is_camera = args.source.isdigit()
    cap = cv2.VideoCapture(int(args.source) if is_camera else args.source)
    if not cap.isOpened():
        raise SystemExit(f"Could not open source: {args.source}")

    writer = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if is_camera:
                frame = cv2.flip(frame, 1)  # mirror, so it behaves like a mirror
            hands = tracker.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            annotate(frame, tracker, hands)

            if args.save:
                if writer is None:
                    h, w = frame.shape[:2]
                    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
                    writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
                writer.write(frame)
            if not args.no_window:
                cv2.imshow("Vision-Controlled Claw", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        tracker.close()


if __name__ == "__main__":
    main()
