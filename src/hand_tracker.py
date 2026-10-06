"""
Thin wrapper around MediaPipe's HandLandmarker (Tasks API).

MediaPipe returns 21 3D landmarks per detected hand (the wrist plus four
joints per finger), normalized to [0, 1] relative to the image. This module
handles model setup and converts results into plain (x, y, z) tuples so the
rest of the pipeline doesn't touch the MediaPipe API directly.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.core.base_options import BaseOptions

Landmark = tuple[float, float, float]

DEFAULT_MODEL_PATH = Path(__file__).parent.parent / "models" / "hand_landmarker.task"

HAND_CONNECTIONS = vision.HandLandmarksConnections.HAND_CONNECTIONS


@dataclass
class HandResult:
    landmarks: list[Landmark]  # 21 entries, indexed as in finger_angles.FINGERS
    handedness: str  # "Left" or "Right"


class HandTracker:
    def __init__(
        self,
        model_path: Path = DEFAULT_MODEL_PATH,
        max_hands: int = 1,
        detection_confidence: float = 0.6,
        tracking_confidence: float = 0.6,
        fps: float = 30.0,
    ):
        if not Path(model_path).exists():
            raise FileNotFoundError(
                f"{model_path} not found. Run scripts/download_model.py first."
            )
        options = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=max_hands,
            min_hand_detection_confidence=detection_confidence,
            min_tracking_confidence=tracking_confidence,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._frame_ms = 1000.0 / fps
        self._frame_index = 0

    def process(self, frame_rgb) -> list[HandResult]:
        """Run detection on one RGB frame and return the landmarks of each hand."""
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)

        # VIDEO mode requires strictly increasing timestamps.
        timestamp_ms = int(self._frame_index * self._frame_ms)
        self._frame_index += 1

        result = self._landmarker.detect_for_video(image, timestamp_ms)
        hands = []
        for i, hand_landmarks in enumerate(result.hand_landmarks):
            label = "Unknown"
            if result.handedness and i < len(result.handedness):
                label = result.handedness[i][0].category_name
            hands.append(
                HandResult(
                    landmarks=[(lm.x, lm.y, lm.z) for lm in hand_landmarks],
                    handedness=label,
                )
            )
        return hands

    @staticmethod
    def draw(frame_bgr, hand: HandResult) -> None:
        """Draw the hand skeleton onto a BGR frame in place."""
        h, w = frame_bgr.shape[:2]
        points = [(int(x * w), int(y * h)) for x, y, _ in hand.landmarks]
        for conn in HAND_CONNECTIONS:
            cv2.line(frame_bgr, points[conn.start], points[conn.end], (255, 255, 255), 2)
        for point in points:
            cv2.circle(frame_bgr, point, 4, (0, 200, 255), -1)

    def close(self) -> None:
        self._landmarker.close()
