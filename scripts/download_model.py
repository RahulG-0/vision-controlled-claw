"""Download the MediaPipe hand landmark model (about 8 MB) into models/."""

import urllib.request
from pathlib import Path

URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
DEST = Path(__file__).parent.parent / "models" / "hand_landmarker.task"

if __name__ == "__main__":
    DEST.parent.mkdir(exist_ok=True)
    if DEST.exists():
        print(f"Already present: {DEST}")
    else:
        print(f"Downloading to {DEST} ...")
        urllib.request.urlretrieve(URL, DEST)
        print("Done.")
