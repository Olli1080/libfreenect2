"""Loads the MediaPipe palm-detector SSD anchor grid from the single source
of truth already used by the C++ build: examples/mediapipe_palm_anchors.inc.h
(2016 entries, opencv/opencv_zoo's palm_detection_mediapipe demo.cpp,
Apache-2.0). Parsed directly from that file rather than hand-transcribed, so
there's no risk of a copy-paste mismatch and any future update to the data
flows to Python automatically.
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np

_ANCHORS_PATH = Path(__file__).resolve().parent.parent / "mediapipe_palm_anchors.inc.h"


def _load() -> np.ndarray:
    text = _ANCHORS_PATH.read_text()
    pairs = re.findall(r"\{\s*([-\d.]+)f?\s*,\s*([-\d.]+)f?\s*\}", text)
    arr = np.array(pairs, dtype=np.float32)
    if arr.shape != (2016, 2):
        raise ValueError(f"expected 2016x2 anchors from {_ANCHORS_PATH}, got {arr.shape}")
    return arr


PALM_ANCHORS: np.ndarray = _load()
