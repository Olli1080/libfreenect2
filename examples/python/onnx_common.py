"""Shared ONNX Runtime + OpenCV helpers for the YOLOv8-family example apps
(onnx_object_detection.py, onnx_pose_estimation.py). Python port of
examples/onnx_common.h / onnx_common.cpp.

CPU execution provider only, matching the C++ examples (see onnx_common.h's
file header comment for why: DirectML hit a version wall, CUDA wasn't
pursued once CPU was chosen instead).
"""

from __future__ import annotations

import numpy as np
import cv2
import onnxruntime as ort


def create_session(model_path: str) -> ort.InferenceSession:
    """Create a CPU inference session for the model at model_path. Raises on
    failure (bad path, malformed model, unsupported ops) -- callers should
    catch and report, not crash."""
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    return ort.InferenceSession(model_path, sess_options=so, providers=["CPUExecutionProvider"])


class LetterboxInfo:
    """Describes letterbox()'s transform so results can be mapped back to
    the source image's coordinate space afterward."""

    __slots__ = ("scale", "pad_x", "pad_y")

    def __init__(self, scale: float = 1.0, pad_x: int = 0, pad_y: int = 0):
        self.scale = scale
        self.pad_x = pad_x
        self.pad_y = pad_y


def letterbox(src: np.ndarray, model_size: int) -> tuple[np.ndarray, LetterboxInfo]:
    """Aspect-ratio-preserving resize + pad to model_size x model_size,
    matching Ultralytics' own YOLOv8 preprocessing convention so a stock
    export works unmodified."""
    h, w = src.shape[:2]
    scale = min(model_size / w, model_size / h)
    new_w, new_h = round(w * scale), round(h * scale)
    resized = cv2.resize(src, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_x = (model_size - new_w) // 2
    pad_y = (model_size - new_h) // 2
    pad_right = model_size - new_w - pad_x
    pad_bottom = model_size - new_h - pad_y

    dst = cv2.copyMakeBorder(resized, pad_y, pad_bottom, pad_x, pad_right,
                              cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return dst, LetterboxInfo(scale, pad_x, pad_y)


def to_chw_tensor(letterboxed_bgr: np.ndarray) -> np.ndarray:
    """Converts a letterboxed BGR (or BGRA/BGRX, alpha ignored) uint8 image
    into a normalized [0,1] RGB float32 NCHW tensor (N=1), the layout
    YOLOv8 ONNX exports expect."""
    if letterboxed_bgr.shape[2] == 4:
        rgb = cv2.cvtColor(letterboxed_bgr, cv2.COLOR_BGRA2RGB)
    else:
        rgb = cv2.cvtColor(letterboxed_bgr, cv2.COLOR_BGR2RGB)
    chw = rgb.astype(np.float32).transpose(2, 0, 1) / 255.0
    return np.ascontiguousarray(chw[np.newaxis, ...])


def map_box_to_original(cx: float, cy: float, w: float, h: float, info: LetterboxInfo) -> tuple[int, int, int, int]:
    """Maps a box in letterboxed model-input pixel space back to
    letterbox()'s original image space. Returns (x1, y1, x2, y2)."""
    x1 = (cx - w * 0.5 - info.pad_x) / info.scale
    y1 = (cy - h * 0.5 - info.pad_y) / info.scale
    x2 = (cx + w * 0.5 - info.pad_x) / info.scale
    y2 = (cy + h * 0.5 - info.pad_y) / info.scale
    return int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))


def map_point_to_original(x: float, y: float, info: LetterboxInfo) -> tuple[float, float]:
    """Maps a single point (e.g. a keypoint) the same way as map_box_to_original."""
    return (x - info.pad_x) / info.scale, (y - info.pad_y) / info.scale


def nms(boxes: list[tuple[int, int, int, int]], scores: list[float], iou_threshold: float) -> list[int]:
    """Greedy NMS, boxes as (x1, y1, x2, y2). Returns indices into
    boxes/scores to keep, highest score first. A direct port of
    onnx_common.cpp's own loop (not cv2.dnn.NMSBoxes) so tie-breaking and
    iteration order match exactly."""
    order = sorted(range(len(boxes)), key=lambda i: scores[i], reverse=True)
    areas = [max(0, b[2] - b[0]) * max(0, b[3] - b[1]) for b in boxes]
    suppressed = [False] * len(boxes)
    keep: list[int] = []

    for oi in range(len(order)):
        i = order[oi]
        if suppressed[i]:
            continue
        keep.append(i)

        for oj in range(oi + 1, len(order)):
            j = order[oj]
            if suppressed[j]:
                continue
            xx1 = max(boxes[i][0], boxes[j][0])
            yy1 = max(boxes[i][1], boxes[j][1])
            xx2 = min(boxes[i][2], boxes[j][2])
            yy2 = min(boxes[i][3], boxes[j][3])
            inter = max(0, xx2 - xx1) * max(0, yy2 - yy1)
            union = areas[i] + areas[j] - inter
            iou = inter / union if union > 0 else 0.0
            if iou > iou_threshold:
                suppressed[j] = True

    return keep


COCO_CLASS_NAMES: tuple[str, ...] = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
)

# 0-indexed COCO 17-keypoint skeleton connections (nose, eyes, ears, shoulders, elbows,
# wrists, hips, knees, ankles), Ultralytics' standard edge list.
COCO_SKELETON_EDGES: tuple[tuple[int, int], ...] = (
    (15, 13), (13, 11), (16, 14), (14, 12), (11, 12), (5, 11), (6, 12), (5, 6), (5, 7),
    (6, 8), (7, 9), (8, 10), (1, 2), (0, 1), (0, 2), (1, 3), (2, 4), (3, 5), (4, 6),
)
