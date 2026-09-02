#!/usr/bin/env python3
"""pyfreenect2 example -- a Python port of examples/ONNXHandTracking.cpp.

Two-stage MediaPipe hand tracking (palm detector + 21-point hand landmark
model) on the Kinect's color stream, drawing hand skeletons in an OpenCV
window. Ported from the same opencv/opencv_zoo reference implementations
the C++ version cites (palm_detection_mediapipe, handpose_estimation_mediapipe,
Apache-2.0) -- same math, same constants, translated to onnxruntime-python +
cv2 (calling cv2.getRotationMatrix2D/warpAffine/floodFill/etc. directly,
the same OpenCV implementations the C++ version calls, rather than
reimplementing that math in numpy -- the parts most likely to have a subtle
bug if hand-rewritten).

Usage:
    python onnx_hand_tracking.py --palm-model <path> --hand-model <path>
        [--palm-conf <0..1>] [--palm-iou <0..1>] [--hand-conf <0..1>]
        [--min-depth <meters>] [--max-depth <meters>] [--max-hands <n>]
        [--pythonpath <dir>] [--duration <seconds>]

Palm detection never runs on the raw full color frame -- see the C++ file's
header comment for why (a distant hand is too small in pixels for the palm
CNN to see). Instead an already-tracked hand gets a tight crop+upscale each
frame, and a new hand is (re)acquired via the depth sensor picking out
plausible-distance blobs to crop around.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np

import onnx_common
from mediapipe_palm_anchors import PALM_ANCHORS

PI = math.pi

# --- Stage 1: palm detection (192x192 NHWC input) ---------------------------

PALM_INPUT_SIZE = 192
PALM_NUM_ANCHORS = 2016


@dataclass
class PalmDetection:
    box: tuple[float, float, float, float]  # x, y, w, h -- original image space
    landmarks: np.ndarray  # (7, 2) -- original image space
    score: float


def preprocess_for_palm(bgrx: np.ndarray) -> tuple[np.ndarray, tuple[float, float], float]:
    """NHWC RGB float32 [0,1], black-padded aspect-preserving resize. Returns
    (tensor[1,192,192,3], pad_bias_in_original_pixels, ratio)."""
    h, w = bgrx.shape[:2]
    ratio = min(PALM_INPUT_SIZE / w, PALM_INPUT_SIZE / h)
    new_w, new_h = int(w * ratio), int(h * ratio)
    resized = cv2.resize(bgrx, (new_w, new_h))

    pad_w = PALM_INPUT_SIZE - new_w
    pad_h = PALM_INPUT_SIZE - new_h
    pad_x = pad_w // 2
    pad_y = pad_h // 2

    padded = cv2.copyMakeBorder(resized, pad_y, pad_h - pad_y, pad_x, pad_w - pad_x,
                                 cv2.BORDER_CONSTANT, value=(0, 0, 0))
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGRA2RGB if padded.shape[2] == 4 else cv2.COLOR_BGR2RGB)

    tensor = (rgb.astype(np.float32) / 255.0)[np.newaxis, ...]
    pad_bias = (pad_x / ratio, pad_y / ratio)
    return tensor, pad_bias, ratio


def detect_palms(session, input_name, output_names, bgrx: np.ndarray,
                  score_threshold: float, iou_threshold: float) -> list[PalmDetection]:
    tensor, pad_bias, _ratio = preprocess_for_palm(bgrx)
    outputs = session.run(output_names, {input_name: tensor})
    boxes_out = outputs[0][0]   # (2016, 18)
    scores_out = outputs[1][0]  # (2016, 1)

    scores_raw = scores_out[:, 0]
    scores_sig = 1.0 / (1.0 + np.exp(-scores_raw))

    scale = float(max(bgrx.shape[0], bgrx.shape[1]))
    anchors = PALM_ANCHORS  # (2016, 2)

    cxy_delta = boxes_out[:, 0:2] / PALM_INPUT_SIZE
    wh_delta = boxes_out[:, 2:4] / PALM_INPUT_SIZE
    xy1 = (cxy_delta - wh_delta * 0.5 + anchors) * scale - pad_bias
    xy2 = (cxy_delta + wh_delta * 0.5 + anchors) * scale - pad_bias

    landmark_delta = boxes_out[:, 4:18].reshape(PALM_NUM_ANCHORS, 7, 2) / PALM_INPUT_SIZE
    landmarks_all = (landmark_delta + anchors[:, np.newaxis, :]) * scale - np.asarray(pad_bias)

    mask = scores_sig > score_threshold
    idxs = np.nonzero(mask)[0]

    nms_boxes: list[tuple[int, int, int, int]] = []
    nms_scores: list[float] = []
    nms_landmarks: list[np.ndarray] = []

    for i in idxs:
        x1, y1 = int(xy1[i, 0]), int(xy1[i, 1])
        x2, y2 = int(xy2[i, 0]), int(xy2[i, 1])
        nms_boxes.append((x1, y1, x2, y2))
        nms_scores.append(float(scores_sig[i]))
        nms_landmarks.append(landmarks_all[i])

    keep = onnx_common.nms(nms_boxes, nms_scores, iou_threshold)

    result = []
    for idx in keep:
        x1, y1, x2, y2 = nms_boxes[idx]
        result.append(PalmDetection(box=(float(x1), float(y1), float(x2 - x1), float(y2 - y1)),
                                     landmarks=nms_landmarks[idx].copy(),
                                     score=nms_scores[idx]))
    return result


# --- Between stage 1 and 2: crop, rotate upright, pad, resize to 224 --------

HAND_INPUT_SIZE = 224
PALM_LANDMARK_INDEX_OF_PALM_BASE = 0
PALM_LANDMARK_INDEX_OF_MIDDLE_FINGER_BASE = 2
PALM_BOX_PRE_SHIFT_VECTOR = np.array([0.0, 0.0])
PALM_BOX_PRE_ENLARGE_FACTOR = 4.0
PALM_BOX_SHIFT_VECTOR = np.array([0.0, -0.4])
PALM_BOX_ENLARGE_FACTOR = 3.0
HAND_BOX_SHIFT_VECTOR = np.array([0.0, -0.1])
HAND_BOX_ENLARGE_FACTOR = 1.65

# --- Temporal tracking + depth-guided acquisition ---------------------------

TRACKING_CROP_MARGIN_FACTOR = 3.0
TRACKING_CROP_MIN_SIZE = 100
MAX_MISSED_FRAMES = 5
TRACK_OVERLAP_IOU_THRESHOLD = 0.3
HAND_FOOTPRINT_METERS = 0.35
DEPTH_CANDIDATE_MIN_CROP_SIZE = 100
DEPTH_CANDIDATE_MAX_CROP_SIZE = 900
FLOOD_DEPTH_TOLERANCE_MM = 60.0
DEPTH_CANDIDATE_SEARCH_HEADROOM = 6


def _clamp(v, lo, hi):
    return max(lo, min(v, hi))


def crop_and_pad_from_palm(image: np.ndarray, box: np.ndarray, shift_vector: np.ndarray,
                            enlarge_factor: float, for_rotation: bool):
    """box: (2,2) array [[x0,y0],[x1,y1]]. Returns (padded_image, box_in_source_space, bias)."""
    wh = box[1] - box[0]
    shift = shift_vector * wh
    box = box + shift

    center = (box[0] + box[1]) * 0.5
    wh = box[1] - box[0]
    half = wh * (enlarge_factor * 0.5)
    box = np.array([center - half, center + half])

    img_w, img_h = image.shape[1], image.shape[0]
    tl_x = _clamp(int(box[0, 0]), 0, img_w)
    tl_y = _clamp(int(box[0, 1]), 0, img_h)
    br_x = _clamp(int(box[1, 0]), tl_x + 1, img_w)
    br_y = _clamp(int(box[1, 1]), tl_y + 1, img_h)

    crop = image[tl_y:br_y, tl_x:br_x].copy()

    if for_rotation:
        side_len_f = math.sqrt(float(crop.shape[0]) ** 2 + float(crop.shape[1]) ** 2)
    else:
        side_len_f = float(max(crop.shape[0], crop.shape[1]))
    side = int(side_len_f)
    pad_h = max(0, side - crop.shape[0])
    pad_w = max(0, side - crop.shape[1])
    left, top = pad_w // 2, pad_h // 2
    right, bottom = pad_w - left, pad_h - top

    padded = cv2.copyMakeBorder(crop, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(0, 0, 0))

    result_box = np.array([[float(tl_x), float(tl_y)], [float(br_x), float(br_y)]])
    result_bias = np.array([float(tl_x - left), float(tl_y - top)])
    return padded, result_box, result_bias


@dataclass
class HandPreprocessResult:
    blob: np.ndarray                 # 224x224x3 RGB float32 [0,1], HWC
    rotated_palm_bbox: np.ndarray    # (2,2)
    angle_deg: float
    rotation_matrix: np.ndarray      # 2x3
    pad_bias: np.ndarray             # (2,)


def preprocess_for_hand(bgrx: np.ndarray, palm: PalmDetection) -> HandPreprocessResult:
    palm_box = np.array([[palm.box[0], palm.box[1]],
                          [palm.box[0] + palm.box[2], palm.box[1] + palm.box[3]]])

    bgr = cv2.cvtColor(bgrx, cv2.COLOR_BGRA2BGR) if bgrx.shape[2] == 4 else bgrx

    first_img, first_box, first_bias = crop_and_pad_from_palm(
        bgr, palm_box, PALM_BOX_PRE_SHIFT_VECTOR, PALM_BOX_PRE_ENLARGE_FACTOR, True)
    image_rgb = cv2.cvtColor(first_img, cv2.COLOR_BGR2RGB)
    pad_bias = first_bias

    local_box = first_box - pad_bias
    local_landmarks = palm.landmarks - pad_bias  # (7,2)

    p1 = local_landmarks[PALM_LANDMARK_INDEX_OF_PALM_BASE]
    p2 = local_landmarks[PALM_LANDMARK_INDEX_OF_MIDDLE_FINGER_BASE]
    radians = PI / 2.0 - math.atan2(-(p2[1] - p1[1]), p2[0] - p1[0])
    radians = radians - 2.0 * PI * math.floor((radians + PI) / (2.0 * PI))
    angle_deg = radians * 180.0 / PI

    center = (local_box[0] + local_box[1]) * 0.5
    rotation_matrix = cv2.getRotationMatrix2D((float(center[0]), float(center[1])), angle_deg, 1.0)

    rotated_image = cv2.warpAffine(image_rgb, rotation_matrix, (image_rgb.shape[1], image_rgb.shape[0]))

    m00, m01, m02 = rotation_matrix[0]
    m10, m11, m12 = rotation_matrix[1]
    rotated_points = np.empty((7, 2))
    for i in range(7):
        x, y = local_landmarks[i]
        rotated_points[i] = (m00 * x + m01 * y + m02, m10 * x + m11 * y + m12)

    rotated_box_pre = np.array([rotated_points.min(axis=0), rotated_points.max(axis=0)])

    second_img, second_box, _second_bias = crop_and_pad_from_palm(
        rotated_image, rotated_box_pre, PALM_BOX_SHIFT_VECTOR, PALM_BOX_ENLARGE_FACTOR, False)

    resized = cv2.resize(second_img, (HAND_INPUT_SIZE, HAND_INPUT_SIZE), interpolation=cv2.INTER_AREA)
    blob = resized.astype(np.float32) / 255.0

    return HandPreprocessResult(blob=blob, rotated_palm_bbox=second_box, angle_deg=angle_deg,
                                 rotation_matrix=rotation_matrix, pad_bias=pad_bias)


# --- Stage 2: hand landmarks (224x224 NHWC input, direct regression) -------

NUM_HAND_KEYPOINTS = 21


@dataclass
class HandResult:
    box: np.ndarray            # (2,2) -- original image space
    landmarks: np.ndarray      # (21, 3) -- x,y in original image space; z relative to wrist
    handedness: float          # 0 = left, 1 = right
    confidence: float


def postprocess_hand(landmarks_raw: np.ndarray, conf: float, handedness: float,
                      pre: HandPreprocessResult, conf_threshold: float) -> HandResult | None:
    if conf < conf_threshold:
        return None

    landmarks = landmarks_raw.reshape(NUM_HAND_KEYPOINTS, 3).astype(np.float64).copy()

    wh_rotated = pre.rotated_palm_bbox[1] - pre.rotated_palm_bbox[0]
    scale_x = wh_rotated[0] / HAND_INPUT_SIZE
    scale_y = wh_rotated[1] / HAND_INPUT_SIZE
    max_scale = max(scale_x, scale_y)

    landmarks[:, 0] = (landmarks[:, 0] - HAND_INPUT_SIZE / 2.0) * max_scale
    landmarks[:, 1] = (landmarks[:, 1] - HAND_INPUT_SIZE / 2.0) * max_scale
    landmarks[:, 2] = landmarks[:, 2] * max_scale

    # Deliberately NOT the same row-dot-matrix convention as preprocess_for_hand's
    # point rotation above: the reference implementation applies
    # `landmarks @ coords_rotation_matrix[:, :2]` (row-vector times matrix
    # columns) here -- the transpose of the standard affine convention. See
    # ONNXHandTracking.cpp's postprocessHand() comment.
    coords_rot = cv2.getRotationMatrix2D((0.0, 0.0), pre.angle_deg, 1.0)
    c00, c01 = coords_rot[0, 0], coords_rot[0, 1]
    c10, c11 = coords_rot[1, 0], coords_rot[1, 1]

    rotated_landmarks = landmarks.copy()
    rotated_landmarks[:, 0] = landmarks[:, 0] * c00 + landmarks[:, 1] * c10
    rotated_landmarks[:, 1] = landmarks[:, 0] * c01 + landmarks[:, 1] * c11

    m = pre.rotation_matrix
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    r00, r01 = m00, m10
    r10, r11 = m01, m11
    inv_tx = -(r00 * m02 + r01 * m12)
    inv_ty = -(r10 * m02 + r11 * m12)

    bbox_center = (pre.rotated_palm_bbox[0] + pre.rotated_palm_bbox[1]) * 0.5
    original_center = (r00 * bbox_center[0] + r01 * bbox_center[1] + inv_tx,
                        r10 * bbox_center[0] + r11 * bbox_center[1] + inv_ty)

    landmarks[:, 0] = rotated_landmarks[:, 0] + original_center[0] + pre.pad_bias[0]
    landmarks[:, 1] = rotated_landmarks[:, 1] + original_center[1] + pre.pad_bias[1]
    landmarks[:, 2] = rotated_landmarks[:, 2]

    rmin = landmarks[:, :2].min(axis=0)
    rmax = landmarks[:, :2].max(axis=0)
    box = np.array([rmin, rmax])
    wh = box[1] - box[0]
    shift = HAND_BOX_SHIFT_VECTOR * wh
    box = box + shift
    center2 = (box[0] + box[1]) * 0.5
    wh = box[1] - box[0]
    half = wh * (HAND_BOX_ENLARGE_FACTOR * 0.5)
    box = np.array([center2 - half, center2 + half])

    return HandResult(box=box, landmarks=landmarks, handedness=handedness, confidence=conf)


def run_hand_landmark_stage(hand_session, hand_input_name, hand_output_names,
                             bgrx: np.ndarray, palm: PalmDetection, hand_conf_threshold: float) -> HandResult | None:
    pre = preprocess_for_hand(bgrx, palm)
    tensor = pre.blob[np.newaxis, ...]
    outputs = hand_session.run(hand_output_names, {hand_input_name: tensor})
    landmarks_raw = np.asarray(outputs[0]).reshape(-1)
    conf = float(np.asarray(outputs[1]).reshape(-1)[0])
    handedness = float(np.asarray(outputs[2]).reshape(-1)[0])
    return postprocess_hand(landmarks_raw, conf, handedness, pre, hand_conf_threshold)


# Standard MediaPipe 21-point hand skeleton connections.
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


@dataclass
class Track:
    box: tuple[int, int, int, int]  # x, y, w, h
    missed_frames: int = 0


def offset_palm_detection(palm: PalmDetection, offset: tuple[float, float]) -> PalmDetection:
    x, y, w, h = palm.box
    return PalmDetection(box=(x + offset[0], y + offset[1], w, h),
                          landmarks=palm.landmarks + np.asarray(offset), score=palm.score)


def iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0.0
    inter = (ix2 - ix1) * (iy2 - iy1)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def clamp_rect_to_frame(rect: tuple[int, int, int, int], frame_w: int, frame_h: int) -> tuple[int, int, int, int]:
    x, y, w, h = rect
    ix1, iy1 = max(x, 0), max(y, 0)
    ix2, iy2 = min(x + w, frame_w), min(y + h, frame_h)
    if ix2 <= ix1 or iy2 <= iy1:
        return (0, 0, 0, 0)
    return (ix1, iy1, ix2 - ix1, iy2 - iy1)


def enlarge_and_clamp(box: tuple[int, int, int, int], margin_factor: float, min_size: int,
                       frame_size: tuple[int, int]) -> tuple[int, int, int, int]:
    x, y, w, h = box
    cx, cy = x + w * 0.5, y + h * 0.5
    nw = max(w * margin_factor, float(min_size))
    nh = max(h * margin_factor, float(min_size))
    r = (int(cx - nw * 0.5), int(cy - nh * 0.5), int(nw), int(nh))
    return clamp_rect_to_frame(r, frame_size[0], frame_size[1])


def find_depth_guided_candidates(depth_mm: np.ndarray, existing_tracks: list[tuple[int, int, int, int]],
                                  min_depth_m: float, max_depth_m: float, focal_length_px: float,
                                  max_candidates: int) -> list[tuple[int, int, int, int]]:
    mask = cv2.inRange(depth_mm, min_depth_m * 1000.0, max_depth_m * 1000.0)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    k_invalid = float(np.finfo(np.float32).max)
    search = np.full(depth_mm.shape, k_invalid, dtype=np.float32)
    nonzero = mask > 0
    search[nonzero] = depth_mm[nonzero]

    frame_w, frame_h = depth_mm.shape[1], depth_mm.shape[0]
    for t in existing_tracks:
        ex_x, ex_y, ex_w, ex_h = enlarge_and_clamp(t, TRACKING_CROP_MARGIN_FACTOR, TRACKING_CROP_MIN_SIZE,
                                                    (frame_w, frame_h))
        if ex_w > 0 and ex_h > 0:
            search[ex_y:ex_y + ex_h, ex_x:ex_x + ex_w] = k_invalid

    candidates: list[tuple[int, int, int, int]] = []
    max_iterations = max_candidates + 2
    for _ in range(max_iterations):
        if len(candidates) >= max_candidates:
            break
        min_val, _max_val, min_loc, _max_loc = cv2.minMaxLoc(search)
        if min_val >= k_invalid:
            break

        depth_m = min_val / 1000.0
        crop_size_f = focal_length_px * HAND_FOOTPRINT_METERS / max(depth_m, 0.1)
        crop_size = int(_clamp(crop_size_f, DEPTH_CANDIDATE_MIN_CROP_SIZE, DEPTH_CANDIDATE_MAX_CROP_SIZE))

        px, py = min_loc
        peak_box = clamp_rect_to_frame((px - crop_size // 2, py - crop_size // 2, crop_size, crop_size),
                                        frame_w, frame_h)
        bx, by, bw, bh = peak_box
        if bw > 0 and bh > 0:
            search[by:by + bh, bx:bx + bw] = k_invalid

        flood_mask = np.zeros((frame_h + 2, frame_w + 2), dtype=np.uint8)
        cv2.floodFill(search, flood_mask, min_loc, k_invalid,
                       loDiff=FLOOD_DEPTH_TOLERANCE_MM, upDiff=FLOOD_DEPTH_TOLERANCE_MM,
                       flags=4 | cv2.FLOODFILL_FIXED_RANGE)

        candidates.append(peak_box)
    return candidates


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--palm-model", required=True)
    parser.add_argument("--hand-model", required=True)
    parser.add_argument("--palm-conf", type=float, default=0.5)
    parser.add_argument("--palm-iou", type=float, default=0.3)
    parser.add_argument("--hand-conf", type=float, default=0.8)
    parser.add_argument("--min-depth", type=float, default=0.4)
    parser.add_argument("--max-depth", type=float, default=3.5)
    parser.add_argument("--max-hands", type=int, default=2)
    parser.add_argument("--pythonpath", help="Directory containing pyfreenect2.pyd/.so")
    parser.add_argument("--duration", type=float, default=None,
                         help="Auto-close after this many seconds (default: run until Esc/window close)")
    args = parser.parse_args(argv)

    if args.pythonpath:
        sys.path.insert(0, args.pythonpath)
    import pyfreenect2 as f2

    try:
        palm_session = onnx_common.create_session(args.palm_model)
        hand_session = onnx_common.create_session(args.hand_model)
    except Exception as e:
        print(f"Failed to load ONNX model(s): {e}", file=sys.stderr)
        return 1

    palm_input_name = palm_session.get_inputs()[0].name
    palm_output_names = [o.name for o in palm_session.get_outputs()]
    hand_input_name = hand_session.get_inputs()[0].name
    hand_output_names = [o.name for o in hand_session.get_outputs()]

    fn = f2.Freenect2()
    if fn.enumerate_devices() == 0:
        print("No Kinect v2 device found.", file=sys.stderr)
        return 1

    serial = fn.get_default_device_serial_number()
    device = fn.open_device(serial)
    if device is None:
        print(f"Failed to open device with serial {serial}.", file=sys.stderr)
        return 1

    listener = f2.SyncMultiFrameListener(f2.FrameType.Color | f2.FrameType.Depth)
    device.set_color_frame_listener(listener)
    device.set_ir_and_depth_frame_listener(listener)

    if not device.start():
        print("Failed to start device.", file=sys.stderr)
        return 1

    print(f"Serial:   {device.get_serial_number()}")
    print(f"Firmware: {device.get_firmware_version()}")

    window_name = "ONNX Hand Tracking"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    # Depth-in-color-space (bigdepth) drives find_depth_guided_candidates()
    # below -- undistorted/registered are unused beyond being required
    # apply() out-params (same pattern as Protonect.cpp/protonect.py).
    registration = f2.Registration(device.get_ir_camera_params(), device.get_color_camera_params())
    undistorted = f2.Frame(512, 424, 4)
    registered = f2.Frame(512, 424, 4)
    bigdepth = f2.Frame(1920, 1082, 4)
    color_focal_length_px = device.get_color_camera_params().fx
    tracks: list[Track] = []

    start = time.monotonic()

    try:
        while cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) > 0:
            if args.duration is not None and (time.monotonic() - start) >= args.duration:
                break

            frames = listener.wait_for_new_frame()
            color = frames[f2.FrameType.Color]
            depth = frames[f2.FrameType.Depth]
            bgrx = np.asarray(color)

            registration.apply(color, depth, undistorted, registered, True, bigdepth)
            bigdepth_arr = np.asarray(bigdepth)  # (1082, 1920) float32, mm
            # bigdepth has one blank row top and bottom (1082 vs color's 1080 rows).
            depth_color_space = bigdepth_arr[1:1 + bgrx.shape[0], :]

            palms: list[PalmDetection] = []

            # Step 1: advance existing tracks.
            new_tracks: list[Track] = []
            for tr in tracks:
                crop_rect = enlarge_and_clamp(tr.box, TRACKING_CROP_MARGIN_FACTOR, TRACKING_CROP_MIN_SIZE,
                                               (bgrx.shape[1], bgrx.shape[0]))
                cx, cy, cw, ch = crop_rect
                found = detect_palms(palm_session, palm_input_name, palm_output_names,
                                      bgrx[cy:cy + ch, cx:cx + cw], args.palm_conf, args.palm_iou) if cw > 0 and ch > 0 else []

                best = max(found, key=lambda p: p.score) if found else None
                if best is not None:
                    palm = offset_palm_detection(best, (float(cx), float(cy)))
                    tr.box = (int(palm.box[0]), int(palm.box[1]), int(palm.box[2]), int(palm.box[3]))
                    tr.missed_frames = 0
                    palms.append(palm)
                    new_tracks.append(tr)
                else:
                    tr.missed_frames += 1
                    if tr.missed_frames <= MAX_MISSED_FRAMES:
                        new_tracks.append(tr)
            tracks = new_tracks

            # Step 2: depth-guided acquisition of new hands while under max_hands.
            if len(tracks) < args.max_hands:
                existing_boxes = [t.box for t in tracks]
                slots_left = args.max_hands - len(tracks)
                candidates = find_depth_guided_candidates(
                    depth_color_space, existing_boxes, args.min_depth, args.max_depth,
                    color_focal_length_px, slots_left + DEPTH_CANDIDATE_SEARCH_HEADROOM)

                for crop_rect in candidates:
                    if len(tracks) >= args.max_hands:
                        break
                    cx, cy, cw, ch = crop_rect
                    if cw <= 0 or ch <= 0:
                        continue
                    found = detect_palms(palm_session, palm_input_name, palm_output_names,
                                          bgrx[cy:cy + ch, cx:cx + cw], args.palm_conf, args.palm_iou)
                    for raw_palm in found:
                        palm = offset_palm_detection(raw_palm, (float(cx), float(cy)))
                        palm_box_i = (int(palm.box[0]), int(palm.box[1]), int(palm.box[2]), int(palm.box[3]))
                        if any(iou(t.box, palm_box_i) > TRACK_OVERLAP_IOU_THRESHOLD for t in tracks):
                            continue
                        if run_hand_landmark_stage(hand_session, hand_input_name, hand_output_names,
                                                    bgrx, palm, args.hand_conf) is None:
                            continue
                        tracks.append(Track(box=palm_box_i, missed_frames=0))
                        palms.append(palm)
                        if len(tracks) >= args.max_hands:
                            break

            display = cv2.cvtColor(bgrx, cv2.COLOR_BGRA2BGR)
            h_disp, w_disp = display.shape[:2]

            for palm in palms:
                pre = preprocess_for_hand(bgrx, palm)
                tensor = pre.blob[np.newaxis, ...]
                outputs = hand_session.run(hand_output_names, {hand_input_name: tensor})
                landmarks_raw = np.asarray(outputs[0]).reshape(-1)
                conf = float(np.asarray(outputs[1]).reshape(-1)[0])
                handedness = float(np.asarray(outputs[2]).reshape(-1)[0])
                hand = postprocess_hand(landmarks_raw, conf, handedness, pre, args.hand_conf)
                if hand is None:
                    continue

                x1, y1 = int(hand.box[0, 0]), int(hand.box[0, 1])
                x2, y2 = int(hand.box[1, 0]), int(hand.box[1, 1])
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w_disp, x2), min(h_disp, y2)
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 200, 255), 2)

                label = f"{'Left' if hand.handedness < 0.5 else 'Right'} {hand.confidence:.2f}"
                cv2.putText(display, label, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

                pts = hand.landmarks
                for a, b in HAND_CONNECTIONS:
                    p1 = (int(pts[a, 0]), int(pts[a, 1]))
                    p2 = (int(pts[b, 0]), int(pts[b, 1]))
                    cv2.line(display, p1, p2, (0, 255, 0), 2)
                for k in range(NUM_HAND_KEYPOINTS):
                    cv2.circle(display, (int(pts[k, 0]), int(pts[k, 1])), 3, (0, 0, 255), cv2.FILLED)

            cv2.imshow(window_name, display)
            listener.release(frames)

            if cv2.waitKey(1) == 27:  # Esc
                break
    finally:
        device.stop()
        device.close()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
