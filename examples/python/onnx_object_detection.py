#!/usr/bin/env python3
"""pyfreenect2 example -- a Python port of examples/ONNXObjectDetection.cpp.

YOLOv8 object-detection: runs a YOLOv8-family ONNX model (CPU execution
provider) on the Kinect's color stream and draws detected boxes + COCO class
labels in an OpenCV window. The per-anchor decode loop is vectorized with
numpy rather than a Python for loop (yolov8n has 8400 anchors) -- same
decode formula as onnx_common.cpp's loop, not a behavioral shortcut.

Usage:
    python onnx_object_detection.py --model <path-to-yolov8n.onnx> [--conf <0..1>] [--iou <0..1>]
        [--pythonpath <dir>] [--duration <seconds>]
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np

import onnx_common


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True, help="Path to a yolov8n.onnx-family model")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--pythonpath", help="Directory containing pyfreenect2.pyd/.so")
    parser.add_argument("--duration", type=float, default=None,
                         help="Auto-close after this many seconds (default: run until Esc/window close)")
    args = parser.parse_args(argv)

    if args.pythonpath:
        sys.path.insert(0, args.pythonpath)
    import pyfreenect2 as f2

    try:
        session = onnx_common.create_session(args.model)
    except Exception as e:  # onnxruntime raises plain Exception subclasses, not a dedicated base
        print(f"Failed to load ONNX model '{args.model}': {e}", file=sys.stderr)
        return 1

    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    input_shape = session.get_inputs()[0].shape
    model_size = 640
    if len(input_shape) == 4 and isinstance(input_shape[2], int) and input_shape[2] > 0:
        model_size = input_shape[2]
    print(f"Model input size: {model_size}x{model_size}")

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

    window_name = "ONNX Object Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    start = time.monotonic()

    try:
        while cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) > 0:
            if args.duration is not None and (time.monotonic() - start) >= args.duration:
                break

            frames = listener.wait_for_new_frame()
            color = frames[f2.FrameType.Color]
            bgrx = np.asarray(color)  # (height, width, 4)

            letterboxed, lb = onnx_common.letterbox(bgrx, model_size)
            tensor = onnx_common.to_chw_tensor(letterboxed)

            output = session.run([output_name], {input_name: tensor})[0]
            # [1, 84, num_anchors]: 4 box coords + 80 class scores, anchor-free (no objectness).
            output = output[0]
            num_channels, num_anchors = output.shape
            num_classes = num_channels - 4

            cx, cy, w, h = output[0], output[1], output[2], output[3]
            class_scores = output[4:4 + num_classes]  # (num_classes, num_anchors)
            best_class = np.argmax(class_scores, axis=0)
            best_score = np.max(class_scores, axis=0)

            mask = best_score >= args.conf
            boxes = []
            scores = []
            class_ids = []
            for cxi, cyi, wi, hi, cls, score in zip(cx[mask], cy[mask], w[mask], h[mask],
                                                       best_class[mask], best_score[mask]):
                boxes.append(onnx_common.map_box_to_original(float(cxi), float(cyi), float(wi), float(hi), lb))
                scores.append(float(score))
                class_ids.append(int(cls))

            keep = onnx_common.nms(boxes, scores, args.iou)

            display = cv2.cvtColor(bgrx, cv2.COLOR_BGRA2BGR)
            h_disp, w_disp = display.shape[:2]

            for idx in keep:
                x1, y1, x2, y2 = boxes[idx]
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w_disp, x2), min(h_disp, y2)
                cid = class_ids[idx]
                label = onnx_common.COCO_CLASS_NAMES[cid] if 0 <= cid < len(onnx_common.COCO_CLASS_NAMES) else "?"
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                text = f"{label} {scores[idx]:.2f}"
                (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(display, (x1, y1 - th - 4), (x1 + tw + 4, y1), (0, 255, 0), cv2.FILLED)
                cv2.putText(display, text, (x1 + 2, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

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
