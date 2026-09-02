#!/usr/bin/env python3
"""pyfreenect2 example -- a Python port of examples/ONNXPoseEstimation.cpp.

YOLOv8-pose keypoint estimation: runs a YOLOv8-pose ONNX model (CPU
execution provider) on the Kinect's color stream and draws detected person
boxes + 17-point COCO skeletons in an OpenCV window.

Usage:
    python onnx_pose_estimation.py --model <path-to-yolov8n-pose.onnx> [--conf <0..1>] [--iou <0..1>]
        [--pythonpath <dir>] [--duration <seconds>]
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np

import onnx_common

NUM_KEYPOINTS = 17
KEYPOINT_VISIBLE_THRESHOLD = 0.5


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True, help="Path to a yolov8n-pose.onnx-family model")
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
    except Exception as e:
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

    window_name = "ONNX Pose Estimation"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    start = time.monotonic()

    try:
        while cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) > 0:
            if args.duration is not None and (time.monotonic() - start) >= args.duration:
                break

            frames = listener.wait_for_new_frame()
            color = frames[f2.FrameType.Color]
            bgrx = np.asarray(color)

            letterboxed, lb = onnx_common.letterbox(bgrx, model_size)
            tensor = onnx_common.to_chw_tensor(letterboxed)

            output = session.run([output_name], {input_name: tensor})[0][0]
            # [56, num_anchors]: 4 box + 1 person-confidence + 17 keypoints x (x, y, visibility).
            cx, cy, w, h, conf = output[0], output[1], output[2], output[3], output[4]
            mask = conf >= args.conf

            boxes = []
            scores = []
            all_keypoints: list[list[tuple[float, float, float]]] = []

            kp_block = output[5:5 + 3 * NUM_KEYPOINTS]  # (51, num_anchors)
            for cxi, cyi, wi, hi, ci, kp_col in zip(cx[mask], cy[mask], w[mask], h[mask], conf[mask],
                                                      kp_block[:, mask].T):
                boxes.append(onnx_common.map_box_to_original(float(cxi), float(cyi), float(wi), float(hi), lb))
                scores.append(float(ci))
                kpts = []
                for k in range(NUM_KEYPOINTS):
                    kx, ky, kvis = kp_col[3 * k], kp_col[3 * k + 1], kp_col[3 * k + 2]
                    mx, my = onnx_common.map_point_to_original(float(kx), float(ky), lb)
                    kpts.append((mx, my, float(kvis)))
                all_keypoints.append(kpts)

            keep = onnx_common.nms(boxes, scores, args.iou)

            display = cv2.cvtColor(bgrx, cv2.COLOR_BGRA2BGR)
            h_disp, w_disp = display.shape[:2]

            for idx in keep:
                x1, y1, x2, y2 = boxes[idx]
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w_disp, x2), min(h_disp, y2)
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 200, 255), 2)

                kpts = all_keypoints[idx]
                for a, b in onnx_common.COCO_SKELETON_EDGES:
                    if kpts[a][2] >= KEYPOINT_VISIBLE_THRESHOLD and kpts[b][2] >= KEYPOINT_VISIBLE_THRESHOLD:
                        p1 = (int(kpts[a][0]), int(kpts[a][1]))
                        p2 = (int(kpts[b][0]), int(kpts[b][1]))
                        cv2.line(display, p1, p2, (0, 255, 0), 2)
                for kx, ky, kvis in kpts:
                    if kvis >= KEYPOINT_VISIBLE_THRESHOLD:
                        cv2.circle(display, (int(kx), int(ky)), 3, (0, 0, 255), cv2.FILLED)

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
