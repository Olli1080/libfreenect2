#!/usr/bin/env python3
"""pyfreenect2 example -- a Python port of examples/PCLPointCloud.cpp.

PCL has no Python binding, so this renders a top-down (bird's-eye) point
cloud projection with tkinter instead of pcl::visualization -- stdlib-only,
no extra installs needed. See pointcloud_audio.py for the same idea plus
live direction-of-arrival highlighting.

Usage:
    python pointcloud.py [--pythonpath <dir>] [cl | gl | vk | cpu] [--stride N]
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import tkinter as tk

DEPTH_WIDTH, DEPTH_HEIGHT = 512, 424


class Viewer:
    def __init__(self, root: tk.Tk, width: int, height: int, scale: float, stride: int):
        self.width, self.height, self.scale, self.stride = width, height, scale, stride
        self.origin_x, self.origin_y = width // 2, height - 30

        self.canvas = tk.Canvas(root, width=width, height=height, bg="black", highlightthickness=0)
        self.canvas.pack()

        rows = (DEPTH_HEIGHT + stride - 1) // stride
        cols = (DEPTH_WIDTH + stride - 1) // stride
        self.pool = [
            self.canvas.create_oval(0, 0, 2, 2, fill="#000000", outline="", state="hidden")
            for _ in range(rows * cols)
        ]
        self.canvas.create_oval(
            self.origin_x - 4, self.origin_y - 4, self.origin_x + 4, self.origin_y + 4,
            fill="#4080ff", outline="",
        )

    def project(self, x: float, z: float) -> tuple[float, float]:
        return self.origin_x + x * self.scale, self.origin_y - z * self.scale

    def render(self, points):
        i = 0
        for x, y, z, r, g, b in points:
            if i >= len(self.pool):
                break
            item = self.pool[i]
            i += 1
            px, py = self.project(x, z)
            self.canvas.coords(item, px - 1.5, py - 1.5, px + 1.5, py + 1.5)
            self.canvas.itemconfigure(item, fill=f"#{r:02x}{g:02x}{b:02x}", state="normal")
        for j in range(i, len(self.pool)):
            self.canvas.itemconfigure(self.pool[j], state="hidden")


def make_pipeline(f2, name):
    if name is None:
        return None
    mapping = {
        "cl": "OpenCLPacketPipeline",
        "gl": "OpenGLPacketPipeline",
        "vk": "VulkanPacketPipeline",
        "vulkan": "VulkanPacketPipeline",
        "cpu": "CpuPacketPipeline",
    }
    cls_name = mapping.get(name)
    if cls_name is None:
        print(f"Unknown pipeline argument: {name} (expected cl | gl | vk | cpu)", file=sys.stderr)
        return None
    cls = getattr(f2, cls_name, None)
    if cls is None:
        print(f"{cls_name} is not supported in this build!", file=sys.stderr)
        return None
    return cls()


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pipeline", nargs="?", choices=["cl", "gl", "vk", "vulkan", "cpu"], default=None)
    parser.add_argument("--pythonpath", help="Directory containing pyfreenect2.pyd/.so")
    parser.add_argument("--stride", type=int, default=6, help="Downsample stride over the 512x424 depth grid (default: 6)")
    parser.add_argument("--scale", type=float, default=110.0, help="Pixels per meter in the top-down view (default: 110)")
    parser.add_argument("--width", type=int, default=760)
    parser.add_argument("--height", type=int, default=760)
    parser.add_argument("--duration", type=float, default=None,
                         help="Auto-close after this many seconds (default: run until the window is closed)")
    args = parser.parse_args(argv)

    if args.pythonpath:
        sys.path.insert(0, args.pythonpath)
    import pyfreenect2 as f2

    fn = f2.Freenect2()
    if fn.enumerate_devices() == 0:
        print("No Kinect v2 device found.", file=sys.stderr)
        return 1

    serial = fn.get_default_device_serial_number()
    pipeline = make_pipeline(f2, args.pipeline)
    device = fn.open_device(serial, pipeline) if pipeline is not None else fn.open_device(serial)
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

    registration = f2.Registration(device.get_ir_camera_params(), device.get_color_camera_params())
    undistorted = f2.Frame(DEPTH_WIDTH, DEPTH_HEIGHT, 4)
    registered = f2.Frame(DEPTH_WIDTH, DEPTH_HEIGHT, 4)

    stop_event = threading.Event()
    state_lock = threading.Lock()
    latest_points: list = []

    def video_loop():
        nonlocal latest_points
        while not stop_event.is_set():
            frames = listener.wait_for_new_frame(1000)
            if frames is None:
                continue

            rgb = frames[f2.FrameType.Color]
            depth = frames[f2.FrameType.Depth]
            registration.apply(rgb, depth, undistorted, registered)

            points = []
            for r in range(0, DEPTH_HEIGHT, args.stride):
                for c in range(0, DEPTH_WIDTH, args.stride):
                    x, y, z, rgb_packed = registration.get_point_xyz_rgb(undistorted, registered, r, c)
                    if not math.isfinite(z):
                        continue
                    bgrx = struct.pack("<f", rgb_packed)
                    points.append((x, y, z, bgrx[2], bgrx[1], bgrx[0]))

            with state_lock:
                latest_points = points

            listener.release(frames)

    video_thread = threading.Thread(target=video_loop, daemon=True)

    root = tk.Tk()
    root.title("Kinect v2 Point Cloud (pyfreenect2)")
    viewer = Viewer(root, args.width, args.height, args.scale, args.stride)

    def redraw():
        with state_lock:
            points = latest_points
        viewer.render(points)
        if not stop_event.is_set():
            root.after(33, redraw)

    def on_close():
        stop_event.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        video_thread.start()
        root.after(0, redraw)
        if args.duration is not None:
            root.after(int(args.duration * 1000), on_close)
        root.mainloop()
    finally:
        stop_event.set()
        video_thread.join(timeout=2.0)
        device.stop()
        device.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
