#!/usr/bin/env python3
"""pyfreenect2 example -- a Python port of examples/PCLPointCloudAudio.cpp.

PCL has no Python binding, so this renders a top-down (bird's-eye/"radar")
projection of the point cloud with tkinter instead of pcl::visualization --
stdlib-only, no extra installs needed. A top-down view is actually a natural
fit here: DOA azimuth (the whole point of this example) reads directly as an
angle on a radar-style display.

Ported 1:1 from PCLPointCloudAudio.cpp:
  - same highlight tolerance/confidence-threshold constants
  - same array-to-camera offset caveat (see ARRAY_TO_CAMERA_OFFSET_X/Z below)
  - same red->yellow->green confidence color scale
  - same "highlight tints the point's own color, GCC-PHAT and beamforming
    shown as two separate arrows" design

One deliberate behavioral difference, forced by pyfreenect2's v1 scope (see
README.md#python-bindings): the C++ version feeds every audio buffer into
DoaEstimator via a push callback (AudioFrameListener), so it never drops a
buffer. pyfreenect2 only exposes the polling SyncAudioListener (latest-frame-
wins), so this script instead runs a tight background polling thread calling
add_samples() on whatever's newest -- DoaEstimator.addSamples()/estimate*()
are documented as safe to call from different threads for exactly this
producer/consumer split, so this is a legitimate use of the binding, but it
can still drop an audio buffer if the poll loses a race with the next one
arriving (unlike the C++ version, which structurally cannot).

Usage:
    python pointcloud_audio.py [--pythonpath <dir>] [--stride N]

Requires: numpy (`pip install numpy`); pyfreenect2 itself has no such
dependency. tkinter ships with standard Python on Windows/macOS; on Linux
it's usually a separate distro package (e.g. `python3-tk`).
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import time
import tkinter as tk

# Ported constants -- see PCLPointCloudAudio.cpp for the full derivation/
# citation of each.
HIGHLIGHT_TOLERANCE_DEG = 10.0
HIGHLIGHT_CONFIDENCE_THRESHOLD = 0.15
# meters, array center's (x, z) in the depth/color camera's own frame --
# best-effort estimate (Yamasaki et al., CONVR2016), not independently
# re-verified against a physical unit the way the mic-spacing numbers in
# audio_doa.hpp were. If highlighting looks worse than no correction at all,
# try flipping these signs first.
ARRAY_TO_CAMERA_OFFSET_X = -0.095
ARRAY_TO_CAMERA_OFFSET_Z = -0.013

DEPTH_WIDTH, DEPTH_HEIGHT = 512, 424


def confidence_to_color(confidence: float) -> str:
    """Red (unreliable) -> yellow -> green (reliable). Hue, not brightness,
    carries the signal -- see PCLPointCloudAudio.cpp's confidenceToColor()
    for why."""
    c = max(0.0, min(1.0, confidence))
    if c < 0.5:
        t = c / 0.5
        r, g, b = 1.0, t, 0.0
    else:
        t = (c - 0.5) / 0.5
        r, g, b = 1.0 - t, 1.0, 0.0
    return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


def blend_toward(channel: int, target_255: float, strength: float) -> int:
    return int(min(255.0, channel + strength * (target_255 - channel)))


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
        self.gcc_arrow = self.canvas.create_line(0, 0, 0, 0, fill="", width=2, state="hidden")
        self.beam_arrow = self.canvas.create_line(0, 0, 0, 0, fill="", width=3, state="hidden")
        self.status = self.canvas.create_text(8, 8, anchor="nw", fill="white", font=("Consolas", 10))

    def project(self, x: float, z: float) -> tuple[float, float]:
        return self.origin_x + x * self.scale, self.origin_y - z * self.scale

    def draw_arrow(self, item, estimate, length_px: float):
        if estimate is None or not estimate.valid or estimate.confidence < HIGHLIGHT_CONFIDENCE_THRESHOLD:
            self.canvas.itemconfigure(item, state="hidden")
            return
        rad = math.radians(estimate.azimuth_degrees)
        ex = self.origin_x + length_px * math.sin(rad)
        ey = self.origin_y - length_px * math.cos(rad)
        self.canvas.coords(item, self.origin_x, self.origin_y, ex, ey)
        self.canvas.itemconfigure(item, fill=confidence_to_color(estimate.confidence), state="normal")

    def render(self, points, gcc, beam, status_text: str):
        highlight = beam is not None and beam.valid and beam.confidence >= HIGHLIGHT_CONFIDENCE_THRESHOLD
        highlight_rad = math.radians(beam.azimuth_degrees) if highlight else 0.0
        highlight_color = confidence_to_color(beam.confidence) if highlight else None
        tol_rad = math.radians(HIGHLIGHT_TOLERANCE_DEG)

        i = 0
        for x, y, z, r, g, b in points:
            if i >= len(self.pool):
                break
            item = self.pool[i]
            i += 1

            if highlight:
                azimuth = math.atan2(x - ARRAY_TO_CAMERA_OFFSET_X, z - ARRAY_TO_CAMERA_OFFSET_Z)
                diff = abs(azimuth - highlight_rad)
                if diff < tol_rad:
                    strength = 1.0 - diff / tol_rad
                    hr = int(highlight_color[1:3], 16)
                    hg = int(highlight_color[3:5], 16)
                    hb = int(highlight_color[5:7], 16)
                    r = blend_toward(r, hr, strength)
                    g = blend_toward(g, hg, strength)
                    b = blend_toward(b, hb, strength)

            px, py = self.project(x, z)
            self.canvas.coords(item, px - 1.5, py - 1.5, px + 1.5, py + 1.5)
            self.canvas.itemconfigure(item, fill=f"#{r:02x}{g:02x}{b:02x}", state="normal")

        for j in range(i, len(self.pool)):
            self.canvas.itemconfigure(self.pool[j], state="hidden")

        self.draw_arrow(self.gcc_arrow, gcc, length_px=self.scale * 1.0)
        self.draw_arrow(self.beam_arrow, beam, length_px=self.scale * 1.0)
        self.canvas.itemconfigure(self.status, text=status_text)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pythonpath", help="Directory containing pyfreenect2.pyd/.so")
    parser.add_argument("--stride", type=int, default=8, help="Downsample stride over the 512x424 depth grid (default: 8)")
    parser.add_argument("--scale", type=float, default=110.0, help="Pixels per meter in the top-down view (default: 110)")
    parser.add_argument("--width", type=int, default=760)
    parser.add_argument("--height", type=int, default=760)
    parser.add_argument("--duration", type=float, default=None,
                         help="Auto-close after this many seconds (default: run until the window is closed). "
                              "Goes through the normal clean-shutdown path, same as closing the window by hand.")
    args = parser.parse_args(argv)

    if args.pythonpath:
        sys.path.insert(0, args.pythonpath)
    import pyfreenect2 as f2

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

    # Reuses the video device's already-open USB session, same as
    # PCLPointCloudAudio.cpp / AudioVideoSync.cpp. Video must already be
    # started before this.
    doa = f2.DoaEstimator()
    audio_listener = f2.SyncAudioListener()
    audio = f2.AudioCapture(device)
    audio_running = audio.is_available() and audio.start(audio_listener)
    if not audio_running:
        print("No Kinect v2 microphone array found -- continuing video-only, no DOA.", file=sys.stderr)

    registration = f2.Registration(device.get_ir_camera_params(), device.get_color_camera_params())
    undistorted = f2.Frame(DEPTH_WIDTH, DEPTH_HEIGHT, 4)
    registered = f2.Frame(DEPTH_WIDTH, DEPTH_HEIGHT, 4)

    stop_event = threading.Event()
    state_lock = threading.Lock()
    latest = {"points": [], "gcc": None, "beam": None, "status": "starting..."}

    def audio_poll_loop():
        # See module docstring: this is the "several tight polls per audio
        # buffer" substitute for the C++ version's lossless push callback.
        while not stop_event.is_set():
            frame = audio_listener.get_latest_frame()
            if frame is not None:
                doa.add_samples(frame)
            else:
                time.sleep(0.001)

    def video_loop():
        while not stop_event.is_set():
            frames = listener.wait_for_new_frame(1000)
            if frames is None:
                continue

            rgb = frames[f2.FrameType.Color]
            depth = frames[f2.FrameType.Depth]
            registration.apply(rgb, depth, undistorted, registered)

            gcc = doa.estimate_gcc_phat() if audio_running else None
            beam = doa.estimate_beamforming() if audio_running else None

            points = []
            for r in range(0, DEPTH_HEIGHT, args.stride):
                for c in range(0, DEPTH_WIDTH, args.stride):
                    x, y, z, rgb_packed = registration.get_point_xyz_rgb(undistorted, registered, r, c)
                    if not math.isfinite(z):
                        continue
                    bgrx = struct.pack("<f", rgb_packed)
                    points.append((x, y, z, bgrx[2], bgrx[1], bgrx[0]))

            status = f"points: {len(points)}"
            if gcc is not None and beam is not None and gcc.valid and beam.valid:
                status += f"\nDOA gcc={gcc.azimuth_degrees:5.1f} deg (conf={gcc.confidence:.2f})"
                status += f"\nDOA beam={beam.azimuth_degrees:5.1f} deg (conf={beam.confidence:.2f})"
                print(f"DOA gcc={gcc.azimuth_degrees:.1f}deg (conf={gcc.confidence:.2f})  "
                      f"beam={beam.azimuth_degrees:.1f}deg (conf={beam.confidence:.2f})")
            elif not audio_running:
                status += "\n(no microphone array -- video only)"

            with state_lock:
                latest["points"] = points
                latest["gcc"] = gcc
                latest["beam"] = beam
                latest["status"] = status

            listener.release(frames)

    audio_thread = threading.Thread(target=audio_poll_loop, daemon=True)
    video_thread = threading.Thread(target=video_loop, daemon=True)

    root = tk.Tk()
    root.title("Kinect v2 Point Cloud + DOA (pyfreenect2)")
    viewer = Viewer(root, args.width, args.height, args.scale, args.stride)

    def redraw():
        with state_lock:
            points, gcc, beam, status = latest["points"], latest["gcc"], latest["beam"], latest["status"]
        viewer.render(points, gcc, beam, status)
        if not stop_event.is_set():
            root.after(33, redraw)

    def on_close():
        stop_event.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        audio_thread.start()
        video_thread.start()
        root.after(0, redraw)
        if args.duration is not None:
            root.after(int(args.duration * 1000), on_close)
        root.mainloop()
    finally:
        stop_event.set()
        video_thread.join(timeout=2.0)
        audio_thread.join(timeout=2.0)
        if audio_running:
            audio.stop()
        device.stop()
        device.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
