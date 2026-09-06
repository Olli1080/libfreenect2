#!/usr/bin/env python3
"""pyfreenect2 example -- open every connected Kinect v2 at once.

The Python counterpart of examples/MultiKinect.cpp, and the multi-device
demonstration that neither the C++ nor the Python example set had before.
libfreenect2's core already supports any number of concurrently open devices
(each gets its own USB transfer pool, packet-processing threads and
pipeline); this script enumerates the bus, opens each device with its own
fresh pipeline, runs one capture thread per device, prints a live per-serial
FPS table, and -- with --viewer -- shows one OpenCV window with an
RGB + colourised-depth panel per device, side by side.

What matters when driving more than one Kinect from one process:

  - Each open_device() needs its OWN freshly built PacketPipeline. pyfreenect2
    disowns the Python pipeline object on the call, matching libfreenect2's
    "do not reuse or free it" contract -- so make_pipeline() is called once
    per device below.
  - One SyncMultiFrameListener PER device (its frame dict is keyed by
    FrameType, so a shared one would let two devices' Color frames collide).
    Its frame pool is small: each capture thread must release() a frame dict
    promptly, and from the SAME thread it waited on -- so the viewer copies
    what it needs on the capture thread and never parks a listener frame.
  - wait_for_new_frame() releases the GIL, so the per-device capture threads
    genuinely run in parallel.
  - GPU pipelines (cl/cuda/vk) take a device id -- pass -gpu=<id> to place
    Kinects on different GPUs (one id for all here; edit the call site for
    per-device placement).
  - USB 3 bandwidth is the ceiling: 3+ Kinects streaming at once want
    multiple separate PCIe USB3 controllers (README, "Requirements for
    multiple Kinects"). On one shared controller the per-device rate in the
    table sags once the bus saturates.

Run with -help for the full option list (parsed with argparse). Summary:
    [gl | cl | clkde | cuda | cudakde | vk | cpu] [<serial> ...]
    [-gpu <id>] [-frames <N>] [-capacity <N>] [-norgb | -nodepth] [-viewer]
    [--pythonpath <dir>]

    No pipeline arg      -> libfreenect2's compile-time default per device.
    Trailing digit args  -> restrict to those serials (default: all found).
    -frames <N>          -> stop once every device has delivered N frames.
    -capacity <N>        -> per-device listener buffer depth (default 1; >= 2
                           copies frames so a bursty consumer drops fewer).
    -viewer              -> one window, RGB + depth per device (needs numpy +
                           opencv-python); q or Esc closes it. Headless FPS
                           table is the default.

Requires: numpy + opencv-python only when -viewer is given; pyfreenect2
itself has no such dependency (see README.md#python-bindings).
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

# Sibling module (examples/python/protonect.py) -- reuse its pipeline factory,
# depth colouriser and pipeline-token set instead of duplicating them.
from protonect import PIPELINE_TOKENS, depth_to_bgr, make_pipeline

PANEL_W = 512          # each device's column width, px
RGB_H = 288            # 16:9 RGB strip height (depth strip is the native 424)


def parse_args(argv: list[str]):
    p = argparse.ArgumentParser(
        prog="multikinect.py", add_help=False,
        description="pyfreenect2 port of examples/MultiKinect.cpp -- opens every "
                    "connected Kinect v2 at once.")
    p.add_argument("-help", "-h", "--help", action="help", help="show this help and exit")
    p.add_argument("-gpu", type=int, default=-1, metavar="<id>",
                   help="GPU device id for the cl/cuda/vk pipelines")
    p.add_argument("-frames", type=int, default=None, metavar="<N>",
                   help="stop once every device has delivered N frames")
    p.add_argument("-capacity", type=int, default=1, metavar="<N>",
                   help="per-device listener buffer depth (default 1)")
    p.add_argument("-norgb", "--norgb", action="store_true", help="disable the color stream")
    p.add_argument("-nodepth", "--nodepth", action="store_true", help="disable the depth stream")
    p.add_argument("-viewer", "--viewer", action="store_true",
                   help="one OpenCV window, RGB + depth per device")
    p.add_argument("--pythonpath", metavar="<dir>",
                   help="directory containing the pyfreenect2 extension module")
    p.add_argument("tokens", nargs="*", metavar="[pipeline] [serial ...]",
                   help="a pipeline token and/or the device serials to open (default: all)")
    args = p.parse_args(argv)

    pipeline_name = None
    serials: list[str] = []
    for tok in args.tokens:
        if tok.isdigit():
            serials.append(tok)
        elif tok in PIPELINE_TOKENS:
            pipeline_name = tok
        else:
            p.error(f"unrecognized token: {tok!r}")

    if args.norgb and args.nodepth:
        p.error("disabling both streams is not allowed")

    return argparse.Namespace(
        pipeline_name=pipeline_name,
        device_id=args.gpu,
        framemax=args.frames,
        capacity=max(1, args.capacity),
        enable_rgb=not args.norgb,
        enable_depth=not args.nodepth,
        viewer_enabled=args.viewer,
        pythonpath=args.pythonpath,
        serials=serials,
    )


class DeviceRunner:
    """One open Kinect plus its capture thread."""

    def __init__(self, f2, dev, serial, frame_types, viewer_enabled, capacity=1, np=None, cv2=None):
        self.f2 = f2
        self.dev = dev
        self.serial = serial
        mode = (f2.FrameQueueMode.DropOldest if capacity > 1
                else f2.FrameQueueMode.DropNew)
        self.listener = f2.SyncMultiFrameListener(frame_types, capacity, mode)
        dev.set_color_frame_listener(self.listener)
        dev.set_ir_and_depth_frame_listener(self.listener)

        self.viewer_enabled = viewer_enabled
        self._np = np
        self._cv2 = cv2
        self.frames = 0            # single writer (capture thread), monotonic
        self.last_report_frames = 0
        self.fps_display = 0.0     # main-thread only, refreshed once a second
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._pending = None       # (rgb_bgr, depth_bgr) ndarrays for the main thread
        self.displayed = None      # main-thread-only: last (rgb, depth) shown
        self._thread = threading.Thread(target=self._loop, name=f"cap-{serial}", daemon=True)

    def start_thread(self):
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            fm = self.listener.wait_for_new_frame(1000)
            if fm is None:
                continue  # timeout -- re-check the stop flag
            self.frames += 1

            if self.viewer_enabled:
                # Downscale/colourise here so the listener frame can be
                # released immediately; the main thread only ever sees plain
                # ndarrays with no tie to the listener pool.
                rgb_bgr = depth_bgr = None
                color = fm.get(self.f2.FrameType.Color)
                if color is not None:
                    a = self._np.asarray(color)          # H x W x 4, BGRX
                    rgb_bgr = self._cv2.resize(a[:, :, :3], (PANEL_W, RGB_H))
                depth = fm.get(self.f2.FrameType.Depth)
                if depth is not None:
                    depth_bgr = depth_to_bgr(self._np, self._cv2, self._np.asarray(depth))
                with self._lock:
                    self._pending = (rgb_bgr, depth_bgr)

            self.listener.release(fm)

    def take_pending(self):
        with self._lock:
            p, self._pending = self._pending, None
            return p

    def stop_and_join(self):
        self._stop.set()
        self._thread.join()


def build_panel(np, cv2, r):
    """Stack a device's RGB strip over its depth strip, with a labelled header."""
    rgb_bgr, depth_bgr = r.displayed
    parts = []
    if rgb_bgr is not None:
        parts.append(rgb_bgr)
    if depth_bgr is not None:
        parts.append(depth_bgr if depth_bgr.shape[1] == PANEL_W
                     else cv2.resize(depth_bgr, (PANEL_W, depth_bgr.shape[0])))
    body = cv2.vconcat(parts) if len(parts) > 1 else parts[0]

    header = np.zeros((26, body.shape[1], 3), np.uint8)
    cv2.putText(header, f"{r.serial}   {r.fps_display:4.1f} fps", (8, 19),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1, cv2.LINE_AA)
    return cv2.vconcat([header, body])


def render_window(np, cv2, runners):
    panels = []
    for r in runners:
        fresh = r.take_pending()
        if fresh is not None:
            r.displayed = fresh
        if r.displayed is not None:
            panels.append(build_panel(np, cv2, r))
    if panels:
        h = max(p.shape[0] for p in panels)
        panels = [p if p.shape[0] == h else
                  cv2.copyMakeBorder(p, 0, h - p.shape[0], 0, 0, cv2.BORDER_CONSTANT)
                  for p in panels]
        cv2.imshow("MultiKinect", cv2.hconcat(panels))
    return (cv2.waitKey(1) & 0xFF) in (ord("q"), 27)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if args.pythonpath:
        sys.path.insert(0, args.pythonpath)

    import pyfreenect2 as f2

    cv2 = np = None
    if args.viewer_enabled:
        import cv2
        import numpy as np

    fn = f2.Freenect2()
    n = fn.enumerate_devices()
    if n == 0:
        print("no devices connected!")
        return -1

    serials = args.serials or [fn.get_device_serial_number(i) for i in range(n)]

    frame_types = 0
    if args.enable_rgb:
        frame_types |= f2.FrameType.Color
    if args.enable_depth:
        frame_types |= f2.FrameType.Ir | f2.FrameType.Depth

    runners: list[DeviceRunner] = []
    for serial in serials:
        pipeline = make_pipeline(f2, args.pipeline_name, args.device_id)
        dev = fn.open_device(serial, pipeline) if pipeline is not None else fn.open_device(serial)
        if dev is None:
            print(f"  [{serial}] failed to open -- skipping")
            continue
        runners.append(DeviceRunner(f2, dev, serial, frame_types, args.viewer_enabled,
                                    args.capacity, np, cv2))

    if not runners:
        print("no devices opened!")
        return -1

    for r in runners:
        ok = (r.dev.start() if args.enable_rgb and args.enable_depth
              else r.dev.start_streams(rgb=args.enable_rgb, depth=args.enable_depth))
        if not ok:
            print(f"  [{r.serial}] failed to start streams")
            return -1
        print(f"opened  serial={r.dev.get_serial_number()}  firmware={r.dev.get_firmware_version()}")

    for r in runners:
        r.start_thread()

    print(f"\nstreaming from {len(runners)} device(s) -- Ctrl-C to stop\n")

    last_report = time.monotonic()
    try:
        while True:
            if args.framemax is not None and all(r.frames >= args.framemax for r in runners):
                break

            if args.viewer_enabled:
                if render_window(np, cv2, runners):
                    break
            else:
                time.sleep(0.005)

            now = time.monotonic()
            dt = now - last_report
            if dt >= 1.0:
                cells = []
                for r in runners:
                    total = r.frames
                    r.fps_display = (total - r.last_report_frames) / dt
                    r.last_report_frames = total
                    cells.append(f"|  {r.serial} {r.fps_display:5.1f} ({total})  ")
                print("fps " + "".join(cells), flush=True)
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        # Stop capture threads, then stop+close every device (same hazard
        # protonect.py / Protonect.cpp warn about -- do it before the
        # listeners go away; the DeviceRunner objects keep them referenced
        # until the process exits below).
        for r in runners:
            r.stop_and_join()
        for r in runners:
            r.dev.stop()
            r.dev.close()
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass

    print("\nper-device totals:")
    for r in runners:
        print(f"  {r.serial}  {r.frames} frames")
    sys.stdout.flush()

    # Every device is stopped and closed and all output is flushed. Exit now
    # instead of falling through CPython's finalizer chain: pyfreenect2's
    # pybind11 objects (Freenect2 <-> Freenect2Device <-> SyncMultiFrameListener,
    # wired with reference_internal / keep_alive) are not guaranteed a safe
    # teardown order at interpreter shutdown, and with several devices open
    # that can segfault after main() has already returned. os._exit skips it.
    os._exit(0)


if __name__ == "__main__":
    # A successful run ends via os._exit(0) inside main(); only the early
    # error paths (no device opened yet) fall through to here.
    sys.exit(main(sys.argv[1:]) or 0)
