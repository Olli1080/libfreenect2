#!/usr/bin/env python3
"""pyfreenect2 example -- a Python port of examples/Protonect.cpp.

Mirrors Protonect's CLI and behavior as closely as pyfreenect2's v1 binding
surface allows: pipeline selection, RGB/depth stream toggles, frame limit,
device serial selection, and a live OpenCV-based viewer standing in for
Protonect's OpenGL one (pyfreenect2 doesn't bind an OpenGL viewer).

Accepted arguments (same spelling as Protonect.cpp):
    [-gpu=<id>] [gl | cl | clkde | cuda | cudakde | vk | cpu] [<device serial>]
    [-noviewer] [-norgb | -nodepth] [-help] [-version]
    [-frames <number of frames to process>]

Plus one addition Protonect.cpp doesn't need (it's a compiled binary that
already knows where its own libraries are): --pythonpath <dir>, pointing at
the directory containing the built pyfreenect2 extension module (e.g.
build/vs2026-vcpkg-all/bin) if it isn't already on PYTHONPATH.

Requires: numpy, opencv-python (`pip install numpy opencv-python`) for the
viewer; pyfreenect2 itself has no such dependency (see README.md#python-bindings).

Not ported from Protonect.cpp: the SIGUSR1 pause/unpause handler (POSIX-only,
and pyfreenect2 doesn't bind Freenect2Device.start()/stop() as safe to call
concurrently from a signal handler thread) and custom file-logger setup
(pyfreenect2 only exposes set_log_level(), not custom spdlog sinks -- see
README.md#python-bindings's documented v1 limitations).
"""

from __future__ import annotations

import argparse
import sys


def parse_args(argv: list[str]):
    pipeline_names = {"cpu", "gl", "cl", "clkde", "cuda", "cudakde", "vk", "vulkan"}

    pipeline_name = None
    device_id = -1
    serial = ""
    viewer_enabled = True
    enable_rgb = True
    enable_depth = True
    framemax = None
    pythonpath = None

    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg in ("-help", "--help", "-h", "-v", "--version", "-version"):
            print(__doc__)
            raise SystemExit(0)
        elif arg.startswith("-gpu="):
            if pipeline_name is not None:
                print("-gpu must be specified before pipeline argument", file=sys.stderr)
                raise SystemExit(1)
            device_id = int(arg[len("-gpu="):])
        elif arg in pipeline_names:
            if pipeline_name is None:
                pipeline_name = "vk" if arg == "vulkan" else arg
        elif arg.isdigit():
            serial = arg
        elif arg in ("-noviewer", "--noviewer"):
            viewer_enabled = False
        elif arg in ("-norgb", "--norgb"):
            enable_rgb = False
        elif arg in ("-nodepth", "--nodepth"):
            enable_depth = False
        elif arg == "-frames":
            i += 1
            framemax = int(argv[i])
        elif arg == "--pythonpath":
            i += 1
            pythonpath = argv[i]
        else:
            print(f"Unknown argument: {arg}")
        i += 1

    if not enable_rgb and not enable_depth:
        print("Disabling both streams is not allowed!", file=sys.stderr)
        raise SystemExit(1)

    return argparse.Namespace(
        pipeline_name=pipeline_name,
        device_id=device_id,
        serial=serial,
        viewer_enabled=viewer_enabled,
        enable_rgb=enable_rgb,
        enable_depth=enable_depth,
        framemax=framemax,
        pythonpath=pythonpath,
    )


def make_pipeline(f2, name, device_id):
    """None name -> let libfreenect2 pick its default. Otherwise a canonical
    token (cpu/gl/cl/clkde/cuda/cudakde/vk) or alias, via the shared registry."""
    if name is None:
        return None
    pipeline = f2.create_pipeline(name, device_id)
    if pipeline is None:
        print(f"pipeline '{name}' is not available in this build")
    return pipeline


def depth_to_bgr(np, cv2, depth, max_depth_mm=4500.0):
    vis = depth.copy()
    vis[~(vis > 0)] = 0.0  # non-positive/NaN/inf are invalid (see frame_listener.hpp)
    vis = (np.clip(vis, 0.0, max_depth_mm) / max_depth_mm * 255.0).astype(np.uint8)
    return cv2.applyColorMap(vis, cv2.COLORMAP_JET)


def ir_to_bgr(np, cv2, ir):
    vis = np.sqrt(np.clip(ir, 0.0, 65535.0) / 65535.0)
    vis = (vis * 255.0).astype(np.uint8)
    return cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if args.pythonpath:
        sys.path.insert(0, args.pythonpath)

    import numpy as np
    import pyfreenect2 as f2

    if args.viewer_enabled:
        import cv2
    else:
        cv2 = None

    print(f"Environment variables: LIBFREENECT2_LOGGER_LEVEL=<none|error|warning|info|debug>")
    print("Usage: protonect.py [--pythonpath <dir>] [-gpu=<id>] [gl | cl | clkde | cuda | cudakde | vk | cpu] [<device serial>]")
    print("        [-noviewer] [-norgb | -nodepth] [-help] [-version]")
    print("        [-frames <number of frames to process>]")

    fn = f2.Freenect2()
    if fn.enumerate_devices() == 0:
        print("no device connected!")
        return -1

    serial = args.serial or fn.get_default_device_serial_number()

    pipeline = make_pipeline(f2, args.pipeline_name, args.device_id)
    dev = fn.open_device(serial, pipeline) if pipeline is not None else fn.open_device(serial)
    if dev is None:
        print("failure opening device!")
        return -1

    frame_types = 0
    if args.enable_rgb:
        frame_types |= f2.FrameType.Color
    if args.enable_depth:
        frame_types |= f2.FrameType.Ir | f2.FrameType.Depth
    listener = f2.SyncMultiFrameListener(frame_types)

    dev.set_color_frame_listener(listener)
    dev.set_ir_and_depth_frame_listener(listener)

    if args.enable_rgb and args.enable_depth:
        if not dev.start():
            return -1
    elif not dev.start_streams(rgb=args.enable_rgb, depth=args.enable_depth):
        return -1

    print(f"device serial: {dev.get_serial_number()}")
    print(f"device firmware: {dev.get_firmware_version()}")

    registration = None
    undistorted = registered = None
    if args.enable_rgb and args.enable_depth:
        registration = f2.Registration(dev.get_ir_camera_params(), dev.get_color_camera_params())
        undistorted = f2.Frame(512, 424, 4)
        registered = f2.Frame(512, 424, 4)

    framecount = 0
    try:
        while args.framemax is None or framecount < args.framemax:
            try:
                frames = listener.wait_for_new_frame(10_000)
            except RuntimeError:
                print("timeout!")
                return -1

            rgb = frames.get(f2.FrameType.Color)
            ir = frames.get(f2.FrameType.Ir)
            depth = frames.get(f2.FrameType.Depth)

            if registration is not None:
                registration.apply(rgb, depth, undistorted, registered)

            framecount += 1

            if not args.viewer_enabled:
                if framecount % 100 == 0:
                    print(f"The viewer is turned off. Received {framecount} frames. Ctrl-C to stop.")
                listener.release(frames)
                continue

            if rgb is not None:
                cv2.imshow("RGB", cv2.resize(np.asarray(rgb), (960, 540)))
            if ir is not None:
                cv2.imshow("ir", ir_to_bgr(np, cv2, np.asarray(ir)))
            if depth is not None:
                cv2.imshow("depth", depth_to_bgr(np, cv2, np.asarray(depth)))
            if registered is not None:
                cv2.imshow("registered", np.asarray(registered))

            listener.release(frames)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        # bad things happen if frame listeners are freed before dev.stop()
        # (same warning as Protonect.cpp) -- `listener`/`registration` stay
        # referenced by this function's locals until after stop()/close().
        dev.stop()
        dev.close()
        if cv2 is not None:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
