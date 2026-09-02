#!/usr/bin/env python3
"""pyfreenect2 example -- a Python port of examples/AudioVideoSync.cpp.

Opens the Kinect's Color+Depth stream and the mic array simultaneously.
Each depth/RGB frame's arrival is tagged with pyfreenect2.now_microseconds()
-- the same monotonic clock AudioFrame.timestamp_us uses -- and used to query
an AudioRingBuffer for the audio captured around that same instant.

One behavioral difference, forced by pyfreenect2's v1 scope (see
README.md#python-bindings): the C++ version's RingBufferFeeder is a push
AudioFrameListener callback, so every captured buffer reaches the ring
buffer. pyfreenect2 only exposes the polling SyncAudioListener, so this
instead runs a tight background poll thread pushing whatever's newest into
the AudioRingBuffer -- it can drop a buffer under load in a way the C++
original structurally cannot (same tradeoff as pointcloud_audio.py).

Usage:
    python audio_video_sync.py [--pythonpath <dir>] [--duration <seconds>]
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import numpy as np


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pythonpath", help="Directory containing pyfreenect2.pyd/.so")
    parser.add_argument("--duration", type=float, default=None,
                         help="Stop automatically after this many seconds (default: run until Ctrl-C)")
    args = parser.parse_args(argv)

    if args.pythonpath:
        sys.path.insert(0, args.pythonpath)
    import pyfreenect2 as f2

    # Simultaneous audio+video capture: AudioCapture(device) reuses the video
    # device's already-open USB session directly, the same way core
    # libfreenect2 itself reads video/depth -- see AudioVideoSync.cpp's file
    # header comment for why an OS audio API (WASAPI/RtAudio) can't do this
    # (it can't survive the device-wide USB reset opening the video device
    # performs).
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

    audio = f2.AudioCapture(device)
    if not audio.is_available():
        print("No Kinect v2 microphone array found -- continuing video-only.", file=sys.stderr)

    ring = f2.AudioRingBuffer(5_000_000)
    audio_listener = f2.SyncAudioListener()
    audio_running = audio.is_available() and audio.start(audio_listener)

    print(f"Streaming depth/RGB{' + audio' if audio_running else ''}. Ctrl-C to stop.")

    stop_event = threading.Event()

    def audio_poll_loop():
        while not stop_event.is_set():
            frame = audio_listener.get_latest_frame()
            if frame is not None:
                ring.push(frame)
            else:
                time.sleep(0.001)

    audio_thread = threading.Thread(target=audio_poll_loop, daemon=True)
    if audio_running:
        audio_thread.start()

    correlation_window_us = 33_000  # ~one frame period at 30fps
    start = time.monotonic()

    try:
        while args.duration is None or (time.monotonic() - start) < args.duration:
            try:
                frames = listener.wait_for_new_frame(1000)
            except RuntimeError:
                continue

            arrival_us = f2.now_microseconds()

            if audio_running:
                matched = ring.query(arrival_us, correlation_window_us)
                total_samples = sum(f.frame_count * f.channels for f in matched)
                peak = 0.0
                for af in matched:
                    if af.frame_count:
                        peak = max(peak, float(np.max(np.abs(np.asarray(af)))))
                print(f"frame t={arrival_us} us  matched={len(matched)} audio buffers "
                      f"({total_samples} samples)  peak={peak:.4f}")

            listener.release(frames)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if audio_running:
            audio_thread.join(timeout=2.0)
            audio.stop()
        device.stop()
        device.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
