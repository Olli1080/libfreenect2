#!/usr/bin/env python3
"""pyfreenect2 example -- a Python port of examples/AudioCapture.cpp.

Minimal Kinect v2 microphone array capture: prints a timestamp and rough
peak amplitude for each captured audio buffer until stopped.

One behavioral difference, forced by pyfreenect2's v1 scope (see
README.md#python-bindings): the C++ version's PrintingListener is a push
AudioFrameListener callback and never misses a buffer. pyfreenect2 only
exposes the polling SyncAudioListener (latest-frame-wins), so this prints
whatever's newest each time the tight poll loop catches up -- it can skip a
buffer under load in a way the C++ original structurally cannot.

Usage:
    python audio_capture.py [--pythonpath <dir>] [--duration <seconds>]
"""

from __future__ import annotations

import argparse
import sys
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

    capture = f2.AudioCapture()
    if not capture.is_available():
        print("No Kinect v2 microphone array found.", file=sys.stderr)
        return 1

    listener = f2.SyncAudioListener()
    if not capture.start(listener):
        print("Failed to start audio capture.", file=sys.stderr)
        return 1

    print("Capturing from Kinect v2 mic array. Ctrl-C to stop.")

    start = time.monotonic()
    try:
        while args.duration is None or (time.monotonic() - start) < args.duration:
            frame = listener.get_latest_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            # AudioFrame exposes its samples via the buffer protocol (see
            # bind_audio.cpp), not a `.samples` attribute -- np.asarray(frame)
            # gives a (frame_count, channels) float32 view.
            peak = float(np.max(np.abs(np.asarray(frame)))) if frame.frame_count else 0.0
            print(f"t={frame.timestamp_us} us  frames={frame.frame_count}  "
                  f"ch={frame.channels}  rate={frame.sample_rate}  peak={peak:.4f}")
    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
