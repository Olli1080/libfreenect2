/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/** @file device_reset_coordination.h Coordinates the Kinect's one-time
 * device-wide USB reset between core libfreenect2 (video/depth) and
 * freenect2_audio (mic array), whichever of the two opens the physical
 * device first in this process.
 *
 * A libusb_reset_device() call is a real USB *port-level* reset -- it
 * affects the whole composite device, not just the interfaces the caller
 * claims. If both core and freenect2_audio independently reset it (or if
 * one resets it after the other has already claimed and is actively using
 * its own interfaces), the second reset breaks whatever session the first
 * one already established. Confirmed empirically in both directions: video
 * opening after audio was already streaming broke audio's WASAPI session in
 * earlier iterations of this backend, and (with the current libusb-based
 * backend) audio starting first and claiming its interfaces without a prior
 * reset caused core's own subsequent libusb_open() for the video device to
 * fail outright, because core's open sequence always performs a reset on
 * first open and that reset collided with audio's already-active session.
 *
 * This is a same-process-only coordination primitive (std::mutex +
 * std::condition_variable) living in freenect2_logging, the one shared
 * zero-dependency library both core freenect2 and freenect2_audio already
 * link -- it does not coordinate across separate processes (e.g. a
 * standalone AudioCapture.exe and a separately-launched Protonect.exe each
 * opening the device independently are not covered; that combination keeps
 * the older "whoever opens second may fail" behavior).
 */

#ifndef DEVICE_RESET_COORDINATION_H_
#define DEVICE_RESET_COORDINATION_H_

#include <freenect2_logging/export.h>

namespace libfreenect2
{

/**
 * Call before performing the Kinect's device-wide USB reset. The first
 * caller in this process (from either core or freenect2_audio) gets
 * `true` back and is responsible for actually performing the reset, then
 * MUST call deviceResetComplete() afterward (regardless of whether the
 * reset itself succeeded) so any other threads waiting in this call can
 * proceed. Every subsequent caller blocks here until deviceResetComplete()
 * has been called, then gets `false` back, meaning: do not reset again,
 * the device has already been reset by someone else in this process.
 */
FREENECT2_LOGGING_EXPORT bool claimDeviceResetResponsibility();

/** Call exactly once, after claimDeviceResetResponsibility() returned
 * true, regardless of whether the reset attempt succeeded -- this
 * unblocks any other threads waiting in claimDeviceResetResponsibility().
 */
FREENECT2_LOGGING_EXPORT void deviceResetComplete();

} /* namespace libfreenect2 */
#endif /* DEVICE_RESET_COORDINATION_H_ */
