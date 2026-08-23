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

/** @file audio_doa.hpp Direction-of-arrival estimation (GCC-PHAT) and
 * delay-and-sum beamforming for the Kinect v2's linear 4-mic array.
 *
 * Mic array geometry -- positions along the array axis, meters, origin at
 * the array's physical center -- sourced from the CHiME-6 challenge's own
 * published diagram of "the 4-channel Kinect microphone array" (CHiME-6
 * recordings used real Kinect v2 units): total span CH1-CH4 = 226mm,
 * CH1-CH3 = 76mm, CH2-CH3 = 36mm, magnitudes CH1=113mm, CH2=73mm,
 * CH3=37mm, CH4=113mm from center. Signs are in camera-forward space
 * (the device's own view looking OUT into the room): CH1=+113mm,
 * CH4=-113mm -- the CHiME-6 diagram is a photo taken FACING the device,
 * which mirrors left/right relative to the device's own perspective, so
 * the signs are flipped from a naive reading of that photo (confirmed
 * empirically against a live source; see doa.cpp). Not independently
 * re-measured against a physical unit -- if you have one, verifying
 * against it would be worthwhile.
 *
 * Angle convention: since it's a linear array, only azimuth is resolvable
 * (no elevation), and there's a front/back ambiguity inherent to any
 * linear array (a source at +30 deg and one at 180-30 deg produce
 * identical inter-mic delays). We only care about the forward hemisphere
 * the camera looks into, so azimuth_degrees is defined as 0 = straight
 * ahead (perpendicular to the array, same direction the camera faces),
 * -90 = full left, +90 = full right -- chosen to match the point cloud's
 * own camera-space azimuth atan2(x, z), so no angle-convention conversion
 * is needed where DOA output meets point-cloud highlighting.
 */

#ifndef LIBFREENECT2_AUDIO_DOA_HPP_
#define LIBFREENECT2_AUDIO_DOA_HPP_

#include <memory>

#include <freenect2_audio/export.h>
#include <libfreenect2/audio_capture.hpp>

namespace libfreenect2
{

struct DoaEstimate
{
  float azimuth_degrees; ///< 0 = straight ahead, -90 = full left, +90 = full right
  float confidence;      ///< 0..1, roughly how distinct/sharp the estimate is; low in silence or diffuse noise
  bool valid;             ///< false until at least one full analysis window has been processed
};

class DoaEstimatorImpl;

/** Accumulates incoming AudioFrames into a rolling analysis window and
 * produces direction-of-arrival estimates via two independent methods.
 * Thread-safe: addSamples() is meant to be called from the audio capture
 * callback thread, estimate*() from any other thread (e.g. a render
 * loop). @ingroup audio */
class FREENECT2_AUDIO_EXPORT DoaEstimator
{
public:
  DoaEstimator();
  ~DoaEstimator();

  /** Feed newly captured audio. Internally accumulates into a rolling
   * window and recomputes both estimates once enough new samples have
   * arrived (see doa.cpp for window/hop size). */
  void addSamples(const AudioFrame &frame);

  /** GCC-PHAT cross-correlation direction estimate: TDOA between mic
   * pairs (all 6 pairs, not just adjacent -- the wider baselines give
   * better angular resolution), combined via least-squares fit against
   * the known geometry. */
  DoaEstimate estimateGccPhat() const;

  /** Delay-and-sum beamformer steered-response-power estimate: scans
   * candidate angles, computes delay-and-sum output energy per angle in
   * the frequency domain, returns the angle with maximum energy. */
  DoaEstimate estimateBeamforming() const;

private:
  std::unique_ptr<DoaEstimatorImpl> impl_;
};

} /* namespace libfreenect2 */
#endif /* LIBFREENECT2_AUDIO_DOA_HPP_ */
