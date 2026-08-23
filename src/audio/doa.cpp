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

/** @file doa.cpp GCC-PHAT direction estimation and delay-and-sum
 * beamforming, per doa.h.
 *
 * Math grounded in Kumatani et al., "Microphone Array Processing for
 * Distant Speech Recognition: Towards Real-World Deployment" -- far-field
 * TDOA model τ(θ) = d·cos(θ)/c (their eq. 2, θ measured from the array
 * axis), array manifold vector v(k,ω) (eq. 3), delay-and-sum weights
 * w_DS = v(k,ω)/S (eq. 5). We use azimuth-from-forward φ instead of their
 * θ-from-array-axis (φ = 90deg - θ, i.e. broadside/perpendicular-to-array
 * is φ=0 instead of θ=90deg) since that's what the point-cloud
 * azimuth (atan2(x,z)) needs directly -- so the projection onto the array
 * axis is d·sin(φ)/c here rather than d·cos(θ)/c.
 */

#include <libfreenect2/audio_doa.hpp>
#include "internal/libfreenect2/logging.h"

#include <fftw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <mutex>
#include <vector>

namespace libfreenect2
{

namespace
{
const int kChannels = 4;

// Mic positions along the array axis, meters, origin at array center --
// see doa.h for the CHiME-6-diagram-derived geometry this comes from.
// Signs are in camera-forward space (the device's own view looking OUT
// into the room), not photo space: the CHiME-6 diagram is a photo taken
// FACING the device (like looking at someone's face), where CH1 appears
// on the photo's left -- but that's the mirror image of the device's own
// left/right. CH1 is on the camera's own RIGHT (+x) looking outward, CH4
// on its own left, hence the signs below are flipped from a naive reading
// of the photo (confirmed empirically: the un-flipped signs put the
// highlighted point-cloud region on the opposite side from the actual
// sound source).
const std::array<float, kChannels> kMicPositionsM = {0.113f, 0.073f, 0.037f, -0.113f};

const float kSpeedOfSoundMps = 343.0f; // room-temperature approximation
const float kPi = 3.14159265358979323846f;

const int kWindowSize = 1024; // ~64ms @ 16kHz
const int kHopSize = 512;     // ~32ms @ 16kHz -> ~31Hz update rate
const int kFftSize = 2048;    // next pow2 >= 2*kWindowSize, avoids circular-correlation wraparound

const float kBeamAngleStepDeg = 2.0f;
const float kBeamAngleMinDeg = -90.0f;
const float kBeamAngleMaxDeg = 90.0f;

// Speech-relevant band for beamforming energy summation -- avoids DC and
// near-Nyquist bins that carry mostly noise, not directional signal.
const float kBandLowHz = 300.0f;
const float kBandHighHz = 3400.0f;

struct Pair
{
  int m, n;
};
// All 6 mic pairs, not just adjacent ones -- wider baselines (e.g. 0-3,
// the full 226mm span) give better angular resolution than any adjacent
// pair alone, at the cost of needing to search a longer lag range (still
// tiny: max delay for the widest baseline is ~226mm/343m/s =~0.66ms =~
// 10.6 samples @ 16kHz, far short of kWindowSize).
const std::array<Pair, 6> kPairs = {{{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}}};

float clamp(float v, float lo, float hi)
{
  return std::max(lo, std::min(hi, v));
}
} // namespace

class DoaEstimatorImpl
{
public:
  DoaEstimatorImpl() :
    sample_rate_(0)
  {
    real_in_ = static_cast<float *>(fftwf_malloc(sizeof(float) * kFftSize));
    for (int c = 0; c < kChannels; c++)
      channel_spectrum_[c] = static_cast<fftwf_complex *>(fftwf_malloc(sizeof(fftwf_complex) * (kFftSize / 2 + 1)));

    forward_plan_ = fftwf_plan_dft_r2c_1d(kFftSize, real_in_, channel_spectrum_[0], FFTW_ESTIMATE);

    gcc_spectrum_ = static_cast<fftwf_complex *>(fftwf_malloc(sizeof(fftwf_complex) * (kFftSize / 2 + 1)));
    gcc_time_ = static_cast<float *>(fftwf_malloc(sizeof(float) * kFftSize));
    inverse_plan_ = fftwf_plan_dft_c2r_1d(kFftSize, gcc_spectrum_, gcc_time_, FFTW_ESTIMATE);
  }

  ~DoaEstimatorImpl()
  {
    fftwf_destroy_plan(forward_plan_);
    fftwf_destroy_plan(inverse_plan_);
    fftwf_free(real_in_);
    fftwf_free(gcc_spectrum_);
    fftwf_free(gcc_time_);
    for (int c = 0; c < kChannels; c++)
      fftwf_free(channel_spectrum_[c]);
  }

  void addSamples(const AudioFrame &frame)
  {
    if (frame.channels != kChannels || frame.frame_count == 0)
      return;

    std::lock_guard<std::mutex> lock(mutex_);
    sample_rate_ = frame.sample_rate;

    for (size_t i = 0; i < frame.frame_count; i++)
    {
      for (int c = 0; c < kChannels; c++)
        channel_buffer_[c].push_back(frame.samples[i * kChannels + c]);
    }

    while (static_cast<int>(channel_buffer_[0].size()) >= kWindowSize)
    {
      processWindow();
      for (int c = 0; c < kChannels; c++)
        channel_buffer_[c].erase(channel_buffer_[c].begin(), channel_buffer_[c].begin() + kHopSize);
    }
  }

  DoaEstimate estimateGccPhat() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return gcc_estimate_;
  }

  DoaEstimate estimateBeamforming() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return beam_estimate_;
  }

private:
  // Computes each channel's FFT for the current front-of-buffer window,
  // then both estimates from those spectra. Called with mutex_ already
  // held (from addSamples()).
  void processWindow()
  {
    for (int c = 0; c < kChannels; c++)
    {
      for (int i = 0; i < kWindowSize; i++)
        real_in_[i] = channel_buffer_[c][i];
      for (int i = kWindowSize; i < kFftSize; i++)
        real_in_[i] = 0.0f;

      fftwf_execute_dft_r2c(forward_plan_, real_in_, channel_spectrum_[c]);
    }

    computeGccPhat();
    computeBeamforming();
  }

  void computeGccPhat()
  {
    const int max_lag_samples = static_cast<int>(std::ceil(0.226f / kSpeedOfSoundMps * sample_rate_)) + 1;
    // Reference range for the correlation "noise floor", well outside any
    // physically possible TDOA for this array -- a real, sharp GCC-PHAT
    // peak should stand out against typical values here, whereas comparing
    // only against the tiny +-max_lag_samples window (as an earlier version
    // of this code did) doesn't give enough samples to tell a real peak
    // from a spurious one: PHAT-normalized spectra have roughly uniform
    // magnitude everywhere, so even pure noise produces *some* local bump
    // within a ~23-sample window just by chance.
    const int kNoiseFloorLo = kFftSize / 4;
    const int kNoiseFloorHi = 3 * kFftSize / 4;

    struct PairResult
    {
      float tau, d, sharpness;
    };
    std::array<PairResult, kPairs.size()> results{};

    float weighted_tau_sum = 0.0f;
    float weight_sum = 0.0f;
    float weighted_sharpness_sum = 0.0f;

    for (size_t p = 0; p < kPairs.size(); p++)
    {
      const Pair &pair = kPairs[p];
      const fftwf_complex *Xm = channel_spectrum_[pair.m];
      const fftwf_complex *Xn = channel_spectrum_[pair.n];
      const int num_bins = kFftSize / 2 + 1;

      for (int k = 0; k < num_bins; k++)
      {
        // cross-power spectrum: Xm * conj(Xn)
        float re = Xm[k][0] * Xn[k][0] + Xm[k][1] * Xn[k][1];
        float im = Xm[k][1] * Xn[k][0] - Xm[k][0] * Xn[k][1];
        float mag = std::sqrt(re * re + im * im) + 1e-9f;
        // PHAT weighting: normalize to unit magnitude, keep phase only
        gcc_spectrum_[k][0] = re / mag;
        gcc_spectrum_[k][1] = im / mag;
      }

      fftwf_execute_dft_c2r(inverse_plan_, gcc_spectrum_, gcc_time_);

      // Search lags in [-max_lag_samples, +max_lag_samples]. Positive
      // lags are gcc_time_[0..max_lag], negative lags wrap to the end of
      // the kFftSize-length circular result.
      int best_lag = 0;
      float best_val = gcc_time_[0];
      for (int lag = 1; lag <= max_lag_samples; lag++)
      {
        if (gcc_time_[lag] > best_val)
        {
          best_val = gcc_time_[lag];
          best_lag = lag;
        }
        int wrapped = kFftSize - lag;
        if (gcc_time_[wrapped] > best_val)
        {
          best_val = gcc_time_[wrapped];
          best_lag = -lag;
        }
      }

      float tau = static_cast<float>(best_lag) / static_cast<float>(sample_rate_);
      float d = kMicPositionsM[pair.n] - kMicPositionsM[pair.m];

      // Sharpness: peak value vs. RMS correlation over a wide reference
      // range far from any physically-plausible lag (see above).
      float noise_sum_sq = 0.0f;
      for (int i = kNoiseFloorLo; i < kNoiseFloorHi; i++)
        noise_sum_sq += gcc_time_[i] * gcc_time_[i];
      float noise_rms = std::sqrt(noise_sum_sq / static_cast<float>(kNoiseFloorHi - kNoiseFloorLo));
      float sharpness = best_val / (noise_rms + 1e-9f);

      results[p] = PairResult{tau, d, sharpness};

      // Least-squares combination across pairs. NOTE the sign: IFFT(Xm *
      // conj(Xn)) evaluated at lag tau equals sum_t x_m(t) x_n(t-tau),
      // which is x_m's autocorrelation at lag (D+tau) when x_n(t) is x_m
      // delayed by D seconds -- so it peaks at tau=-D, i.e. our measured
      // "tau" is the NEGATIVE of how much later channel n arrives than m.
      // The physical model (matching computeBeamforming()'s convention,
      // verified correct against a live directional source) is
      // (delay of n) - (delay of m) = d_pair*sin(phi)/c where
      // d_pair = pos_n - pos_m; measured tau is the negative of that
      // left-hand side, so we fit tau_pair = -d_pair*sin(phi)/c here and
      // correct the sign back out once, below, rather than negating d
      // itself (which is also used unsigned-ly for weighting).
      float weight = std::fabs(d);
      weighted_tau_sum += weight * (tau * kSpeedOfSoundMps * d);
      weight_sum += weight * (d * d);
      weighted_sharpness_sum += weight * sharpness;
    }

    // Raw fit of tau_pair = d_pair * s / c (see sign note above) -- s is
    // -sin(phi), not sin(phi); kept as the fit variable so it stays
    // consistent with the residual computation below, negated only once
    // when actually producing the azimuth.
    float s_fit = weight_sum > 1e-12f ? weighted_tau_sum / weight_sum : 0.0f;
    float sin_phi = clamp(-s_fit, -1.0f, 1.0f);
    float azimuth_deg = std::asin(sin_phi) * 180.0f / kPi;

    // Confidence combines two independent signals: how sharp the
    // correlation peaks are on average (weighted the same way the angle
    // combination itself is), and how well the individual pairs' measured
    // TDOAs actually agree with the single fitted angle -- a real,
    // spatially coherent source should have all 6 pairs agreeing closely;
    // noise tends to give inconsistent per-pair estimates even if any one
    // pair's peak looks locally sharp.
    float avg_sharpness = weight_sum > 1e-12f ? weighted_sharpness_sum / weight_sum : 0.0f;
    const float kSharpnessScale = 4.0f;
    float sharpness_confidence = clamp((avg_sharpness - 1.0f) / kSharpnessScale, 0.0f, 1.0f);

    float residual_sum_sq = 0.0f;
    float residual_weight_sum = 0.0f;
    for (const PairResult &r : results)
    {
      float predicted_tau = r.d * s_fit / kSpeedOfSoundMps; // same convention as the fit itself, see sign note above
      float w = std::fabs(r.d);
      residual_sum_sq += w * (r.tau - predicted_tau) * (r.tau - predicted_tau);
      residual_weight_sum += w;
    }
    float residual_rms_s = residual_weight_sum > 1e-12f ? std::sqrt(residual_sum_sq / residual_weight_sum) : 0.0f;
    // Scale: residual comparable to one sample period is "consistent
    // enough", several sample periods off is not.
    float residual_scale_s = 3.0f / static_cast<float>(sample_rate_);
    float fit_confidence = std::exp(-residual_rms_s / residual_scale_s);

    float confidence = clamp(sharpness_confidence * fit_confidence, 0.0f, 1.0f);

    gcc_estimate_ = DoaEstimate{azimuth_deg, confidence, true};
  }

  void computeBeamforming()
  {
    const int num_bins = kFftSize / 2 + 1;
    const int k_min = std::max(1, static_cast<int>(kBandLowHz * kFftSize / sample_rate_));
    const int k_max = std::min(num_bins - 1, static_cast<int>(kBandHighHz * kFftSize / sample_rate_));

    const int num_angles = static_cast<int>((kBeamAngleMaxDeg - kBeamAngleMinDeg) / kBeamAngleStepDeg) + 1;
    std::vector<float> energies(num_angles);

    for (int a = 0; a < num_angles; a++)
    {
      float angle_deg = kBeamAngleMinDeg + a * kBeamAngleStepDeg;
      float sin_phi = std::sin(angle_deg * kPi / 180.0f);

      std::array<float, kChannels> tau;
      for (int c = 0; c < kChannels; c++)
        tau[c] = kMicPositionsM[c] * sin_phi / kSpeedOfSoundMps;

      float energy = 0.0f;
      for (int k = k_min; k <= k_max; k++)
      {
        float omega = 2.0f * kPi * static_cast<float>(k) * static_cast<float>(sample_rate_) / static_cast<float>(kFftSize);

        float sum_re = 0.0f, sum_im = 0.0f;
        for (int c = 0; c < kChannels; c++)
        {
          // delay-and-sum weight for this channel/angle: e^{+i*omega*tau_c}/S (eq. 5, Hermitian)
          float phase = omega * tau[c];
          float wc = std::cos(phase), ws = std::sin(phase);
          const float *Xc = channel_spectrum_[c][k];
          // (wc + i*ws) * (Xc_re + i*Xc_im)
          sum_re += wc * Xc[0] - ws * Xc[1];
          sum_im += wc * Xc[1] + ws * Xc[0];
        }
        sum_re /= kChannels;
        sum_im /= kChannels;
        energy += sum_re * sum_re + sum_im * sum_im;
      }

      energies[a] = energy;
    }

    int best_idx = static_cast<int>(std::max_element(energies.begin(), energies.end()) - energies.begin());
    float best_energy = energies[best_idx];
    float best_angle = kBeamAngleMinDeg + best_idx * kBeamAngleStepDeg;

    // Background energy: mean over angles OUTSIDE the beam's own mainlobe
    // around the peak, not the mean over all angles including it -- a
    // real delay-and-sum mainlobe with this array (226mm aperture) is
    // roughly 20-30 degrees wide depending on frequency, so including it
    // in the "background" dilutes the contrast and makes anything but a
    // very loud, sharp transient (e.g. a clap) look unconvincing even
    // when there IS a real, if quieter/more continuous, source (e.g.
    // speech).
    const float kMainlobeHalfWidthDeg = 15.0f;
    float background_sum = 0.0f;
    int background_count = 0;
    for (int a = 0; a < num_angles; a++)
    {
      float angle_deg = kBeamAngleMinDeg + a * kBeamAngleStepDeg;
      if (std::fabs(angle_deg - best_angle) < kMainlobeHalfWidthDeg)
        continue;
      background_sum += energies[a];
      background_count++;
    }
    float background_energy = background_count > 0 ? background_sum / static_cast<float>(background_count) : 0.0f;

    const float kContrastScale = 2.5f;
    float confidence = clamp((best_energy / (background_energy + 1e-9f) - 1.0f) / kContrastScale, 0.0f, 1.0f);

    beam_estimate_ = DoaEstimate{best_angle, confidence, true};
  }

  mutable std::mutex mutex_;
  std::array<std::deque<float>, kChannels> channel_buffer_;
  int sample_rate_;

  float *real_in_;
  std::array<fftwf_complex *, kChannels> channel_spectrum_;
  fftwf_plan forward_plan_;

  fftwf_complex *gcc_spectrum_;
  float *gcc_time_;
  fftwf_plan inverse_plan_;

  DoaEstimate gcc_estimate_{0.0f, 0.0f, false};
  DoaEstimate beam_estimate_{0.0f, 0.0f, false};
};

DoaEstimator::DoaEstimator() : impl_(std::make_unique<DoaEstimatorImpl>())
{
}

DoaEstimator::~DoaEstimator() = default;

void DoaEstimator::addSamples(const AudioFrame &frame)
{
  impl_->addSamples(frame);
}

DoaEstimate DoaEstimator::estimateGccPhat() const
{
  return impl_->estimateGccPhat();
}

DoaEstimate DoaEstimator::estimateBeamforming() const
{
  return impl_->estimateBeamforming();
}

} /* namespace libfreenect2 */
