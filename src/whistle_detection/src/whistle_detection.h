#ifndef WHISTLE_WHISTLE_DETECTION_H
#define WHISTLE_WHISTLE_DETECTION_H

#include "fftw_allocator.h"
#include "ring_buffer.h"

#include <array>
#include <cstdint>
#include <fftw3.h>
#include <vector>

namespace whistle {

/**
 * Whistle detection for Robocup: process RAW output from the Booster mic
 * (3 channels, 16 kHz, S16).
 *
 * One frame = FRAMES samples per channel (e.g. 1024). Internally uses
 * 160-sample FFT frames (~10 ms at 16 kHz).
 */
class WhistleDetector {
public:
  static constexpr uint32_t kSampleRate = 16000;
  /** FFT frame size: 10 ms at 16 kHz. */
  static constexpr uint32_t kFftFrameSize = 160;

  WhistleDetector();
  ~WhistleDetector();
  WhistleDetector(const WhistleDetector &) = delete;
  WhistleDetector &operator=(const WhistleDetector &) = delete;

  /** Process one frame of RAW audio (3 channels, interleaved; length = 3 *
   * frame_samples).
   *  Returns true when the noise floor calibration is complete (after ~10 s). */
  bool processFrame(const int16_t *raw_interleaved, int frame_samples);

  /** True if a whistle was detected in the last processed frame(s). */
  bool whistleDetected() const { return whistle_detected_; }

  /** Reset detector state (e.g. after handling a whistle). */
  void reset();

private:
  static constexpr int32_t kFrameBlockSize = 10;
  static constexpr int32_t kFramesToAggregate = 60;  // 0.6 seconds
  static constexpr int32_t kNoiseToAggregate = 1000; // 10 seconds
  static constexpr size_t kNumFeaturesDetection = 6;

  /** Process one 160-sample block of interleaved 3-channel S16; uses channel 0.
   *  Returns true if whistle detected. */
  bool processOneBlock(const int16_t *interleaved_3ch_160);

  std::vector<float> discretize(const std::vector<float> &spec_buffer);

  const uint32_t spec_size_;
  const size_t num_features_noise_;
  fftwf_vector<float> real_buffer_;
  /** FFTW complex output; std::array<float,2> is layout-compatible with
   * fftwf_complex. */
  fftwf_vector<std::array<float, 2>> complex_buffer_;
  fftwf_plan plan_;
  RingBuffer<std::vector<float>, kFramesToAggregate> last_frames_;
  std::vector<float> accumulated_;
  float noise_sum_ = 0.f;
  RingBuffer<float, kNoiseToAggregate> last_noise_{0.f};
  int64_t init_frames_ = 0;

  bool whistle_detected_ = false;
};

} // namespace whistle

#endif // WHISTLE_WHISTLE_DETECTION_H
