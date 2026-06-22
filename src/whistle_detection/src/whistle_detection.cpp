#include "whistle_detection.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

namespace whistle {

WhistleDetector::WhistleDetector()
    : spec_size_(kFftFrameSize / 2 + 1),
      num_features_noise_(spec_size_ / kFrameBlockSize),
      real_buffer_(kFftFrameSize), complex_buffer_(spec_size_),
      last_frames_(std::vector<float>(num_features_noise_, 0.f)),
      accumulated_(num_features_noise_, 0.f) {
  plan_ = fftwf_plan_dft_r2c_1d(
      static_cast<int>(kFftFrameSize), real_buffer_.data(),
      reinterpret_cast<fftwf_complex *>(complex_buffer_.data()), FFTW_ESTIMATE);
}

WhistleDetector::~WhistleDetector() { fftwf_destroy_plan(plan_); }

bool WhistleDetector::processFrame(const int16_t *raw_interleaved,
                                   int frame_samples) {
  whistle_detected_ = false;
  if (!raw_interleaved || frame_samples < static_cast<int>(kFftFrameSize))
    return init_frames_ >= kNoiseToAggregate;

  constexpr int kChannels = 3;
  const int num_blocks = frame_samples / static_cast<int>(kFftFrameSize);
  for (int b = 0; b < num_blocks; ++b) {
    const int16_t *block_start =
        raw_interleaved + b * kFftFrameSize * kChannels;
    if (processOneBlock(block_start)) {
      whistle_detected_ = true;
      return init_frames_ >= kNoiseToAggregate;
    }
  }
  return init_frames_ >= kNoiseToAggregate;
}

bool WhistleDetector::processOneBlock(const int16_t *interleaved_3ch_160) {
  constexpr float kScale = 1.f / 32768.f;
  // Extract channel 0 from interleaved [ch0, ch1, ch2, ch0, ch1, ch2, ...]
  for (size_t i = 0; i < kFftFrameSize; ++i) {
    real_buffer_[i] = interleaved_3ch_160[i * 3] * kScale;
  }
  fftwf_execute(plan_);

  std::vector<float> spec_buffer(spec_size_);
  for (size_t i = 0; i < spec_size_; ++i) {
    const auto &c = complex_buffer_[i];
    spec_buffer[i] = std::isnan(c[0]) ? 0.f : std::abs(c[0]);
  }

  std::vector<float> discretized = discretize(spec_buffer);

  for (size_t i = 0; i < num_features_noise_; ++i) {
    accumulated_[i] -= last_frames_.oldest()[i];
    accumulated_[i] += discretized[i];
  }
  last_frames_.push(discretized);

  float noise = 0.f;
  for (float v : accumulated_)
    noise += v;
  noise_sum_ -= last_noise_.oldest();
  noise_sum_ += noise;
  last_noise_.push(noise);

  if (init_frames_ >= kNoiseToAggregate) {
    float noise_avg = noise_sum_ / kNoiseToAggregate /
                      static_cast<float>(num_features_noise_);
    // Hint: Adjust this constant to make the detection more or less sensitive.
    float score = 0.6050399f;
    static constexpr std::array<float, kNumFeaturesDetection> coefficients{
        -19.1711084f, -19.1711084f, 16.8218568f,
        15.6424958f,  1.1391718f,   -19.1711084f};
    for (size_t i = 0; i < kNumFeaturesDetection; ++i) {
      score += accumulated_[i] / (noise_avg * 10.f) * coefficients[i];
    }
    printf("score: %f\n", std::exp(score) / (1.f + std::exp(score)));
    return std::exp(score) / (1.f + std::exp(score)) > 0.5f;
  }
  init_frames_++;
  return false;
}

std::vector<float>
WhistleDetector::discretize(const std::vector<float> &spec_buffer) {
  std::vector<float> discretized(num_features_noise_, 0.f);
  for (size_t i = 0; i < num_features_noise_; ++i) {
    for (size_t j = i * kFrameBlockSize; j < (i + 1) * kFrameBlockSize; ++j) {
      discretized[i] += spec_buffer[j];
    }
  }
  return discretized;
}

void WhistleDetector::reset() {
  whistle_detected_ = false;
  if (init_frames_ == 0)
    return;
  init_frames_ = 0;
  last_frames_.reset(std::vector<float>(num_features_noise_, 0.f));
  std::fill(accumulated_.begin(), accumulated_.end(), 0.f);
  noise_sum_ = 0.f;
  last_noise_.reset(0.f);
}

} // namespace whistle
