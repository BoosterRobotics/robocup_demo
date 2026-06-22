/**
 * Whistle detection for Booster robot microphone (libduerwen).
 * Pipeline: 6ch ALSA -> Duerwen DSP -> AEC 3ch -> FFT whistle detection.
 * Publishes to /whistle_detected ROS2 topic when whistle is heard.
 */

#include "whistle_detection.h"

extern "C" {
#include <RecvDataCache.h>
#include <WakeupApi.h>
#include <duerwen_alsa.h>
}

#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

constexpr int kChannels = 6;
constexpr int kFrames = 1024;
constexpr int kSampleRate = 16000;
constexpr const char *kPcmDevice = "hw:1,0";
constexpr const char *kWhistleTopic = "/whistle_detected";

volatile sig_atomic_t g_running = 1;

void sigintHandler(int) { g_running = 0; }

#pragma pack(push, 1)
struct WavHeader {
  char chunk_id[4];
  uint32_t chunk_size;
  char format[4];
  char sub_chunk1_id[4];
  uint32_t sub_chunk1_size;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char sub_chunk2_id[4];
  uint32_t sub_chunk2_size;
};
#pragma pack(pop)

void writeWavHeader(FILE *file, int channels, int sample_rate,
                    int bits_per_sample) {
  WavHeader h = {};
  std::memcpy(h.chunk_id, "RIFF", 4);
  h.chunk_size = 0;
  std::memcpy(h.format, "WAVE", 4);
  std::memcpy(h.sub_chunk1_id, "fmt ", 4);
  h.sub_chunk1_size = 16;
  h.audio_format = 1;
  h.num_channels = static_cast<uint16_t>(channels);
  h.sample_rate = static_cast<uint32_t>(sample_rate);
  h.bits_per_sample = static_cast<uint16_t>(bits_per_sample);
  h.byte_rate = sample_rate * channels * bits_per_sample / 8;
  h.block_align = channels * bits_per_sample / 8;
  std::memcpy(h.sub_chunk2_id, "data", 4);
  h.sub_chunk2_size = 0;
  std::fwrite(&h, sizeof(WavHeader), 1, file);
}

void finalizeWavHeader(FILE *file) {
  if (!file) return;
  uint32_t file_size = static_cast<uint32_t>(std::ftell(file));
  uint32_t chunk_size = file_size - 8;
  uint32_t data_size = file_size - sizeof(WavHeader);
  std::fseek(file, 4, SEEK_SET);
  std::fwrite(&chunk_size, 4, 1, file);
  std::fseek(file, 40, SEEK_SET);
  std::fwrite(&data_size, 4, 1, file);
}

void printUsage(const char *prog) {
  std::fprintf(stderr, "Usage: %s [-r|--record <output.wav>]\n", prog);
  std::fprintf(stderr, "  -r, --record   Record 3ch RAW output to a WAV file.\n");
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("whistle_detection");
  auto publisher = node->create_publisher<std_msgs::msg::String>(kWhistleTopic, 10);

  const char *record_path = nullptr;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "-r") == 0 || std::strcmp(argv[i], "--record") == 0) {
      if (i + 1 >= argc) {
        printUsage(argv[0]);
        rclcpp::shutdown();
        return 1;
      }
      record_path = argv[++i];
    } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
      printUsage(argv[0]);
      rclcpp::shutdown();
      return 0;
    }
  }

  std::signal(SIGINT, sigintHandler);

  std::vector<unsigned char> cache_buf(1024 * 64, 0);
  RecvDataCacheInfo alsa_cache;
  RecvDataCacheInit(&alsa_cache, cache_buf.data(), static_cast<unsigned int>(cache_buf.size()));

  HWWakeup wakeup_handle = nullptr;
  int ret = Duerwen_wakeup_init(&wakeup_handle, 0);
  if (ret != 0) {
    std::fprintf(stderr, "Duerwen_wakeup_init failed: %d\n", ret);
    rclcpp::shutdown();
    return 1;
  }

  void *alsa_handle = nullptr;
  ret = duerwen_alsa_init(&alsa_handle, const_cast<char *>(kPcmDevice),
                          kChannels, kSampleRate, SND_PCM_STREAM_CAPTURE);
  if (ret != 0) {
    std::fprintf(stderr, "duerwen_alsa_init failed: %d\n", ret);
    Duerwen_wakeup_unit(wakeup_handle);
    rclcpp::shutdown();
    return 1;
  }

  std::vector<int16_t> sources(kFrames * kChannels);
  std::vector<int16_t> mic1(kFrames), mic2(kFrames), mic3(kFrames);
  std::vector<int16_t> ref1(kFrames);
  std::vector<int16_t> aec1(kFrames), aec2(kFrames), aec3(kFrames);
  std::vector<int16_t> raw_interleaved(kFrames * 3);
  std::vector<int16_t> naec(kFrames);

  FILE *record_file = nullptr;
  if (record_path) {
    record_file = std::fopen(record_path, "wb");
    if (!record_file) {
      std::fprintf(stderr, "Cannot open record file: %s\n", record_path);
      duerwen_alsa_unit(alsa_handle);
      Duerwen_wakeup_unit(wakeup_handle);
      rclcpp::shutdown();
      return 1;
    }
    writeWavHeader(record_file, 3, kSampleRate, 16);
    RCLCPP_INFO(node->get_logger(), "Recording 3ch RAW to %s", record_path);
  }

  whistle::WhistleDetector detector;
  bool was_ready = false;

  RCLCPP_INFO(node->get_logger(), "Whistle detection started on topic '%s'", kWhistleTopic);
  RCLCPP_INFO(node->get_logger(), "Whistle detection calibrating (need ~10s of ambient noise)...");

  while (g_running && rclcpp::ok()) {
    int frames = duerwen_alsa_read(alsa_handle, sources.data());
    if (frames <= 0) {
      continue;
    }
    if (frames != kFrames) {
      continue;
    }

    for (int i = 0; i < kFrames; ++i) {
      mic1[i] = sources[i * kChannels + 0];
      mic2[i] = sources[i * kChannels + 1];
      mic3[i] = sources[i * kChannels + 2];
      ref1[i] = sources[i * kChannels + 4];
    }

    ret = Duerwen_wakeup_three_write_data(
        wakeup_handle, mic1.data(), mic2.data(), mic3.data(), ref1.data(),
        aec1.data(), aec2.data(), aec3.data(), naec.data());

    for (int i = 0; i < kFrames; ++i) {
      raw_interleaved[i * 3 + 0] = aec1[i];
      raw_interleaved[i * 3 + 1] = aec2[i];
      raw_interleaved[i * 3 + 2] = aec3[i];
    }

    if (record_file) {
      std::fwrite(raw_interleaved.data(), sizeof(int16_t), static_cast<size_t>(kFrames * 3), record_file);
    }

    bool ready = detector.processFrame(raw_interleaved.data(), kFrames);
    if (ready && !was_ready) {
      RCLCPP_INFO(node->get_logger(), "Whistle detection ready (noise floor calibrated)");
      was_ready = true;
    }

    if (detector.whistleDetected()) {
      RCLCPP_INFO(node->get_logger(), "Whistle DETECTED! Publishing to '%s'", kWhistleTopic);
      std_msgs::msg::String msg;
      msg.data = "whistle_detected";
      publisher->publish(msg);
      detector.reset();
      RCLCPP_INFO(node->get_logger(), "Whistle detection recalibrating...");
      was_ready = false;
    }
  }

  if (record_file) {
    finalizeWavHeader(record_file);
    std::fclose(record_file);
  }

  Duerwen_wakeup_unit(wakeup_handle);
  duerwen_alsa_unit(alsa_handle);

  rclcpp::shutdown();
  return 0;
}
