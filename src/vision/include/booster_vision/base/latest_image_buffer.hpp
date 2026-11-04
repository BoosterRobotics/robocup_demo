#pragma once

#include <mutex>
#include <utility>

#include <opencv2/core.hpp>

namespace booster_vision {

// Callbacks publish snapshots; the main thread takes the newest one without
// holding this lock while HighGUI processes window events. Older frames are
// discarded so a slow display cannot build up latency or block inference.
class LatestImageBuffer {
public:
    void Store(const cv::Mat &image) {
        if (image.empty()) {
            return;
        }
        cv::Mat snapshot = image.clone();
        std::lock_guard<std::mutex> lock(mutex_);
        image_ = std::move(snapshot);
    }

    cv::Mat Take() {
        std::lock_guard<std::mutex> lock(mutex_);
        cv::Mat image;
        std::swap(image, image_);
        return image;
    }

private:
    std::mutex mutex_;
    cv::Mat image_;
};

} // namespace booster_vision
