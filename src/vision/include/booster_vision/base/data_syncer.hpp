#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <booster_vision/base/pose.h>

namespace booster_vision {
template <typename T>
struct DataBlock {
    T data;
    double timestamp;

    DataBlock() :
        timestamp(0) {
    }
    DataBlock(const T &data, double timestamp) :
        data(data), timestamp(timestamp) {
    }
};

using ColorDataBlock = DataBlock<cv::Mat>;
using DepthDataBlock = DataBlock<cv::Mat>;
using PoseDataBlock = DataBlock<Pose>;

struct SyncedDataBlock {
    SyncedDataBlock() = default;
    SyncedDataBlock(const SyncedDataBlock &other) :
        pose_data(other.pose_data) {
        color_data.timestamp = other.color_data.timestamp;
        if (other.color_data.data.empty()) {
            color_data.data = cv::Mat();
        } else {
            other.color_data.data.copyTo(color_data.data);
        }

        depth_data.timestamp = other.depth_data.timestamp;
        if (other.depth_data.data.empty()) {
            depth_data.data = cv::Mat();
        } else {
            other.depth_data.data.copyTo(depth_data.data);
        }
    }

    ColorDataBlock color_data;
    DepthDataBlock depth_data;
    PoseDataBlock pose_data;
};

template <int MaxLength, typename T, typename Allocator = std::allocator<T>>
class DataBuffer : public std::deque<T, Allocator> {
public:
    DataBuffer() :
        std::deque<T, Allocator>(MaxLength) {
    }
    void push_back(const T &value) {
        if (this->size() == MaxLength) {
            this->pop_front();
        }
        std::deque<T, Allocator>::push_back(value);
    }
};

const int kDepthBufferLength = 30;
const int kPoseBufferLength = 500;

class DataSyncer {
public:
    DataSyncer(bool enable_depth) :
        enable_depth_(enable_depth) {
    }

    void LoadData(const std::string &data_dir);

    void AddDepth(const DepthDataBlock &depth_data);
    void AddPose(const PoseDataBlock &pose_data);

    SyncedDataBlock getSyncedDataBlock(const ColorDataBlock &color_data);
    SyncedDataBlock getSyncedDataBlock();

private:
    bool enable_depth_;

    std::mutex depth_buffer_mutex_;
    std::mutex pose_buffer_mutex_;

    int data_index_;
    std::string data_dir_;
    std::vector<double> time_stamp_list_;

    using DepthBuffer = DataBuffer<kDepthBufferLength, DepthDataBlock>;
    using PoseBuffer = DataBuffer<kPoseBufferLength, PoseDataBlock>;

    DepthBuffer depth_buffer_;
    PoseBuffer pose_buffer_;
};

} // namespace booster_vision
