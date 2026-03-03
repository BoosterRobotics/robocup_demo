#pragma once

#include <string>
#include <fstream>
#include <filesystem>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <booster_vision/base/data_syncer.hpp>
#include <booster_vision/base/pose.h>
#include <booster_vision/base/misc_utils.hpp>

namespace booster_vision {
class DataLogger {
public:
    DataLogger(const std::string& log_root, bool save_data_nonstationary = false) :
        log_root_(log_root),
        save_data_nonstationary_(save_data_nonstationary),
        stop_logging_(false) {
        if (!std::filesystem::exists(log_root_)) {
            std::filesystem::create_directories(log_root_);
            std::cout << "creating log directory: " << log_root_ << std::endl;
        }
        logging_thread_ = std::thread(&DataLogger::ProcessQueue, this);
    }

    ~DataLogger() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_logging_ = true;
        }
        queue_cv_.notify_all();
        logging_thread_.join();
    }

    void LogYAML(const YAML::Node &node, const std::string &file_name) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        std::string full_file_path = log_root_ + "/" + file_name;
        YAML::Node node_copy = YAML::Clone(node); // Make a deep copy of the YAML node
        log_queue_.emplace([full_file_path, node_copy]() {
            std::ofstream fout(full_file_path);
            fout << node_copy;
        });
        queue_cv_.notify_one();
    }

    void LogImage(const cv::Mat &image, const std::string &file_name) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        std::string full_file_path = log_root_ + "/" + file_name;
        cv::Mat image_copy = image.clone(); // Make a deep copy of the image
        log_queue_.emplace([full_file_path, image_copy]() {
            cv::imwrite(full_file_path, image_copy);
        });
        queue_cv_.notify_one();
    }

    void LogDataBlock(const SyncedDataBlock &synced_data) {
        if (save_data_nonstationary_) {
            auto p_head2base = synced_data.pose_data.data;
            auto p_now2previous = p_previous_head2base_.inverse() * p_head2base;
            // compute angle
            auto r_now2previous = p_now2previous.getRotationMatrix();
            double trace = cv::trace(r_now2previous)[0];
            auto input = std::min(1.0, std::max(-1.0, (trace - 1) / 2));
            double angle = acos(input) * 180 / M_PI;
            // compute distance
            auto translation_diff = p_now2previous.getTranslationVec();
            auto distance = cv::norm(translation_diff, cv::NORM_L2);

            if (!(angle > 1.5 || distance > 0.05)) {
                return;
            }
            p_previous_head2base_ = p_head2base;
        }

        std::string timestamp = std::to_string(synced_data.color_data.timestamp);
        if (!synced_data.color_data.data.empty()) {
            LogImage(synced_data.color_data.data, "color_" + timestamp + ".jpg");
        }
        if (!synced_data.depth_data.data.empty()) {
            LogImage(synced_data.depth_data.data, "depth_" + timestamp + ".png");
        }
        if (synced_data.pose_data.data != Pose()) {
            YAML::Node pose_node(synced_data.pose_data.data);
            LogYAML(pose_node, "pose_" + timestamp + ".yaml");
        }
    }

    void ChangeLogPath(const std::string& log_root) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() { return log_queue_.empty(); });
        log_root_ = log_root;
        if (!std::filesystem::exists(log_root_)) {
            std::filesystem::create_directories(log_root_);
            std::cout << "creating new log directory: " << log_root_ << std::endl;
        } else {
            std::cout << "changing log directory to: " << log_root_ << std::endl;
        }
    }

    std::string get_log_path() {
        return log_root_;
    }

private:
    std::string log_root_;
    std::thread logging_thread_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<std::function<void()>> log_queue_;
    bool stop_logging_;

    bool save_data_nonstationary_ = false;
    Pose p_previous_head2base_;

    void ProcessQueue() {
        while (true) {
            std::function<void()> log_task;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this]() { return !log_queue_.empty() || stop_logging_; });
                if (stop_logging_ && log_queue_.empty()) {
                    break;
                }
                log_task = std::move(log_queue_.front());
                log_queue_.pop();
            }
            log_task();
        }
    }
};

} // namespace booster_vision