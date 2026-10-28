#ifndef BUFFER_H
#define BUFFER_H

#include <vector>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

template<typename T>
class Buffer {
public:
    // 构造函数,设置最大长度
    Buffer(size_t max_size) : max_size_(max_size) {}

    // 添加新数据,使用当前时间戳
    void add(const T& data) {
        add(data, rclcpp::Clock().now());
    }

    // 添加新数据,使用指定时间戳
    void add(const T& data, const rclcpp::Time& timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);
        // 如果达到最大长度,删除最早的数据
        if(data_.size() >= max_size_) {
            data_.erase(data_.begin());
            timestamps_.erase(timestamps_.begin());
        }
        
        data_.push_back(data);
        timestamps_.push_back(timestamp);
    }

    // 获取指定索引的数据
    T get(size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if(index >= data_.size()) {
            throw std::out_of_range("索引超出范围");
        }
        return data_[index];
    }
    
    // 获取与指定时间戳最近的数据和时间戳
    bool get_nearest(const rclcpp::Time& target_time, T& data_out, rclcpp::Time& time_out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if(data_.empty()) {
            return false;
        }

        size_t nearest_idx = 0;
        auto min_diff = std::abs((timestamps_[0] - target_time).nanoseconds());

        // 遍历所有时间戳找到最近的
        for(size_t i = 1; i < timestamps_.size(); ++i) {
            auto diff = std::abs((timestamps_[i] - target_time).nanoseconds());
            if(diff < min_diff) {
                min_diff = diff;
                nearest_idx = i;
            }
        }

        data_out = data_[nearest_idx];
        time_out = timestamps_[nearest_idx];
        return true;
    }

    // 获取指定索引的时间戳
    rclcpp::Time get_timestamp(size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if(index >= timestamps_.size()) {
            throw std::out_of_range("索引超出范围");
        }
        return timestamps_[index];
    }

    // 获取当前缓存的数据量
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.size();
    }

    // 清空缓存
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.clear();
        timestamps_.clear();
    }

private:
    std::vector<T> data_;                    // 数据存储
    std::vector<rclcpp::Time> timestamps_;   // 对应的时间戳
    const size_t max_size_;                  // 最大长度
    mutable std::mutex mutex_;               // 线程安全锁
};

#endif // BUFFER_H
