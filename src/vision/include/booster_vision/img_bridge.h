#pragma once

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace booster_vision {

cv::Mat toCVMat(const sensor_msgs::msg::Image &source);

// sensor_msgs::msg::Image::SharedPtr

} // namespace booster_vision