#ifndef SIM_DETECTION_BRIDGE_H
#define SIM_DETECTION_BRIDGE_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_interface/msg/detections.hpp>

class SimDetectionBridge : public rclcpp::Node
{
public:
    SimDetectionBridge();

private:
    void markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    rclcpp::Publisher<vision_interface::msg::Detections>::SharedPtr publisher_;
};

#endif // SIM_DETECTION_BRIDGE_H
