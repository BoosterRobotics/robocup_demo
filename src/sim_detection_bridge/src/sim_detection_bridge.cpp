#include "sim_detection_bridge.h"

SimDetectionBridge::SimDetectionBridge() : rclcpp::Node("sim_detection_bridge")
{
    // 声明参数，从参数服务器读取
    // 参数值来自启动文件传入的配置
    this->declare_parameter<std::string>("robot.robot_name", "");
    auto robot_name = this->get_parameter("robot.robot_name").as_string();
    
    if (robot_name.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'robot.robot_name' is not set.");
        throw std::runtime_error("Parameter 'robot.robot_name' is not set.");
    }
    
    RCLCPP_INFO(this->get_logger(), "Read robot_name: %s", robot_name.c_str());
    // Subscribe to the simulation detection topic
    subscription_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/booster/detections/" + robot_name + "_rgbd_camera",
        1,
        std::bind(&SimDetectionBridge::markerArrayCallback, this, std::placeholders::_1));

    // Publisher for the converted detections
    publisher_ = create_publisher<vision_interface::msg::Detections>(
        "/booster_vision/detection/" + robot_name,
        1);

    RCLCPP_INFO(get_logger(), "SimDetectionBridge initialized");
}

void SimDetectionBridge::markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    // Create output detections message
    auto detections = std::make_shared<vision_interface::msg::Detections>();

    // Process each marker in the array
    for (const auto& marker : msg->markers)
    {
        // Skip markers that are not being used (action == DELETE or DELETEALL)
        if (marker.action == visualization_msgs::msg::Marker::DELETE || 
            marker.action == visualization_msgs::msg::Marker::DELETEALL)
        {
            continue;
        }
        
        // Use the first valid marker's header for the detections message
        if (detections->detected_objects.empty() && marker.header.frame_id != "")
        {
            detections->header = marker.header;
        }

        vision_interface::msg::DetectedObject detected_object;

        // Extract label from marker namespace or text
        detected_object.label = marker.ns.empty() ? marker.text : marker.ns;

        // Extract confidence from marker color alpha (0-1 range)
        detected_object.confidence = 1.0;

        // bbox mapping:
        // R: xmin, G: ymin, B: xmax, A: ymax

        detected_object.xmin = static_cast<int64_t>(marker.color.r);
        detected_object.ymin = static_cast<int64_t>(marker.color.g);
        detected_object.xmax = static_cast<int64_t>(marker.color.b);
        detected_object.ymax = static_cast<int64_t>(marker.color.a);

        // Extract position from marker pose
        std::vector<float> position = {
            static_cast<float>(marker.pose.position.x),
            static_cast<float>(marker.pose.position.y),
            static_cast<float>(marker.pose.position.z)};
        detected_object.position = position;

        // Use position projection as fallback (same as position for now)
        detected_object.position_projection = position;

        detections->detected_objects.push_back(detected_object);
    }

    // Publish the converted detections
    publisher_->publish(*detections);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimDetectionBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
