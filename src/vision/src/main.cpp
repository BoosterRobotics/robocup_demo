#include "booster_vision/vision_node.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: vision_node <config_template_file> <config_file>" << std::endl;
        return -1;
    }
    rclcpp::init(argc, argv);

    std::string node_name = "vision_node";
    auto node = std::make_shared<booster_vision::VisionNode>(node_name);

    std::string config_template_path = argv[1];
    std::string config_path = "";
    if (argc > 2) {
        config_path = argv[2];
    }
    node->Init(config_template_path, config_path);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    return 0;
}
