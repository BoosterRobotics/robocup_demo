#include "booster_vision/vision_node.h"

#include <vector>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: vision_node <config_template_file> [config_override_file ...]" << std::endl;
        return -1;
    }
    rclcpp::init(argc, argv);

    std::string node_name = "vision_node";
    auto node = std::make_shared<booster_vision::VisionNode>(node_name);

    std::string config_template_path = argv[1];
    // The first file is the complete runtime template (vision.yaml).
    // Remaining files are optional overrides, in order. This lets a generated
    // calibration result stay minimal while all runtime defaults come from the
    // stable template.
    std::vector<std::string> config_paths;
    for (int i = 2; i < argc; ++i) {
        // ros2 launch appends its own arguments after this marker. They must
        // remain available to rclcpp::init, but are not YAML override files.
        if (std::string(argv[i]) == "--ros-args") {
            break;
        }
        config_paths.emplace_back(argv[i]);
    }
    if (!node->Init(config_template_path, config_paths)) {
        RCLCPP_FATAL(node->get_logger(), "vision initialization failed");
        rclcpp::shutdown();
        return -2;
    }

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    return 0;
}
