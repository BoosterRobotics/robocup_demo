#include "booster_vision/vision_node.h"

#include <chrono>
#include <future>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>

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
    // Remaining files override only camera/calibration, in order. All other
    // YAML sections continue to come from the complete runtime template.
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

    if (!node->VisualizationEnabled()) {
        executor.spin();
        rclcpp::shutdown();
        return 0;
    }

    // A mutually exclusive ROS callback group does not pin callbacks to one
    // thread. Keep every HighGUI call on the main thread; inference and sensor
    // synchronization continue on the executor threads.
    auto spin_future = std::async(std::launch::async, [&executor]() {
        executor.spin();
    });
    int exit_code = 0;
    bool windows_created = false;
    RCLCPP_INFO(node->get_logger(), "[vision][display] HighGUI runs on the main thread");
    try {
        while (rclcpp::ok() &&
               spin_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            auto frames = node->TakeVisualizationFrames();
            if (!frames.first.empty()) {
                windows_created = true;
                cv::imshow("Detection", frames.first);
            }
            if (!frames.second.empty()) {
                windows_created = true;
                cv::imshow("Segmentation", frames.second);
            }
            // Pump events even when inference is slow or no new frame arrives.
            if (windows_created) {
                cv::waitKey(1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Visualization failed: %s", e.what());
        exit_code = -3;
    }

    // Shutdown also handles the case where the executor has not entered spin
    // yet. Join it before destroying node-owned buffers or GUI windows.
    rclcpp::shutdown();
    executor.cancel();
    try {
        spin_future.get();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Vision executor failed: %s", e.what());
        exit_code = -3;
    }
    if (windows_created) {
        cv::destroyAllWindows();
    }

    return exit_code;
}
