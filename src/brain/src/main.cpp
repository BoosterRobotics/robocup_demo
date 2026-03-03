#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

#include "brain.h"

#define HZ 100

using namespace std;

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    std::shared_ptr<Brain> brain = std::make_shared<Brain>();

    // initialize operations: read parameters, construct BehaviorTree, etc.
    brain->init();

    // Start a separate thread to run the tick function of Brain at a fixed frequency (HZ). The main thread will handle ROS2 callbacks.
    thread t([&brain]() {
        while (rclcpp::ok()) {
            auto start_time = brain->get_clock()->now();
            brain->tick();
            auto end_time = brain->get_clock()->now();
            auto duration = (end_time - start_time).nanoseconds() / 1000000.0; // Convert to milliseconds
            brain->log->log_scalar("performance", "brain_tick_duration_ms", duration);
            this_thread::sleep_for(chrono::milliseconds(static_cast<int>(1000 / HZ)));
        } 
    });

    // Start a separate thread to handle joystick and gamecontroller callbacks
    thread t1([&brain, &argc, &argv]() {
        // Use a separate context
        auto context = rclcpp::Context::make_shared();
        context->init(argc, argv);
        rclcpp::NodeOptions opt;
        opt.context(context);
        auto node = rclcpp::Node::make_shared("brain_node_ext", opt);
        auto sub1 = node->create_subscription<booster_interface::msg::RemoteControllerState>("/remote_controller_state", 10, bind(&Brain::joystickCallback, brain, std::placeholders::_1));
        auto sub2 = node->create_subscription<game_controller_interface::msg::GameControlData>("/booster_soccer/game_controller", 10, bind(&Brain::gameControlCallback, brain, std::placeholders::_1));
        auto sub3 = node->create_subscription<std_msgs::msg::String>("/booster_agent/soccer_game_control", 1, bind(&Brain::agentCommandCallback, brain, std::placeholders::_1));
    
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin(); 
    });

    // Use a single-threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(brain);
    executor.spin();

    t.join();
    t1.join();
    rclcpp::shutdown();
    return 0;
}
