#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

#include "brain.h"

#define HZ 100

using namespace std;

int main(int argc, char **argv)
{

    // 初始化 ros2
    rclcpp::init(argc, argv);

    // Brain 对象
    std::shared_ptr<Brain> brain = std::make_shared<Brain>();

    // 执行初始化操作：读取参数，构建 BehaviorTree 等
    brain->init();

    // 单独开一个线程执行 brain.tick
    thread t([&brain]() {
        while (rclcpp::ok()) {
            auto start_time = brain->get_clock()->now();
            brain->tick();
            auto end_time = brain->get_clock()->now();
            auto duration = (end_time - start_time).nanoseconds() / 1000000.0; // 转换为毫秒
            brain->log->setTimeNow();
            brain->log->log("performance/brain_tick", rerun::Scalar(duration));
            this_thread::sleep_for(chrono::milliseconds(static_cast<int>(1000 / HZ)));
        } 
    });

    // 开个独立的线程，处理 joystick 和 gamecontroller 的回调
    thread t1([&brain, &argc, &argv]() {
        // 用独立的 context
        auto context = rclcpp::Context::make_shared();
        context->init(argc, argv);
        rclcpp::NodeOptions opt;
        opt.context(context);
        auto node = rclcpp::Node::make_shared("brain_node_ext", opt);
        auto sub1 = node->create_subscription<booster_interface::msg::RemoteControllerState>("/remote_controller_state", 10, bind(&Brain::joystickCallback, brain, std::placeholders::_1));
        auto sub2 = node->create_subscription<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 10, bind(&Brain::gameControlCallback, brain, std::placeholders::_1));
    
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin(); 
    });

    // 使用单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(brain);
    executor.spin();

    t.join();
    t1.join();
    rclcpp::shutdown();
    return 0;
}
