#include "game_controller_node.h"

using namespace std;


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // 创建 Node，init, spin
    auto node = make_shared<GameControllerNode>("game_controller_node");

    // 初始化后，进入 spin
    node->init();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}