#include "franka_fsm/franka_fsm_node.hpp"

#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<franka_fsm::FrankaFsmNode>();
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    node->runInteractiveMenu();
    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}