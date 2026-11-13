# include "franka_fsm/franka_fsm_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<franka_fsm::FrankaFsmNode>());
    rclcpp::shutdown();

    return 0;
}