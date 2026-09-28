#ifndef FRANKA_OPCUA_BRIDGE__FRANKA_BRIDGE_NODE_HPP_
#define FRANKA_OPCUA_BRIDGE__FRANKA_BRIDGE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <string>

#include "franka_opcua_bridge/franka_robot.hpp"

namespace franka_opcua_bridge
{

class FrankaBridgeNode : public rclcpp::Node
{
public:
    explicit FrankaBridgeNode(const std::string & robot_id);

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;

    std::unique_ptr<FrankaRobot> franka_;

    void commandCallback(
        const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace franka_opcua_bridge

#endif