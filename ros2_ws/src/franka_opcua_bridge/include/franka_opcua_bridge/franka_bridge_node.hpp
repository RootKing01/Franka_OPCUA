#ifndef FRANKA_OPCUA_BRIDGE__FRANKA_BRIDGE_NODE_HPP_
#define FRANKA_OPCUA_BRIDGE__FRANKA_BRIDGE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <string>

#include "franka_opcua_bridge/opcua_worker.hpp"

namespace franka_opcua_bridge
{

class FrankaBridgeNode : public rclcpp::Node
{
public:
  explicit FrankaBridgeNode(const std::string & robot_id);

  ~FrankaBridgeNode() override;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;

  std::unique_ptr<OpcuaWorker> worker_;

  void commandCallback(
    const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace franka_opcua_bridge

#endif
