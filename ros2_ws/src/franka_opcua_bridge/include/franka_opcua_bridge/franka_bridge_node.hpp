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

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;

  rclcpp::TimerBase::SharedPtr result_timer_;

  std::unique_ptr<OpcuaWorker> worker_;

  std::atomic<std::uint64_t> next_command_id_{1};

  void commandCallback(
    const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace franka_opcua_bridge

#endif
