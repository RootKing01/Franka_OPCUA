#ifndef FRANKA_OPCUA_BRIDGE__FRANKA_BRIDGE_NODE_HPP_
#define FRANKA_OPCUA_BRIDGE__FRANKA_BRIDGE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "franka_interfaces/msg/bridge_command.hpp"
#include "franka_interfaces/msg/bridge_result.hpp"
#include "franka_opcua_bridge/performance_monitor.hpp"

#include <chrono>
#include <memory>
#include <cstdint>
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
  rclcpp::Subscription<franka_interfaces::msg::BridgeCommand>::SharedPtr command_subscription_;

  rclcpp::Publisher<franka_interfaces::msg::BridgeResult>::SharedPtr result_publisher_;

  rclcpp::TimerBase::SharedPtr result_timer_;

  std::shared_ptr<PerformanceMonitor> performance_monitor_;
  std::chrono::steady_clock::time_point last_performance_report_{};
  std::unique_ptr<OpcuaWorker> worker_;

  void publishPendingResults();
  void publishRejected(
    std::uint64_t request_id,
    const std::string & reason,
    std::chrono::steady_clock::time_point received_at);

  void commandCallback(
    const franka_interfaces::msg::BridgeCommand::SharedPtr msg);
};

}  // namespace franka_opcua_bridge

#endif
