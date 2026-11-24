#ifndef FRANKA_FSM__FRANKA_FSM_NODE_HPP_
#define FRANKA_FSM__FRANKA_FSM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "franka_interfaces/msg/bridge_command.hpp"
#include "franka_interfaces/msg/bridge_result.hpp"
#include "franka_interfaces/msg/decision_request.hpp"
#include "franka_interfaces/msg/decision_response.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>


namespace franka_fsm
{

class FrankaFsmNode : public rclcpp::Node
{

public:
  FrankaFsmNode();
  void runInteractiveMenu();

  ~FrankaFsmNode() override = default;

private:
  rclcpp::Publisher<franka_interfaces::msg::BridgeCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<franka_interfaces::msg::BridgeResult>::SharedPtr result_subscription_;

  rclcpp::Publisher<franka_interfaces::msg::DecisionResponse>::SharedPtr decision_response_;
  rclcpp::Subscription<franka_interfaces::msg::DecisionRequest>::SharedPtr decision_request_;
  std::queue<franka_interfaces::msg::DecisionRequest> pending_decisions_;
  std::condition_variable state_condition_;
  std::uint64_t next_request_id_{1};
  std::vector<std::string> available_task_ids_;
  std::mutex state_mutex_;
  bool task_active_{false};

  enum class CommandType
  {
    ShowTasks,
    ExecuteTask,
    Stop,
    OpenBrakes,
    CloseBrakes,
    ActivateFci,
    DeactivateFci,
    SwitchToExec,
    ShowAddressSpaceNodes,
    MoveToNamedPose,
    SaveCurrentPose,
    SaveExplicitPose,
    ExecuteComplexTask
  };

  struct CommandRequest
  {
    CommandType command;
    std::string task_name;
    std::string parameter;
    std::vector<std::string> single_operation_pose;
    std::optional<geometry_msgs::msg::Pose> pose;
  };

  std::unordered_map<std::uint64_t, CommandType> pending_commands_;

  enum class FsmState
  {
    Idle,
    WaitingForResult,
    Picking,
    BlockReady,
    Moving,
    TaskRunning,
    Completed,
    Interrupted,
    Error
  };

  enum class BridgeStatus
  {
    Unknown,
    Accepted,
    Completed,
    Rejected,
    Failed,
    Cancelled,
    NotNeeded,
    Timeout,
    NotReady,
    Disconnected
  };

  FsmState state_{FsmState::Idle};

  std::optional<std::uint64_t> sendCommand(
    CommandType command,
    const std::string & task_name = "",
    const std::string & parameter = "",
    const std::vector<std::string> & operationPose = {},
    const std::optional<geometry_msgs::msg::Pose> & pose = std::nullopt);

  std::string operationsPosePrint(const std::vector<std::string> & operationPose);
  void showMainMenu(bool task_active) const;
  void showAvailableCommands() const;
  void showCommandMenu(bool task_active) const;
  std::optional<CommandRequest> readCommandRequest(bool task_active);
  bool checkFullQueue();
  bool answerPendingDecision();
  void waitForComplexTask(std::uint64_t request_id);


};

}  // namespace franka_fsm


#endif
