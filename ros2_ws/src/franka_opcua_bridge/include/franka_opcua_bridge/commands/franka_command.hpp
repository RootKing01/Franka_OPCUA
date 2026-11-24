#ifndef FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_HPP_
#define FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"

namespace franka_opcua_bridge
{


enum class FrankaCommandType
{

  OpenBrakes,
  CloseBrakes,
  Stop,
  ExecuteNamedTask,
  MoveToNamedPose,
  SaveCurrentPose,
  SaveExplicitPose,
  ActivateFci,
  DeactivateFci,
  ShowTasks,
  ShowAddressSpaceNodes,
  SwitchToExec,
  ExecuteComplexTask

};

struct FrankaRobotCommand
{

  std::uint64_t command_id;
  FrankaCommandType type;
  std::string task_name;
  std::string parameter;
  std::vector<std::string> task_poses;
  std::optional<geometry_msgs::msg::Pose> pose;
  std::chrono::steady_clock::time_point created_at;
  std::chrono::steady_clock::time_point received_at{};

};


}

#endif
