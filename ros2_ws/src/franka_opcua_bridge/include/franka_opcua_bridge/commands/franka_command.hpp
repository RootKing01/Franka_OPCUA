#ifndef FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_HPP_
#define FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

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
  DeactivateFci

};

struct FrankaRobotCommand
{

  std::uint64_t command_id;
  FrankaCommandType type;
  std::string parameter;
  std::optional<geometry_msgs::msg::Pose> pose;
  std::chrono::steady_clock::time_point created_at;

};


}

#endif
