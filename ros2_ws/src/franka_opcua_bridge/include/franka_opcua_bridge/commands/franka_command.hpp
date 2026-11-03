#ifndef FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_HPP_
#define FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace franka_opcua_bridge
{


enum class FrankaCommandType
{

  OpenBrakes,
  CloseBrakes,
  Stop,
  ExecuteNamedTask,
  MoveToNamedPose,
  ActivateFci,
  DeactivateFci

};

struct FrankaRobotCommand
{

  std::uint64_t command_id;
  FrankaCommandType type;
  std::string parameter;
  std::chrono::steady_clock::time_point created_at;

};


}

#endif
