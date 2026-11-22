#ifndef FRANKA_OPCUA_BRIDGE__INTERFACES__TASK_STATE_HPP_
#define FRANKA_OPCUA_BRIDGE__INTERFACES__TASK_STATE_HPP_

#include <string>

namespace franka_opcua_bridge
{

enum class TaskState
{
  Unknown,
  Idle,
  Starting,
  Running,
  Stopping,
  Completed,
  Failed,
  Cancelled,
  Timeout
};

struct TaskExecutionResult
{
  TaskState state;
  std::string message;
};

}  // namespace franka_opcua_bridge

#endif
