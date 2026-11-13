#ifndef FRANKA_OPCUA_BRIDGE__INTERFACES__I_TASK_CONTROL_HPP_
#define FRANKA_OPCUA_BRIDGE__INTERFACES__I_TASK_CONTROL_HPP_

#include <string>
#include <vector>

namespace franka_opcua_bridge
{

class ITaskControl
{
public:
  virtual ~ITaskControl() = default;

  virtual bool switchToExecution() = 0;
  virtual std::vector<std::string> showAvailableTasks() = 0;
};

}  // namespace franka_opcua_bridge

#endif
