#ifndef FRANKA_OPCUA_BRIDGE__INTERFACES__I_FRANKA_
#define FRANKA_OPCUA_BRIDGE__INTERFACES__I_FRANKA_

#include "franka_opcua_bridge/interfaces/i_robot.hpp"

#include <functional>
#include <string>

namespace franka_opcua_bridge
{

enum class MotionOutcome
{
  Completed,
  Cancelled,
  Failed,
  Timeout
};

struct MotionResult
{
  MotionOutcome outcome;
  std::string message;
};

class IFranka : public IRobot
{

public:
  virtual bool requestControl(bool force) = 0;
  virtual bool releaseControl() = 0;

  virtual bool executeNamedTask(const std::string & task_id) = 0;
  virtual MotionResult moveToNamedPose(
    const std::string & pose_id,
    const std::function<bool()> & should_abort = {}) = 0;

  virtual bool stop() = 0;

  virtual bool activateFCI() = 0;
  virtual bool deactivateFCI() = 0;


};


}

#endif
