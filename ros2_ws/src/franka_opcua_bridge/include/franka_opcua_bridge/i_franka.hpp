#ifndef FRANKA_OPCUA_BRIDGE__I_FRANKA_
#define FRANKA_OPCUA_BRIDGE__I_FRANKA_

#include "franka_opcua_bridge/i_robot.hpp"

namespace franka_opcua_bridge
{

class IFranka : public IRobot
{

public:
  virtual bool requestControl(bool force) = 0;
  virtual bool releaseControl() = 0;

  virtual bool executeNamedTask(const std::string & task_id) = 0;
  virtual bool moveToNamedPose(const std::string & pose_id) = 0;

  virtual bool stop() = 0;

  //virtual bool activateFCI() = 0;
  //virtual bool deactivateFCI() = 0;


};


}

#endif
