#ifndef FRANKA_OPCUA_BRIDGE__I_ROBOT_HPP_
#define FRANKA_OPCUA_BRIDGE__I_ROBOT_HPP_

#include <string>
#include <vector>
#include <memory>
#include "geometry_msgs/msg/pose.hpp"

namespace franka_opcua_bridge
{

struct RobotStatus
{
  virtual ~RobotStatus() = default;
  bool has_error = false;
  bool is_running = false;
  std::string error_message;
  std::string active_task_name;
};

class IRobot
{
public:
  virtual ~IRobot() = default;

  virtual bool connect() = 0;
  virtual void disconnect() = 0;

  virtual bool requestControl() = 0;
  virtual bool releaseControl() = 0;

  virtual bool openBrakes() = 0;
  virtual bool closeBrakes() = 0;
  virtual bool stop() = 0;

  virtual bool executeNamedTask(const std::string & task_id) = 0;
  virtual bool moveToNamedPose(const std::string & pose_id) = 0;

  virtual std::unique_ptr<RobotStatus> readStatus() = 0;
  virtual std::vector<double> readJointAngles() = 0;
  virtual geometry_msgs::msg::Pose readCartesianPose() = 0;

protected:
  IRobot() = default;
};

}  // namespace franka_opcua_bridge

#endif  // FRANKA_OPCUA_BRIDGE__I_ROBOT_HPP_
