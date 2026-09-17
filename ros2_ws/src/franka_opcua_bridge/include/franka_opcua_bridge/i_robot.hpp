#ifndef FRANKA_OPCUA_BRIDGE__I_ROBOT_HPP_
#define FRANKA_OPCUA_BRIDGE__I_ROBOT_HPP_

#include <string>
#include <vector>
#include <memory>
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/wrench.hpp>

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
  virtual bool disconnect() = 0;
  virtual bool isConnected() const = 0;


  virtual bool openBrakes() = 0;
  virtual bool closeBrakes() = 0;
  virtual bool areBrakesOpen() = 0;

  virtual std::unique_ptr<RobotStatus> readStatus() = 0;

  virtual std::vector<double> readJointAngles() = 0;
  virtual geometry_msgs::msg::Pose readCartesianPose() = 0;

  virtual std::vector<double> readTorque() = 0;
  virtual geometry_msgs::msg::Wrench readWrench() = 0;

protected:
  IRobot() = default;
};

}  // namespace franka_opcua_bridge

#endif  // FRANKA_OPCUA_BRIDGE__I_ROBOT_HPP_
