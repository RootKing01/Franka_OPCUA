#include "franka_opcua_bridge/fake_robot.hpp"

namespace franka_opcua_bridge
{

bool FakeRobot::connect() {connected_ = true; return true;}
void FakeRobot::disconnect() {connected_ = false;}
bool FakeRobot::requestControl() {return connected_;}
bool FakeRobot::releaseControl() {return true;}

bool FakeRobot::openBrakes()
{
  if (!connected_) {return false;}
  brakes_open_ = true;
  return true;
}

bool FakeRobot::closeBrakes()
{
  brakes_open_ = false;
  return true;
}

bool FakeRobot::stop() {last_task_ = ""; return true;}

bool FakeRobot::executeNamedTask(const std::string & task_id)
{
  if (!brakes_open_) {return false;}  // logica plausibile: serve avere i freni aperti
  last_task_ = task_id;
  return true;
}

bool FakeRobot::moveToNamedPose(const std::string & pose_id)
{
  if (!brakes_open_) {return false;}
  last_pose_ = pose_id;
  last_task_ = "opcua_goto";
  return true;
}

RobotStatus FakeRobot::readStatus()
{
  RobotStatus status;
  status.is_running = !last_task_.empty();
  status.active_task_name = last_task_;
  return status;
}

std::vector<double> FakeRobot::readJointAngles()
{
  return {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
}

geometry_msgs::msg::Pose FakeRobot::readCartesianPose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;
  return pose;
}

}  // namespace franka_opcua_bridge