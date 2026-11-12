#include "franka_opcua_bridge/fake_test_version/fake_robot.hpp"

namespace franka_opcua_bridge
{

bool FakeRobot::connect() {connected_ = true; return true;}
bool FakeRobot::disconnect() {connected_ = false; return connected_;}
bool FakeRobot::isConnected()const {return connected_;}
bool FakeRobot::requestControl(bool force) {return connected_;} //Da ricontrollare questa serie di controlli
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
  if (!brakes_open_) {return false;}
  last_task_ = task_id;
  return true;
}

MotionResult FakeRobot::moveToNamedPose(
  const std::string & pose_id,
  const std::function<bool()> & should_abort)
{
  if (should_abort && should_abort()) {
    return {MotionOutcome::Cancelled, "Motion cancelled by request"};
  }
  if (!brakes_open_) {
    return {MotionOutcome::Failed, "Brakes are closed"};
  }
  last_pose_ = pose_id;
  last_task_ = "opcua_goto";
  return {MotionOutcome::Completed, "Pose reached"};
}

std::unique_ptr<RobotStatus> FakeRobot::readStatus()
{
  auto status = std::make_unique<RobotStatus>();
  status->is_running = !last_task_.empty();
  status->active_task_name = last_task_;
  return status;
}

std::vector<double> FakeRobot::readJointAngles()
{
  return {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
}

std::vector<double> FakeRobot::readTorque()
{

  return {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
}

geometry_msgs::msg::Wrench FakeRobot::readWrench()
{

  geometry_msgs::msg::Wrench wrench;

  wrench.force.x = 0.0;
  wrench.force.y = 0.785;
  wrench.force.z = 1.0;

  wrench.torque.x = 0.0;
  wrench.torque.y = 1.0;
  wrench.torque.z = 0.0;

  return wrench;

}

geometry_msgs::msg::Pose FakeRobot::readCartesianPose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;
  return pose;
}

bool FakeRobot::activateFCI()
{
  return connected_;
}

bool FakeRobot::deactivateFCI()
{
  return connected_;
}

std::optional<namespaceTree> FakeRobot::showAddressSpaceNodes()
{
  if (!connected_) {
    return std::nullopt;
  }

  namespaceTree root;
  root.browseName = "RootFolder";
  root.displayName = "Root";
  root.nodeId = "ns=0;i=84";
  root.nodekind = NodeKind::Object;
  return root;
}

}  // namespace franka_opcua_bridge
