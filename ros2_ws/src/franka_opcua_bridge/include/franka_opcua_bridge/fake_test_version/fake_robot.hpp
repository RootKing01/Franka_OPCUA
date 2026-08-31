#ifndef FRANKA_OPCUA_BRIDGE__FAKE_ROBOT_HPP_
#define FRANKA_OPCUA_BRIDGE__FAKE_ROBOT_HPP_

#include "franka_opcua_bridge/i_robot.hpp"

namespace franka_opcua_bridge
{

class FakeRobot : public IRobot
{
public:
  bool connect() override;
  void disconnect() override;
  bool requestControl() override;
  bool releaseControl() override;
  bool openBrakes() override;
  bool closeBrakes() override;
  bool stop() override;
  bool executeNamedTask(const std::string & task_id) override;
  bool moveToNamedPose(const std::string & pose_id) override;
  std::unique_ptr<RobotStatus> readStatus() override;
  std::vector<double> readJointAngles() override;
  geometry_msgs::msg::Pose readCartesianPose() override;

  // Metodi di ispezione per i test: non fanno parte di IRobot,
  // servono solo a verificare cosa e' successo dentro FakeRobot
  bool isConnected() const {return connected_;}
  bool areBrakesOpen() const {return brakes_open_;}
  std::string lastTaskExecuted() const {return last_task_;}
  std::string lastPoseTarget() const {return last_pose_;}

private:
  bool connected_ = false;
  bool brakes_open_ = false;
  std::string last_task_;
  std::string last_pose_;
};

}  // namespace franka_opcua_bridge

#endif  // FRANKA_OPCUA_BRIDGE__FAKE_ROBOT_HPP_
