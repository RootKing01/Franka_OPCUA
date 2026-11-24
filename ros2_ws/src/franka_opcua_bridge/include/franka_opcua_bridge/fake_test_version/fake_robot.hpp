#ifndef FRANKA_OPCUA_BRIDGE__FAKE_ROBOT_HPP_
#define FRANKA_OPCUA_BRIDGE__FAKE_ROBOT_HPP_

#include "franka_opcua_bridge/interfaces/i_franka.hpp"

#include <functional>
#include <unordered_map>

namespace franka_opcua_bridge
{

class FakeRobot : public IFranka
{
public:
  bool connect() override;
  bool disconnect() override;
  bool isConnected() const override;
  bool requestControl(bool force) override;
  bool releaseControl() override;
  bool openBrakes() override;
  bool closeBrakes() override;
  bool areBrakesOpen() {return brakes_open_;}
  bool stop() override;
  TaskExecutionResult executeNamedTask(
    const std::string & task_id,
    const std::function<bool()> & should_abort = {}) override;
  TaskExecutionResult startNamedTask(const std::string & task_id) override;
  MotionResult moveToNamedPose(
    const std::string & taskname,
    const std::string & pose_id,
    const std::vector<std::string> & task_poses,
    const std::function<bool()> & should_abort = {}) override;

  bool activateFCI() override;
  bool deactivateFCI() override;
  std::optional<namespaceTree> showAddressSpaceNodes() override;

  std::unique_ptr<RobotStatus> readStatus() override;
  std::vector<double> readJointAngles() override;
  geometry_msgs::msg::Pose readCartesianPose() override;

  // Metodi di ispezione per i test: non fanno parte di IRobot,
  // servono solo a verificare cosa e' successo dentro FakeRobot
  std::string lastTaskExecuted() const {return last_task_;}
  std::string lastPoseTarget() const {return last_pose_;}

  std::vector<double> readTorque() override;
  geometry_msgs::msg::Wrench readWrench() override;
  bool readIntegerVariable(const std::string & variable_id, int32_t & out_value) override;
  bool setIntegerVariable(const std::string & variable_id, int32_t value) override;

private:
  bool connected_ = false;
  bool brakes_open_ = false;
  std::string last_task_;
  std::string last_pose_;
  std::unordered_map<std::string, int32_t> integer_variables_;
};

}  // namespace franka_opcua_bridge

#endif  // FRANKA_OPCUA_BRIDGE__FAKE_ROBOT_HPP_
