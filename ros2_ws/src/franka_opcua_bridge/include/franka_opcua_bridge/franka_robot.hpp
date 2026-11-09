#ifndef FRANKA_OPCUA_BRIDGE__FRANKA_ROBOT_HPP_
#define FRANKA_OPCUA_BRIDGE__FRANKA_ROBOT_HPP_

#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <optional>
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/wrench.hpp>

#include "franka_opcua_bridge/interfaces/i_franka.hpp"
#include "franka_opcua_bridge/interfaces/i_opcua_protocol_client.hpp"
#include "franka_opcua_bridge/interfaces/i_pose_storage.hpp"
#include "franka_opcua_bridge/interfaces/i_task_control.hpp"
#include "franka_opcua_bridge/opcua_value.hpp"

namespace franka_opcua_bridge
{

struct FrankaRobotStatus : public RobotStatus
{

  std::string active_task_id;

};

class FrankaRobot : public IFranka, public IPoseStorage, public ITaskControl
{

public:
  explicit FrankaRobot(
    std::unique_ptr<IOpcUaProtocolClient> client,
    std::chrono::milliseconds execution_timeout = std::chrono::seconds(60));

  bool connect() override;
  bool disconnect() override;
  bool isConnected() const override;

  bool requestControl(bool force) override;
  bool releaseControl() override;

  bool openBrakes() override;
  bool closeBrakes() override;
  bool areBrakesOpen() override;

  bool stop() override;


  bool executeNamedTask(const std::string & task_id) override;
  bool moveToNamedPose(
    const std::string & pose_id,
    const std::function<bool()> & should_abort = {}) override;

  std::unique_ptr<RobotStatus> readStatus() override;
  std::vector<double> readJointAngles() override;
  geometry_msgs::msg::Pose readCartesianPose() override;

  std::vector<double> readTorque() override;
  geometry_msgs::msg::Wrench readWrench() override;

  bool activateFCI() override;
  bool deactivateFCI() override;

  bool savePose(const std::string & pose_id) override;

  bool savePose(
    const std::string & pose_id,
    const geometry_msgs::msg::Pose & pose) override;

  bool switchToExecution() override;

  std::vector<std::string> showAvailableTasks() override;

  /*
  template<typename T>
  T extractField(const Value::Struct & fields, const std::string & key, const T & default_value);
  */

protected:
  std::unique_ptr<IOpcUaProtocolClient> client_;
  std::chrono::milliseconds execution_timeout_;
  bool force_ = false;

private:
  static const std::vector<std::string> kExecutionControlPath;

  static constexpr double kTolerance = 1e-3;

  //Conversione in lettura, necessaria nel metodo savePose

  std::vector<double> fromPoseToVector(const geometry_msgs::msg::Pose & pose);

  bool buildKeyPosePair(const std::string & pose_id, const std::vector<double> & value);

  bool setIntegerVariable(const std::string & pose_id, const int32_t value);

  std::optional<geometry_msgs::msg::Pose> tryReadCartesianPose();


};


}

#endif
