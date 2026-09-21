#ifndef FRANKA_OPCUA_BRIDGE__FRANKA_ROBOT_HPP_
#define FRANKA_OPCUA_BRIDGE__FRANKA_ROBOT_HPP_

#include <memory>
#include <string>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/wrench.hpp>

#include "franka_opcua_bridge/i_franka.hpp"
#include "franka_opcua_bridge/i_opcua_protocol_client.hpp"
#include "franka_opcua_bridge/opcua_value.hpp"

namespace franka_opcua_bridge
{

struct FrankaRobotStatus : public RobotStatus
{

  std::string active_task_id;

};

class FrankaRobot : public IFranka
{

public:
  explicit FrankaRobot(std::unique_ptr<IOpcUaProtocolClient> client);

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
  bool moveToNamedPose(const std::string & pose_id) override;

  std::unique_ptr<RobotStatus> readStatus() override;
  std::vector<double> readJointAngles() override;
  geometry_msgs::msg::Pose readCartesianPose() override;

  std::vector<double> readTorque() override;
  geometry_msgs::msg::Wrench readWrench() override;

  template<typename T>
  T extractField(const Value::Struct & fields, const std::string & key, const T & default_value);

protected:
  std::unique_ptr<IOpcUaProtocolClient> client_;
  bool force_ = false;

private:
  static const std::vector<std::string> kExecutionControlPath;

};


}

#endif
