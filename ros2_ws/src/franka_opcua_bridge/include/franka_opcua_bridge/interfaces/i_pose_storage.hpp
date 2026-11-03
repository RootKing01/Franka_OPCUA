#ifndef FRANKA_OPCUA_BRIDGE__INTERFACES__I_POSE_STORAGE_HPP_
#define FRANKA_OPCUA_BRIDGE__INTERFACES__I_POSE_STORAGE_HPP_

#include <string>

#include "geometry_msgs/msg/pose.hpp"

namespace franka_opcua_bridge
{

class IPoseStorage
{
public:
  virtual ~IPoseStorage() = default;

  virtual bool savePose(const std::string & pose_id) = 0;
  virtual bool savePose(
    const std::string & pose_id,
    const geometry_msgs::msg::Pose & pose) = 0;
};

}  // namespace franka_opcua_bridge

#endif
