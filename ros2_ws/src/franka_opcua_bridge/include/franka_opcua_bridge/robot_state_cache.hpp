#ifndef FRANKA_OPCUA_BRIDGE__ROBOT_STATE_CACHE_HPP_
#define FRANKA_OPCUA_BRIDGE__ROBOT_STATE_CACHE_HPP_

#include <mutex>
#include <vector>
#include "franka_opcua_bridge/i_robot.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace franka_opcua_bridge
{

// Contenitore thread-safe per l'ultimo stato letto dal robot.
// Un thread di polling scrive (update), il thread ROS legge (get*).
// Nessun chiamante deve mai tenere il lock oltre la singola chiamata.
class RobotStateCache
{
public:
  void updateStatus(const RobotStatus & status)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = status;
  }

  void updateJointAngles(const std::vector<double> & joints)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_angles_ = joints;
  }

  void updateCartesianPose(const geometry_msgs::msg::Pose & pose)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cartesian_pose_ = pose;
  }

  RobotStatus getStatus() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;   // copia, il caller non tiene mai il lock
  }

  std::vector<double> getJointAngles() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return joint_angles_;
  }

  geometry_msgs::msg::Pose getCartesianPose() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cartesian_pose_;
  }

private:
  mutable std::mutex mutex_;
  RobotStatus status_;
  std::vector<double> joint_angles_;
  geometry_msgs::msg::Pose cartesian_pose_;
};

}

#endif
