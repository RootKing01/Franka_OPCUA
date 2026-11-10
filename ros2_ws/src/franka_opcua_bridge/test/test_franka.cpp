#include <gtest/gtest.h>
#include <cmath>

#include <Eigen/Dense>

#include "franka_opcua_bridge/franka_robot.hpp"
#include "franka_opcua_bridge/fake_test_version/fake_protocol_client.hpp"
#include "franka_opcua_bridge/opcua_value.hpp"

using franka_opcua_bridge::FrankaRobot;
using franka_opcua_bridge::FakeProtocolClient;
using franka_opcua_bridge::Value;
using franka_opcua_bridge::FrankaRobotStatus;
using franka_opcua_bridge::ExecutionStatusValue;
using franka_opcua_bridge::KeyPosePairValue;
using franka_opcua_bridge::MotionOutcome;

namespace
{

void expectQuaternionRotationNear(
  const geometry_msgs::msg::Quaternion & actual,
  const Eigen::Quaterniond & expected)
{
  const Eigen::Quaterniond actual_quaternion(
    actual.w, actual.x, actual.y, actual.z);

  EXPECT_NEAR(
    (actual_quaternion.toRotationMatrix() - expected.toRotationMatrix()).norm(),
    0.0,
    1e-12);
}

}


TEST(FrankaRobotTest, RequestControlCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  bool force = true;

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue({"Robot", "ExecutionControl", "ControlTokenActive"}, Value(true));

  bool result = robot.requestControl(force);
  EXPECT_TRUE(result);

  const auto & log = fake_ptr->callLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0].object_path, (std::vector<std::string>{"Robot", "ExecutionControl"}));
  EXPECT_EQ(log[0].method_name, "RequestControlToken");

}

TEST(FrankaRobotTest, releaseControlCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  bool result = robot.releaseControl();
  EXPECT_TRUE(result);

  const auto & log = fake_ptr->callLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0].object_path, (std::vector<std::string>{"Robot", "ExecutionControl"}));
  EXPECT_EQ(log[0].method_name, "FreeControlToken");


}


TEST(FrankaRobotTest, OpenBrakesCallsCorrectMethod)
{
  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();   // puntatore raw, solo per ispezione

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();

  ASSERT_TRUE(success);

  bool result = robot.openBrakes();

  EXPECT_TRUE(result);

  const auto & log = fake_ptr->callLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0].object_path, (std::vector<std::string>{"Robot", "ExecutionControl"}));
  EXPECT_EQ(log[0].method_name, "OpenBrakes");
}


TEST(FrankaRobotTest, CloseBrakesCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();   //puntatore di ispezione

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();

  ASSERT_TRUE(success);

  bool result = robot.closeBrakes();

  EXPECT_TRUE(result);

  const auto & log = fake_ptr->callLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0].object_path, (std::vector<std::string>{"Robot", "ExecutionControl"}));
  EXPECT_EQ(log[0].method_name, "CloseBrakes");

}


TEST(FrankaRobotTest, AreBrakesOpenReturnsBrakeState) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue({"Robot", "ExecutionControl", "BrakesOpen"}, Value(true));

  bool result = robot.areBrakesOpen();

  EXPECT_TRUE(result);

}


TEST(FrankaRobotTest, AreBrakesOpenReturnsFalse) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue({"Robot", "ExecutionControl", "BrakesOpen"}, Value(false));

  bool result = robot.areBrakesOpen();

  EXPECT_FALSE(result);

}

TEST(FrankaRobotTest, stopCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  bool result = robot.stop();
  EXPECT_TRUE(result);

  const auto & log = fake_ptr->callLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0].object_path, (std::vector<std::string>{"Robot", "ExecutionControl"}));
  EXPECT_EQ(log[0].method_name, "StopTask");
}

TEST(FrankaRobotTest, readJointAnglesCallCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  std::vector<std::string> path_ = {"Robot", "ExecutionControl", "JointAngles"};
  std::vector<double> angles_ = {1.0, 1.0, 2.0, 3.0, 5.0, 8.0, 13.0};
  std::vector<double> output_;

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue(path_, Value(angles_));

  output_ = robot.readJointAngles();
  EXPECT_EQ(output_, angles_);


}

TEST(FrankaRobotTest, ReadJointAnglesReturnsEmptyIfNotSet)
{
  auto fake_client = std::make_unique<FakeProtocolClient>();

  FrankaRobot robot(std::move(fake_client));
  robot.connect();

  std::vector<double> output_ = robot.readJointAngles();

  EXPECT_TRUE(output_.empty());
}

TEST(FrankaRobotTest, ReadCartesianPoseCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  std::vector<std::string> path_ = {"Robot", "ExecutionControl", "CartesianPose"};
  std::vector<double> matrix_ = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  geometry_msgs::msg::Pose pose_;

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue(path_, Value(matrix_));

  pose_ = robot.readCartesianPose();

  EXPECT_EQ(pose_.position.x, 0.0);
  EXPECT_EQ(pose_.position.y, 0.0);
  EXPECT_EQ(pose_.position.z, 0.0);
  EXPECT_EQ(pose_.orientation.x, 0.0);
  EXPECT_EQ(pose_.orientation.y, 0.0);
  EXPECT_EQ(pose_.orientation.z, 0.0);
  EXPECT_EQ(pose_.orientation.w, 1.0);


}

TEST(FrankaRobotTest, ReadStatuCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  std::vector<std::string> path_ = {"Robot", "ExecutionControl", "ExecutionStatus"};

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  const ExecutionStatusValue expected_status{
    false,
    true,
    "",
    "opcua_goto",
    "task_0"};

  bool writeSuccess = fake_ptr->writeValue(path_, Value(expected_status));
  ASSERT_TRUE(writeSuccess);

  auto status = robot.readStatus();

  EXPECT_EQ(status->has_error, expected_status.has_error);
  EXPECT_EQ(status->is_running, expected_status.is_running);
  EXPECT_EQ(status->error_message, expected_status.error_message);
  EXPECT_EQ(status->active_task_name, expected_status.active_task_name);

  auto franka_status = dynamic_cast<FrankaRobotStatus *>(status.get());
  ASSERT_NE(franka_status, nullptr);

  EXPECT_EQ(franka_status->active_task_id, expected_status.active_task_id);

}

TEST(FrankaRobotTest, executeNamedPoseCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  const std::string task_id = "opcua_goto";

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  bool taskStarted = robot.executeNamedTask(task_id);
  EXPECT_TRUE(taskStarted);

  const auto & log = fake_ptr->callLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0].object_path, (std::vector<std::string>{"Robot", "ExecutionControl"}));
  EXPECT_EQ(log[0].method_name, "StartTask");
  ASSERT_EQ(log[0].args.size(), 1u);
  EXPECT_EQ(log[0].args[0].as<std::string>(), "opcua_goto");

}

TEST(FrankaRobotTest, ReadTorqueCall) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  std::vector<std::string> path = {"Robot", "ExecutionControl", "EstimatedTorques"};
  std::vector<double> torque;

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue(
    path, Value(
      std::vector<double>(
        {1.0, 1.0, 2.0, 3.0, 5.0, 8.0, 13.0})) );


  torque = robot.readTorque();
  ASSERT_EQ(torque.size(), 7);


  EXPECT_EQ(torque[0], 1.0);
  EXPECT_EQ(torque[1], 1.0);
  EXPECT_EQ(torque[2], 2.0);
  EXPECT_EQ(torque[3], 3.0);
  EXPECT_EQ(torque[4], 5.0);
  EXPECT_EQ(torque[5], 8.0);
  EXPECT_EQ(torque[6], 13.0);


}

TEST(FrankaRobotTest, ReadWrenchCall) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  std::vector<std::string> path = {"Robot", "ExecutionControl", "EstimatedForces"};
  geometry_msgs::msg::Wrench wrench;

  FrankaRobot robot(std::move(fake_client));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue(
    path, Value(
      std::vector<double>(
        {1.0, 1.0, 2.0, 3.0, 5.0, 8.0})) );


  wrench = robot.readWrench();

  EXPECT_EQ(wrench.force.x, 1.0);
  EXPECT_EQ(wrench.force.y, 1.0);
  EXPECT_EQ(wrench.force.z, 2.0);
  EXPECT_EQ(wrench.torque.x, 3.0);
  EXPECT_EQ(wrench.torque.y, 5.0);
  EXPECT_EQ(wrench.torque.z, 8.0);

}


TEST(FrankaRobotTest, MoveToNamedPoseCall) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  const std::string pose_id = "p0";
  std::vector<std::string> variable_browse_path = {"Robot", "KeyValueMaps", "KeyPoseMap", "Read"};

  variable_browse_path.push_back(pose_id);

  FrankaRobot robot(std::move(fake_client), std::chrono::milliseconds(500));

  bool success = robot.connect();
  ASSERT_TRUE(success);

  const std::vector<double> received_pose{
    0.0, 1.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.4, -0.2, 0.6, 1.0};

  fake_ptr->writeValue(
    variable_browse_path, Value(
      received_pose) );

  fake_ptr->queueValue(
    {"Robot", "ExecutionControl", "ExecutionStatus"},
    Value(ExecutionStatusValue{false, true, "", "opcua_goto", "opcua_goto"}));
  fake_ptr->queueValue(
    {"Robot", "ExecutionControl", "ExecutionStatus"},
    Value(ExecutionStatusValue{false, false, "", "", ""}));

  const auto move_result = robot.moveToNamedPose(pose_id);
  ASSERT_EQ(move_result.outcome, MotionOutcome::Completed);

  const auto & log = fake_ptr->callLog();
  ASSERT_EQ(log.size(), 3u);
  EXPECT_EQ(log[0].method_name, "Read");
  EXPECT_EQ(log[1].method_name, "Replace");
  ASSERT_EQ(log[1].args.size(), 1u);
  ASSERT_TRUE(log[1].args[0].is<KeyPosePairValue>());

  const auto & target_pose = log[1].args[0].as<KeyPosePairValue>();
  EXPECT_EQ(target_pose.key, "target_pose");
  ASSERT_EQ(target_pose.value.size(), 16u);

  for (const double value : target_pose.value) {
    EXPECT_TRUE(std::isfinite(value));
  }

  const Eigen::Map<const Eigen::Matrix4d> matrix(target_pose.value.data());
  const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);

  EXPECT_NEAR(matrix(3, 0), 0.0, 1e-12);
  EXPECT_NEAR(matrix(3, 1), 0.0, 1e-12);
  EXPECT_NEAR(matrix(3, 2), 0.0, 1e-12);
  EXPECT_NEAR(matrix(3, 3), 1.0, 1e-12);
  EXPECT_NEAR((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
  EXPECT_GT(rotation.determinant(), 0.0);

  EXPECT_NEAR(matrix(0, 3), 0.4, 1e-12);
  EXPECT_NEAR(matrix(1, 3), -0.2, 1e-12);
  EXPECT_NEAR(matrix(2, 3), 0.6, 1e-12);
  EXPECT_EQ(target_pose.value, received_pose);
  EXPECT_EQ(log[2].method_name, "StartTask");

}

TEST(FrankaRobotTest, ControllerFaultIsNotReportedAsCancellation)
{
  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  const std::string pose_id = "p0";
  const std::vector<std::string> pose_path{
    "Robot", "KeyValueMaps", "KeyPoseMap", "Read", pose_id};
  const std::string collision_error =
    "Moving failed: the defined force limits for collision detection have been exceeded";

  FrankaRobot robot(std::move(fake_client));
  ASSERT_TRUE(robot.connect());

  fake_ptr->writeValue(pose_path, Value(std::vector<double>(16, 0.0)));
  fake_ptr->queueValue(
    {"Robot", "ExecutionControl", "ExecutionStatus"},
    Value(ExecutionStatusValue{true, false, collision_error, "", ""}));

  const auto result = robot.moveToNamedPose(pose_id, []() {return true;});

  EXPECT_EQ(result.outcome, MotionOutcome::Failed);
  EXPECT_EQ(result.message, collision_error);

  const auto & calls = fake_ptr->callLog();
  ASSERT_EQ(calls.size(), 3u);
  EXPECT_EQ(calls[2].method_name, "StartTask");
}

TEST(FrankaRobotTest, StopRequestCancelsMoveWithoutCallingStopTask)
{
  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  const std::string pose_id = "p0";
  const std::vector<std::string> pose_path{
    "Robot", "KeyValueMaps", "KeyPoseMap", "Read", pose_id};

  FrankaRobot robot(std::move(fake_client));
  ASSERT_TRUE(robot.connect());

  fake_ptr->writeValue(pose_path, Value(std::vector<double>(16, 0.0)));
  fake_ptr->queueValue(
    {"Robot", "ExecutionControl", "ExecutionStatus"},
    Value(ExecutionStatusValue{false, true, "", "opcua_goto", "task_0"}));

  const auto result = robot.moveToNamedPose(pose_id, []() {return true;});

  EXPECT_EQ(result.outcome, MotionOutcome::Cancelled);
  const auto & calls = fake_ptr->callLog();
  ASSERT_EQ(calls.size(), 3u);
  EXPECT_EQ(calls.back().method_name, "StartTask");
}


TEST(FrankaRobotTest, SavePoseConvertsToColumnMajorMatrix)
{
  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  FrankaRobot robot(std::move(fake_client));

  ASSERT_TRUE(robot.connect());

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.4;
  pose.position.y = -0.2;
  pose.position.z = 0.6;
  pose.orientation.w = std::sqrt(0.5);
  pose.orientation.z = std::sqrt(0.5);

  ASSERT_TRUE(robot.savePose("p_test", pose));

  const auto & log = fake_ptr->callLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0].method_name, "Replace");
  ASSERT_EQ(log[0].args.size(), 1u);
  ASSERT_TRUE(log[0].args[0].is<KeyPosePairValue>());

  const auto & saved_pose = log[0].args[0].as<KeyPosePairValue>();
  EXPECT_EQ(saved_pose.key, "p_test");
  ASSERT_EQ(saved_pose.value.size(), 16u);

  const std::vector<double> expected_matrix{
    0.0, 1.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.4, -0.2, 0.6, 1.0};

  for (std::size_t index = 0; index < expected_matrix.size(); ++index) {
    EXPECT_NEAR(saved_pose.value[index], expected_matrix[index], 1e-12);
  }

}

TEST(FrankaRobotTest, SavePoseConvertsQuaternionToRotationMatrix)
{
  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  FrankaRobot robot(std::move(fake_client));

  ASSERT_TRUE(robot.connect());

  const Eigen::Quaterniond expected_quaternion(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()));
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = expected_quaternion.w();
  pose.orientation.x = expected_quaternion.x();
  pose.orientation.y = expected_quaternion.y();
  pose.orientation.z = expected_quaternion.z();

  ASSERT_TRUE(robot.savePose("p_rotation", pose));

  const auto & saved_pose = fake_ptr->callLog()[0].args[0].as<KeyPosePairValue>();
  const Eigen::Map<const Eigen::Matrix4d> matrix(saved_pose.value.data());

  EXPECT_NEAR(
    (matrix.block<3, 3>(0, 0) - expected_quaternion.toRotationMatrix()).norm(),
    0.0, 1e-12);
  EXPECT_NEAR(matrix(3, 0), 0.0, 1e-12);
  EXPECT_NEAR(matrix(3, 1), 0.0, 1e-12);
  EXPECT_NEAR(matrix(3, 2), 0.0, 1e-12);
  EXPECT_NEAR(matrix(3, 3), 1.0, 1e-12);
}

TEST(FrankaRobotTest, ReadCartesianPoseConvertsRotationMatrixToQuaternion)
{
  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  FrankaRobot robot(std::move(fake_client));

  ASSERT_TRUE(robot.connect());

  const Eigen::Quaterniond expected_quaternion(
    Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = expected_quaternion.toRotationMatrix();
  matrix(0, 3) = 0.4;
  matrix(1, 3) = -0.2;
  matrix(2, 3) = 0.6;
  const std::vector<double> values(matrix.data(), matrix.data() + 16);

  ASSERT_TRUE(
    fake_ptr->writeValue(
      {"Robot", "ExecutionControl", "CartesianPose"}, Value(values)));

  const auto pose = robot.readCartesianPose();

  EXPECT_NEAR(pose.position.x, 0.4, 1e-12);
  EXPECT_NEAR(pose.position.y, -0.2, 1e-12);
  EXPECT_NEAR(pose.position.z, 0.6, 1e-12);
  expectQuaternionRotationNear(pose.orientation, expected_quaternion);
}

TEST(FrankaRobotTest, PoseMatrixPoseRoundTripPreservesPose)
{
  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  FrankaRobot robot(std::move(fake_client));

  ASSERT_TRUE(robot.connect());

  const Eigen::Quaterniond expected_quaternion(
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.9, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()));
  geometry_msgs::msg::Pose original_pose;
  original_pose.position.x = 0.4;
  original_pose.position.y = -0.2;
  original_pose.position.z = 0.6;
  original_pose.orientation.w = expected_quaternion.w();
  original_pose.orientation.x = expected_quaternion.x();
  original_pose.orientation.y = expected_quaternion.y();
  original_pose.orientation.z = expected_quaternion.z();

  ASSERT_TRUE(robot.savePose("p_round_trip", original_pose));
  const auto & saved_pose = fake_ptr->callLog()[0].args[0].as<KeyPosePairValue>();
  ASSERT_TRUE(
    fake_ptr->writeValue(
      {"Robot", "ExecutionControl", "CartesianPose"}, Value(saved_pose.value)));

  const auto round_trip_pose = robot.readCartesianPose();

  EXPECT_NEAR(round_trip_pose.position.x, original_pose.position.x, 1e-12);
  EXPECT_NEAR(round_trip_pose.position.y, original_pose.position.y, 1e-12);
  EXPECT_NEAR(round_trip_pose.position.z, original_pose.position.z, 1e-12);
  expectQuaternionRotationNear(round_trip_pose.orientation, expected_quaternion);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
