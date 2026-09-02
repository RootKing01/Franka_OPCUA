#include <gtest/gtest.h>
#include "franka_opcua_bridge/franka_robot.hpp"
#include "franka_opcua_bridge/fake_test_version/fake_protocol_client.hpp"

using franka_opcua_bridge::FrankaRobot;
using franka_opcua_bridge::FakeProtocolClient;
using franka_opcua_bridge::Value;


TEST(FrankaRobotTest, RequestControlCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue({"Robot", "ExecutionControl", "ControlTokenActive"}, Value(true));

  bool result = robot.requestControl();
  EXPECT_TRUE(result);

  const auto & log = fake_ptr->callLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0].object_path, (std::vector<std::string>{"Robot", "ExecutionControl"}));
  EXPECT_EQ(log[0].method_name, "RequestControlToken");

}

TEST(FrankaRobotTest, releaseControlCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

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

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

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

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

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

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue({"Robot", "ExecutionControl", "BrakesOpen"}, Value(true));

  bool result = robot.areBrakesOpen();

  EXPECT_TRUE(result);

}


TEST(FrankaRobotTest, AreBrakesOpenReturnsFalse) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue({"Robot", "ExecutionControl", "BrakesOpen"}, Value(false));

  bool result = robot.areBrakesOpen();

  EXPECT_FALSE(result);

}

TEST(FrankaRobotTest, stopCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

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

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue(path_, Value(angles_));

  output_ = robot.readJointAngles();
  EXPECT_EQ(output_, angles_);


}

TEST(FrankaRobotTest, ReadJointAnglesReturnsEmptyIfNotSet)
{
  auto fake_client = std::make_unique<FakeProtocolClient>();

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");
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

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

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

TEST(FrankaRobotTest, executeNamedPoseCallsCorrectMethod) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  const std::string task_id = "opcua_goto";

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

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

TEST(FrankaRobotTest, MoveToNamedPoseCall) {

  auto fake_client = std::make_unique<FakeProtocolClient>();
  FakeProtocolClient * fake_ptr = fake_client.get();
  const std::string pose_id = "p0";
  std::vector<std::string> variable_browse_path = {"Robot", "KeyValueMaps", "KeyPoseMap", "Read"};

  variable_browse_path.push_back(pose_id);

  FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

  bool success = robot.connect();
  ASSERT_TRUE(success);

  fake_ptr->writeValue(
    variable_browse_path, Value(
      std::vector<double>(
        {1.0, 1.0, 2.0, 3.0, 5.0, 8.0, 13.0, 21.0,
          34.0, 55.0, 89.0, 144.0, 3.0, 5.0, 8.0, 13.0})) );

  bool movePoseSuccess = robot.moveToNamedPose(pose_id);
  ASSERT_TRUE(movePoseSuccess);

}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
