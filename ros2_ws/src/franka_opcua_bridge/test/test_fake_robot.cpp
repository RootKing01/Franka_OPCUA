#include <gtest/gtest.h>
#include "franka_opcua_bridge/fake_test_version/fake_robot.hpp"

using franka_opcua_bridge::FakeRobot;
using franka_opcua_bridge::IRobot;

TEST(FakeRobotTest, ConnectSetsConnectedState)
{
  FakeRobot robot;
  EXPECT_FALSE(robot.isConnected());
  EXPECT_TRUE(robot.connect());
  EXPECT_TRUE(robot.isConnected());
}

TEST(FakeRobotTest, CannotOpenBrakesIfNotConnected)
{
  FakeRobot robot;
  EXPECT_FALSE(robot.openBrakes());
  EXPECT_FALSE(robot.areBrakesOpen());
}

TEST(FakeRobotTest, TaskFailsIfBrakesClosed)
{
  FakeRobot robot;
  robot.connect();
  EXPECT_FALSE(robot.executeNamedTask("some_task"));
}

TEST(FakeRobotTest, TaskSucceedsIfBrakesOpen)
{
  FakeRobot robot;
  robot.connect();
  robot.openBrakes();
  EXPECT_TRUE(robot.executeNamedTask("some_task"));

  auto status = robot.readStatus();
  EXPECT_TRUE(status->is_running);
  EXPECT_EQ(status->active_task_name, "some_task");
}

TEST(FakeRobotTest, ReadTorque)
{
  FakeRobot robot;
  robot.connect();

  auto torque = robot.readTorque();

  ASSERT_EQ(torque.size(), 7);

  EXPECT_DOUBLE_EQ(torque[0], 0.0);
  EXPECT_DOUBLE_EQ(torque[1], -0.785);
  EXPECT_DOUBLE_EQ(torque[2], 0.0);
  EXPECT_DOUBLE_EQ(torque[3], -2.356);
  EXPECT_DOUBLE_EQ(torque[4], 0.0);
  EXPECT_DOUBLE_EQ(torque[5], 1.571);
  EXPECT_DOUBLE_EQ(torque[6], 0.785);
}


TEST(FakeRobotTest, ReadWrench)
{
  FakeRobot robot;
  robot.connect();
  
  auto wrench = robot.readWrench();

  EXPECT_DOUBLE_EQ(wrench.force.x, 0.0);
  EXPECT_DOUBLE_EQ(wrench.force.y, 0.785);
  EXPECT_DOUBLE_EQ(wrench.force.z, 1.0);

  EXPECT_DOUBLE_EQ(wrench.torque.x, 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque.y, 1.0);
  EXPECT_DOUBLE_EQ(wrench.torque.z, 0.0);
}

TEST(FakeRobotTest, MoveToNamedPoseUpdatesStatus)
{
  FakeRobot robot;
  robot.connect();
  robot.openBrakes();
  EXPECT_TRUE(robot.moveToNamedPose("p0"));
  EXPECT_EQ(robot.lastPoseTarget(), "p0");
}

TEST(FakeRobotTest, WorksThroughAbstractInterface)
{
  std::unique_ptr<IRobot> robot = std::make_unique<FakeRobot>();
  EXPECT_TRUE(robot->connect());
  EXPECT_TRUE(robot->openBrakes());
  EXPECT_TRUE(robot->executeNamedTask("test_task"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
