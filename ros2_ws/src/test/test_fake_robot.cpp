#include <gtest/gtest.h>
#include "franka_opcua_bridge/fake_robot.hpp"

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
  // freni ancora chiusi
  EXPECT_FALSE(robot.executeNamedTask("some_task"));
}

TEST(FakeRobotTest, TaskSucceedsIfBrakesOpen)
{
  FakeRobot robot;
  robot.connect();
  robot.openBrakes();
  EXPECT_TRUE(robot.executeNamedTask("some_task"));

  auto status = robot.readStatus();
  EXPECT_TRUE(status.is_running);
  EXPECT_EQ(status.active_task_name, "some_task");
}

TEST(FakeRobotTest, MoveToNamedPoseUpdatesStatus)
{
  FakeRobot robot;
  robot.connect();
  robot.openBrakes();
  EXPECT_TRUE(robot.moveToNamedPose("p0"));
  EXPECT_EQ(robot.lastPoseTarget(), "p0");
}

// Verifica che il polimorfismo funzioni davvero: usiamo FakeRobot
// SOLO attraverso il puntatore IRobot*, come farebbe il nodo ROS 2 reale.
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