#include <gtest/gtest.h>
#include "franka_opcua_bridge/franka_robot.hpp"
#include "franka_opcua_bridge/fake_test_version/fake_protocol_client.hpp"

using franka_opcua_bridge::FrankaRobot;
using franka_opcua_bridge::FakeProtocolClient;

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

TEST(FrankaRobotTest, CloseBrakesCallsCorrectMethod){

    auto fake_client = std::make_unique<FakeProtocolClient>();
    FakeProtocolClient * fake_ptr = fake_client.get(); //puntatore di ispezione

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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}