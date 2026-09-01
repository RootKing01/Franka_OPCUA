#include <gtest/gtest.h>
#include "franka_opcua_bridge/franka_robot.hpp"
#include "franka_opcua_bridge/fake_test_version/fake_protocol_client.hpp"

using franka_opcua_bridge::FrankaRobot;
using franka_opcua_bridge::FakeProtocolClient;


TEST(FrankaRobotTest, RequestControlCallsCorrectMethod){

    auto fake_client = std::make_unique<FakeProtocolClient>();
    FakeProtocolClient * fake_ptr = fake_client.get();

    FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

    bool success = robot.connect();
    ASSERT_TRUE(success);

    fake_ptr->writeValue({"Robot", "ExecutionControl", "ControlTokenActive"}, true);

    bool result = robot.requestControl();
    EXPECT_TRUE(result);
    
    const auto & log = fake_ptr->callLog();
    ASSERT_EQ(log.size(), 1u);
    EXPECT_EQ(log[0].object_path, (std::vector<std::string>{"Robot", "ExecutionControl"}));
    EXPECT_EQ(log[0].method_name, "RequestControlToken");
    
}

TEST(FrankaRobotTest, releaseControlCallsCorrectMethod){

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


TEST(FrankaRobotTest, AreBrakesOpenReturnsBrakeState){

    auto fake_client = std::make_unique<FakeProtocolClient>();
    FakeProtocolClient * fake_ptr = fake_client.get();

    FrankaRobot robot(std::move(fake_client), "endpoint", "user", "pass");

    bool success = robot.connect();
    ASSERT_TRUE(success);

    fake_ptr->writeValue({"Robot","ExecutionControl","BrakesOpen"}, true);

    bool result = robot.areBrakesOpen();

    EXPECT_TRUE(result);

}

TEST(FrankaRobotTest, stopCallsCorrectMethod){

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


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}