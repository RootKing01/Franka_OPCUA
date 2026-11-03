#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <stdexcept>
#include <thread>

#include "franka_opcua_bridge/commands/franka_command_manager.hpp"

using franka_opcua_bridge::FrankaCommandManager;
using franka_opcua_bridge::FrankaCommandType;
using franka_opcua_bridge::FrankaRobotCommand;

namespace
{

FrankaRobotCommand command(
  std::uint64_t id,
  FrankaCommandType type = FrankaCommandType::OpenBrakes)
{
  return FrankaRobotCommand{id, type, {}, std::nullopt, std::chrono::steady_clock::now()};
}

}  // namespace

TEST(FrankaCommandManagerTest, PriorityCommandsAreExtractedFirst)
{
  FrankaCommandManager manager(3);
  ASSERT_TRUE(manager.enqueue(command(1)));
  ASSERT_TRUE(manager.enqueue(command(2)));
  ASSERT_TRUE(manager.enqueuePriority(command(3, FrankaCommandType::Stop)));

  EXPECT_EQ(manager.waitAndPop()->command_id, 3u);
  EXPECT_EQ(manager.waitAndPop()->command_id, 1u);
  EXPECT_EQ(manager.waitAndPop()->command_id, 2u);
}

TEST(FrankaCommandManagerTest, RejectsZeroCapacity)
{
  EXPECT_THROW(FrankaCommandManager manager(0), std::invalid_argument);
}

TEST(FrankaCommandManagerTest, TryPopUsesPriorityAndPreservesFifoOrder)
{
  FrankaCommandManager manager(3);
  ASSERT_TRUE(manager.enqueue(command(1)));
  ASSERT_TRUE(manager.enqueue(command(2)));
  ASSERT_TRUE(manager.enqueuePriority(command(3, FrankaCommandType::Stop)));

  EXPECT_FALSE(manager.empty());
  EXPECT_FALSE(manager.priorityEmpty());
  EXPECT_EQ(manager.size(), 2u);
  EXPECT_EQ(manager.prioritySize(), 1u);
  ASSERT_EQ(manager.tryPop()->command_id, 3u);
  ASSERT_EQ(manager.tryPop()->command_id, 1u);
  ASSERT_EQ(manager.tryPop()->command_id, 2u);
  EXPECT_FALSE(manager.tryPop().has_value());
  EXPECT_TRUE(manager.empty());
  EXPECT_TRUE(manager.priorityEmpty());
}

TEST(FrankaCommandManagerTest, EnforcesCapacityForEachQueue)
{
  FrankaCommandManager manager(1);

  EXPECT_TRUE(manager.enqueue(command(1)));
  EXPECT_FALSE(manager.enqueue(command(2)));
  EXPECT_TRUE(manager.enqueuePriority(command(3, FrankaCommandType::Stop)));
  EXPECT_FALSE(manager.enqueuePriority(command(4, FrankaCommandType::Stop)));
}

TEST(FrankaCommandManagerTest, ShutdownRejectsNewCommands)
{
  FrankaCommandManager manager(2);
  manager.shutdown();

  EXPECT_TRUE(manager.isShutdown());
  EXPECT_FALSE(manager.enqueue(command(1)));
  EXPECT_FALSE(manager.enqueuePriority(command(2, FrankaCommandType::Stop)));
}

TEST(FrankaCommandManagerTest, ShutdownDiscardsPendingCommands)
{
  FrankaCommandManager manager(2);
  ASSERT_TRUE(manager.enqueue(command(1)));
  ASSERT_TRUE(manager.enqueuePriority(command(2, FrankaCommandType::Stop)));

  manager.shutdown();

  EXPECT_FALSE(manager.waitAndPop().has_value());
  EXPECT_EQ(manager.size(), 1u);
  EXPECT_EQ(manager.prioritySize(), 1u);
}

TEST(FrankaCommandManagerTest, ShutdownWakesWaitingConsumer)
{
  FrankaCommandManager manager(1);
  auto result = std::async(
    std::launch::async, [&manager]() {
      return manager.waitAndPop();
    });

  ASSERT_EQ(
    result.wait_for(std::chrono::milliseconds(100)),
    std::future_status::timeout);

  manager.shutdown();

  ASSERT_EQ(result.wait_for(std::chrono::milliseconds(500)), std::future_status::ready);
  EXPECT_FALSE(result.get().has_value());
}
