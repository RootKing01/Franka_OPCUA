#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "franka_opcua_bridge/opcua_worker.hpp"

using franka_opcua_bridge::FrankaCommandResult;
using franka_opcua_bridge::FrankaCommandStatus;
using franka_opcua_bridge::FrankaCommandType;
using franka_opcua_bridge::FrankaRobotCommand;
using franka_opcua_bridge::IFranka;
using franka_opcua_bridge::OpcuaWorker;
using franka_opcua_bridge::RobotStatus;
using franka_opcua_bridge::MotionOutcome;
using franka_opcua_bridge::MotionResult;
using franka_opcua_bridge::WorkerStatus;

namespace
{

class BlockingRobot : public IFranka
{
public:
  bool connect() override
  {
    connected_ = true;
    return true;
  }

  bool disconnect() override
  {
    connected_ = false;
    return true;
  }

  bool isConnected() const override {return connected_;}
  bool requestControl(bool) override {return connected_;}
  bool releaseControl() override {return true;}
  bool openBrakes() override {return true;}
  bool closeBrakes() override {return true;}
  bool areBrakesOpen() override {return false;}
  bool stop() override
  {
    stop_call_count_.fetch_add(1);
    return true;
  }

  bool executeNamedTask(const std::string &) override {return true;}

  MotionResult moveToNamedPose(
    const std::string &,
    const std::function<bool()> & should_abort) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      move_entered_ = true;
    }
    condition_.notify_all();

    std::unique_lock<std::mutex> lock(mutex_);
    while (!should_abort()) {
      condition_.wait_for(lock, std::chrono::milliseconds(10));
    }
    move_interrupted_ = true;
    if (collision_after_abort_.load()) {
      return {
        MotionOutcome::Failed,
        "Moving failed: the defined force limits for collision detection have been exceeded"};
    }
    return {MotionOutcome::Cancelled, "Motion cancelled by stop request"};
  }

  bool activateFCI() override {return true;}
  bool deactivateFCI() override {return true;}

  std::unique_ptr<RobotStatus> readStatus() override
  {
    return std::make_unique<RobotStatus>();
  }

  std::vector<double> readJointAngles() override {return {};}
  geometry_msgs::msg::Pose readCartesianPose() override {return geometry_msgs::msg::Pose();}
  std::vector<double> readTorque() override {return {};}
  geometry_msgs::msg::Wrench readWrench() override {return geometry_msgs::msg::Wrench();}

  bool waitUntilMoveEntered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock, std::chrono::milliseconds(500), [this]() {return move_entered_;});
  }

  bool moveWasInterrupted() const {return move_interrupted_;}
  int stopCallCount() const {return stop_call_count_;}
  void setCollisionAfterAbort(bool value) {collision_after_abort_.store(value);}

private:
  bool connected_ = false;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool move_entered_ = false;
  std::atomic_bool move_interrupted_{false};
  std::atomic_bool collision_after_abort_{false};
  std::atomic_int stop_call_count_{0};
};

class FailingRobot final : public BlockingRobot
{
public:
  FailingRobot(bool fail_connect, bool fail_control)
  : fail_connect_(fail_connect), fail_control_(fail_control) {}

  bool connect() override
  {
    if (fail_connect_) {
      return false;
    }
    return BlockingRobot::connect();
  }

  bool requestControl(bool) override
  {
    if (fail_control_) {
      return false;
    }
    return BlockingRobot::requestControl(true);
  }

private:
  bool fail_connect_;
  bool fail_control_;
};

FrankaRobotCommand command(
  std::uint64_t id, FrankaCommandType type,
  const std::string & parameter = {})
{
  return FrankaRobotCommand{id, type, parameter, std::nullopt, std::chrono::steady_clock::now()};
}

std::optional<FrankaCommandResult> waitForResult(OpcuaWorker & worker, std::uint64_t command_id)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
  while (std::chrono::steady_clock::now() < deadline) {
    auto result = worker.tryGetResult();
    if (result && result->command_id == command_id) {
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return std::nullopt;
}

}  // namespace

TEST(OpcuaWorkerTest, StopInterruptsBlockingMove)
{
  auto robot = std::make_unique<BlockingRobot>();
  BlockingRobot * robot_ptr = robot.get();
  OpcuaWorker worker(std::move(robot), nullptr, 2);

  ASSERT_TRUE(worker.start());
  ASSERT_EQ(worker.status(), WorkerStatus::Ready);
  ASSERT_TRUE(worker.enqueueCommand(command(1, FrankaCommandType::MoveToNamedPose, "p0")));
  ASSERT_TRUE(robot_ptr->waitUntilMoveEntered());

  ASSERT_TRUE(worker.enqueuePriority(command(2, FrankaCommandType::Stop)));

  auto move_result = waitForResult(worker, 1);
  ASSERT_TRUE(move_result.has_value());
  EXPECT_EQ(move_result->status, FrankaCommandStatus::Cancelled);
  EXPECT_TRUE(robot_ptr->moveWasInterrupted());

  auto stop_result = waitForResult(worker, 2);
  ASSERT_TRUE(stop_result.has_value());
  EXPECT_EQ(stop_result->status, FrankaCommandStatus::Completed);
  EXPECT_EQ(robot_ptr->stopCallCount(), 1);

  worker.stop();
  EXPECT_EQ(worker.status(), WorkerStatus::Disconnected);
}

TEST(OpcuaWorkerTest, PriorityCommandIsRejectedBeforeWorkerIsReady)
{
  auto robot = std::make_unique<BlockingRobot>();
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  EXPECT_FALSE(worker.enqueuePriority(command(1, FrankaCommandType::Stop)));
}

TEST(OpcuaWorkerTest, StopWhenReadyDisconnectsWorker)
{
  auto robot = std::make_unique<BlockingRobot>();
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  ASSERT_TRUE(worker.start());
  ASSERT_EQ(worker.status(), WorkerStatus::Ready);

  worker.stop();

  EXPECT_EQ(worker.status(), WorkerStatus::Disconnected);
  EXPECT_FALSE(worker.enqueueCommand(command(1, FrankaCommandType::OpenBrakes)));
  EXPECT_FALSE(worker.start());
}

TEST(OpcuaWorkerTest, PreservesControllerFaultAsMoveFailure)
{
  auto robot = std::make_unique<BlockingRobot>();
  BlockingRobot * robot_ptr = robot.get();
  robot_ptr->setCollisionAfterAbort(true);
  OpcuaWorker worker(std::move(robot), nullptr, 2);

  ASSERT_TRUE(worker.start());
  ASSERT_TRUE(worker.enqueueCommand(command(7, FrankaCommandType::MoveToNamedPose, "p0")));
  ASSERT_TRUE(robot_ptr->waitUntilMoveEntered());
  ASSERT_TRUE(worker.enqueuePriority(command(8, FrankaCommandType::Stop)));

  auto result = waitForResult(worker, 7);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, FrankaCommandStatus::Failed);
  EXPECT_EQ(
    result->message,
    "Moving failed: the defined force limits for collision detection have been exceeded");

  auto stop_result = waitForResult(worker, 8);
  ASSERT_TRUE(stop_result.has_value());
  EXPECT_EQ(stop_result->status, FrankaCommandStatus::NotNeeded);
  EXPECT_EQ(robot_ptr->stopCallCount(), 0);

  worker.stop();
}

TEST(OpcuaWorkerTest, ConnectFailureLeavesWorkerInError)
{
  auto robot = std::make_unique<FailingRobot>(true, false);
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  EXPECT_FALSE(worker.start());
  EXPECT_EQ(worker.status(), WorkerStatus::Error);
  EXPECT_FALSE(worker.enqueueCommand(command(1, FrankaCommandType::OpenBrakes)));
}

TEST(OpcuaWorkerTest, ControlFailureLeavesWorkerInError)
{
  auto robot = std::make_unique<FailingRobot>(false, true);
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  EXPECT_FALSE(worker.start());
  EXPECT_EQ(worker.status(), WorkerStatus::Error);
  EXPECT_FALSE(worker.enqueuePriority(command(1, FrankaCommandType::Stop)));
}
