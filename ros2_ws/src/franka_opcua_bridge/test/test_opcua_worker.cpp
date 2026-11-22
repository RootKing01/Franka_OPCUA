#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

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
using franka_opcua_bridge::PerformanceMonitor;
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
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = false;
    events_.push_back("disconnect");
    return true;
  }

  bool isConnected() const override {return connected_;}
  bool requestControl(bool) override {return connected_;}
  bool releaseControl() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    events_.push_back("release");
    return true;
  }
  bool openBrakes() override {return true;}
  bool closeBrakes() override {return true;}
  bool areBrakesOpen() override {return false;}
  bool stop() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    events_.push_back("stop");
    stop_entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return !hold_stop_ || stop_released_;});
    stop_call_count_.fetch_add(1);
    return true;
  }

  franka_opcua_bridge::TaskExecutionResult executeNamedTask(
    const std::string &,
    const std::function<bool()> & should_abort = {}) override
  {
    if (should_abort && should_abort()) {
      return {franka_opcua_bridge::TaskState::Cancelled, "Task cancelled by stop request"};
    }
    return {franka_opcua_bridge::TaskState::Completed, "Task completed"};
  }

  MotionResult moveToNamedPose(
    const std::string &,
    const std::string &,
    const std::vector<std::string> &,
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
  std::optional<franka_opcua_bridge::namespaceTree> showAddressSpaceNodes() override
  {
    return std::nullopt;
  }

  std::unique_ptr<RobotStatus> readStatus() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (return_null_status_) {
      return nullptr;
    }
    auto status = std::make_unique<RobotStatus>();
    status->is_running = status_is_running_;
    status->has_error = status_has_error_;
    return status;
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

  bool waitUntilStopEntered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock, std::chrono::milliseconds(500), [this]() {return stop_entered_;});
  }

  bool moveWasInterrupted() const {return move_interrupted_;}
  int stopCallCount() const {return stop_call_count_;}
  void setCollisionAfterAbort(bool value) {collision_after_abort_.store(value);}
  void setRobotStatus(bool is_running, bool has_error = false)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_is_running_ = is_running;
    status_has_error_ = has_error;
    return_null_status_ = false;
  }
  void setNullRobotStatus()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return_null_status_ = true;
  }
  std::vector<std::string> events() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }
  void holdStopUntilReleased()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_stop_ = true;
  }
  void releaseStop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_released_ = true;
    }
    condition_.notify_all();
  }

private:
  bool connected_ = false;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool move_entered_ = false;
  bool stop_entered_ = false;
  bool hold_stop_ = false;
  bool stop_released_ = false;
  bool status_is_running_ = false;
  bool status_has_error_ = false;
  bool return_null_status_ = false;
  std::vector<std::string> events_;
  std::atomic_bool move_interrupted_{false};
  std::atomic_bool collision_after_abort_{false};
  std::atomic_int stop_call_count_{0};
};

class StopReleaseGuard
{
public:
  explicit StopReleaseGuard(BlockingRobot * robot)
  : robot_(robot) {}

  ~StopReleaseGuard()
  {
    robot_->releaseStop();
  }

private:
  BlockingRobot * robot_;
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
  FrankaRobotCommand result{};
  result.command_id = id;
  result.type = type;
  result.task_name = type == FrankaCommandType::MoveToNamedPose ? "opcua_goto" : "";
  result.parameter = parameter;
  if (type == FrankaCommandType::MoveToNamedPose) {
    result.task_poses = {"target_pose"};
  }
  result.created_at = std::chrono::steady_clock::now();
  return result;
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

std::unordered_map<std::uint64_t, FrankaCommandResult> waitForResults(
  OpcuaWorker & worker, std::size_t expected_count)
{
  std::unordered_map<std::uint64_t, FrankaCommandResult> results;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
  while (std::chrono::steady_clock::now() < deadline && results.size() < expected_count) {
    auto result = worker.tryGetResult();
    if (result) {
      results.emplace(result->command_id, std::move(*result));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  return results;
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

TEST(OpcuaWorkerTest, ReportsNamedTaskOnlyAfterTerminalTaskResult)
{
  auto robot = std::make_unique<BlockingRobot>();
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  ASSERT_TRUE(worker.start());
  ASSERT_TRUE(
    worker.enqueueCommand(
      command(1, FrankaCommandType::ExecuteNamedTask, "pick_task")));

  const auto result = waitForResult(worker, 1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, FrankaCommandStatus::Completed);
  EXPECT_EQ(result->message, "Task completed");

  worker.stop();
}

TEST(OpcuaWorkerTest, CapturesCommandTimingBreakdown)
{
  auto robot = std::make_unique<BlockingRobot>();
  auto performance_monitor = std::make_shared<PerformanceMonitor>();
  OpcuaWorker worker(std::move(robot), nullptr, 2, nullptr, performance_monitor);

  ASSERT_TRUE(worker.start());
  ASSERT_TRUE(worker.enqueueCommand(command(1, FrankaCommandType::OpenBrakes)));

  const auto result = waitForResult(worker, 1);
  ASSERT_TRUE(result.has_value());
  EXPECT_GE(result->queue_wait_duration.count(), 0);
  EXPECT_GE(result->execution_duration.count(), 0);
  EXPECT_GE(result->processing_duration, result->execution_duration);

  const auto snapshot = performance_monitor->takeSnapshot();
  EXPECT_EQ(snapshot.at("command.queue_wait").count, 1u);
  EXPECT_EQ(snapshot.at("command.execution").count, 1u);
  EXPECT_EQ(snapshot.at("command.processing").count, 1u);

  worker.stop();
}

TEST(OpcuaWorkerTest, StopCancelsQueuedCommandsAndReportsEveryResult)
{
  auto robot = std::make_unique<BlockingRobot>();
  BlockingRobot * robot_ptr = robot.get();
  OpcuaWorker worker(std::move(robot), nullptr, 3);

  ASSERT_TRUE(worker.start());
  ASSERT_TRUE(worker.enqueueCommand(command(1, FrankaCommandType::MoveToNamedPose, "p0")));
  ASSERT_TRUE(robot_ptr->waitUntilMoveEntered());
  ASSERT_TRUE(worker.enqueueCommand(command(2, FrankaCommandType::OpenBrakes)));
  ASSERT_TRUE(worker.enqueueCommand(command(3, FrankaCommandType::CloseBrakes)));

  ASSERT_TRUE(worker.enqueuePriority(command(4, FrankaCommandType::Stop)));

  const auto results = waitForResults(worker, 4);
  ASSERT_EQ(results.size(), 4u);
  EXPECT_EQ(results.at(1).status, FrankaCommandStatus::Cancelled);
  EXPECT_EQ(results.at(2).status, FrankaCommandStatus::Cancelled);
  EXPECT_EQ(results.at(3).status, FrankaCommandStatus::Cancelled);
  EXPECT_EQ(results.at(4).status, FrankaCommandStatus::Completed);
  EXPECT_TRUE(robot_ptr->moveWasInterrupted());
  EXPECT_EQ(robot_ptr->stopCallCount(), 1);

  worker.stop();
}

TEST(OpcuaWorkerTest, RejectsNormalCommandWhileStopIsPending)
{
  auto robot = std::make_unique<BlockingRobot>();
  BlockingRobot * robot_ptr = robot.get();
  robot_ptr->holdStopUntilReleased();
  OpcuaWorker worker(std::move(robot), nullptr, 2);
  StopReleaseGuard stop_release_guard(robot_ptr);

  ASSERT_TRUE(worker.start());
  ASSERT_TRUE(worker.enqueueCommand(command(1, FrankaCommandType::MoveToNamedPose, "p0")));
  ASSERT_TRUE(robot_ptr->waitUntilMoveEntered());
  ASSERT_TRUE(worker.enqueuePriority(command(2, FrankaCommandType::Stop)));
  ASSERT_TRUE(robot_ptr->waitUntilStopEntered());

  EXPECT_FALSE(worker.enqueueCommand(command(3, FrankaCommandType::OpenBrakes)));
  robot_ptr->releaseStop();

  const auto results = waitForResults(worker, 2);
  ASSERT_EQ(results.size(), 2u);
  EXPECT_EQ(results.at(1).status, FrankaCommandStatus::Cancelled);
  EXPECT_EQ(results.at(2).status, FrankaCommandStatus::Completed);
  EXPECT_TRUE(robot_ptr->moveWasInterrupted());
  EXPECT_EQ(robot_ptr->stopCallCount(), 1);
  EXPECT_FALSE(worker.tryGetResult().has_value());

  worker.stop();
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

TEST(OpcuaWorkerTest, ShutdownStopsActiveTaskBeforeReleasingControl)
{
  auto robot = std::make_unique<BlockingRobot>();
  BlockingRobot * robot_ptr = robot.get();
  robot_ptr->setRobotStatus(true);
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  ASSERT_TRUE(worker.start());
  worker.stop();

  EXPECT_EQ(robot_ptr->stopCallCount(), 1);
  const auto events = robot_ptr->events();
  const auto stop = std::find(events.begin(), events.end(), "stop");
  const auto release = std::find(events.begin(), events.end(), "release");
  const auto disconnect = std::find(events.begin(), events.end(), "disconnect");
  ASSERT_NE(stop, events.end());
  ASSERT_NE(release, events.end());
  ASSERT_NE(disconnect, events.end());
  EXPECT_LT(stop, release);
  EXPECT_LT(release, disconnect);
}

TEST(OpcuaWorkerTest, ShutdownAbortsActiveMoveAndStopsTask)
{
  auto robot = std::make_unique<BlockingRobot>();
  BlockingRobot * robot_ptr = robot.get();
  robot_ptr->setRobotStatus(true);
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  ASSERT_TRUE(worker.start());
  ASSERT_TRUE(worker.enqueueCommand(command(1, FrankaCommandType::MoveToNamedPose, "p0")));
  ASSERT_TRUE(robot_ptr->waitUntilMoveEntered());

  worker.stop();

  EXPECT_TRUE(robot_ptr->moveWasInterrupted());
  EXPECT_EQ(robot_ptr->stopCallCount(), 1);
  auto move_result = worker.tryGetResult();
  ASSERT_TRUE(move_result.has_value());
  EXPECT_EQ(move_result->command_id, 1u);
  EXPECT_EQ(move_result->status, FrankaCommandStatus::Cancelled);
  const auto events = robot_ptr->events();
  const auto stop = std::find(events.begin(), events.end(), "stop");
  const auto release = std::find(events.begin(), events.end(), "release");
  const auto disconnect = std::find(events.begin(), events.end(), "disconnect");
  ASSERT_NE(stop, events.end());
  ASSERT_NE(release, events.end());
  ASSERT_NE(disconnect, events.end());
  EXPECT_LT(stop, release);
  EXPECT_LT(release, disconnect);
}

TEST(OpcuaWorkerTest, ShutdownSkipsStopWhenRobotIsConfirmedIdle)
{
  auto robot = std::make_unique<BlockingRobot>();
  BlockingRobot * robot_ptr = robot.get();
  robot_ptr->setRobotStatus(false);
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  ASSERT_TRUE(worker.start());
  worker.stop();

  EXPECT_EQ(robot_ptr->stopCallCount(), 0);
  const auto events = robot_ptr->events();
  EXPECT_EQ(std::find(events.begin(), events.end(), "stop"), events.end());
  const auto release = std::find(events.begin(), events.end(), "release");
  const auto disconnect = std::find(events.begin(), events.end(), "disconnect");
  ASSERT_NE(release, events.end());
  ASSERT_NE(disconnect, events.end());
  EXPECT_LT(release, disconnect);
}

TEST(OpcuaWorkerTest, ShutdownAttemptsStopWhenRobotStatusHasError)
{
  auto robot = std::make_unique<BlockingRobot>();
  BlockingRobot * robot_ptr = robot.get();
  robot_ptr->setRobotStatus(false, true);
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  ASSERT_TRUE(worker.start());
  worker.stop();

  EXPECT_EQ(robot_ptr->stopCallCount(), 1);
}

TEST(OpcuaWorkerTest, ShutdownAttemptsStopWhenRobotStatusIsUnavailable)
{
  auto robot = std::make_unique<BlockingRobot>();
  BlockingRobot * robot_ptr = robot.get();
  robot_ptr->setNullRobotStatus();
  OpcuaWorker worker(std::move(robot), nullptr, 1);

  ASSERT_TRUE(worker.start());
  worker.stop();

  EXPECT_EQ(robot_ptr->stopCallCount(), 1);
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
