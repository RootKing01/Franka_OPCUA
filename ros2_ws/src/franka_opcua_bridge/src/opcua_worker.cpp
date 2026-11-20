#include "franka_opcua_bridge/opcua_worker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <stdexcept>

namespace franka_opcua_bridge
{

namespace
{

rclcpp::Logger workerLogger()
{
  return rclcpp::get_logger("franka_opcua_worker");
}

std::chrono::steady_clock::time_point commandReceivedAt(const FrankaRobotCommand & command)
{
  return command.received_at == std::chrono::steady_clock::time_point{} ?
         command.created_at : command.received_at;
}

void populateCommandTimings(
  const FrankaRobotCommand & command,
  FrankaCommandResult & result,
  std::chrono::steady_clock::time_point execution_started_at)
{
  result.received_at = commandReceivedAt(command);
  result.queue_wait_duration = std::max(
    std::chrono::steady_clock::duration::zero(), execution_started_at - command.created_at);
  result.execution_duration = std::max(
    std::chrono::steady_clock::duration::zero(), result.finished_at - execution_started_at);
  result.processing_duration = std::max(
    std::chrono::steady_clock::duration::zero(), result.finished_at - result.received_at);
}

void recordCommandTimings(
  const std::shared_ptr<PerformanceMonitor> & performance_monitor,
  const FrankaCommandResult & result)
{
  if (!performance_monitor) {
    return;
  }
  const bool success = result.status != FrankaCommandStatus::Failed &&
    result.status != FrankaCommandStatus::Timeout;
  performance_monitor->record("command.queue_wait", result.queue_wait_duration, success);
  performance_monitor->record("command.execution", result.execution_duration, success);
  performance_monitor->record("command.processing", result.processing_duration, success);
}

}  // namespace

OpcuaWorker::OpcuaWorker(
  std::unique_ptr<IFranka> robot,
  IPoseStorage * pose_storage,
  std::size_t command_capacity,
  ITaskControl * task_control,
  std::shared_ptr<PerformanceMonitor> performance_monitor)
: franka_(std::move(robot)),
  pose_storage_(pose_storage),
  task_control_(task_control),
  performance_monitor_(std::move(performance_monitor)),
  command_manager_(command_capacity),
  worker_state_(WorkerStatus::Disconnected)
{
  if (franka_ == nullptr) {throw std::invalid_argument("Robot cannot be null");}
}


OpcuaWorker::~OpcuaWorker()
{
  stop();
}

bool OpcuaWorker::start()
{

  RCLCPP_INFO(workerLogger(), "Avvio worker richiesto");

  if (worker_thread_.joinable()) {
    RCLCPP_WARN(workerLogger(), "Avvio rifiutato: thread worker gia' attivo");
    return false;
  }

  std::unique_lock<std::mutex> lock(mutex_);

  if (start_attempted_) {
    RCLCPP_WARN(workerLogger(), "Avvio rifiutato: worker utilizzabile una sola volta");
    return false;
  }

  if (worker_state_ != WorkerStatus::Disconnected) {
    RCLCPP_WARN(workerLogger(), "Avvio rifiutato: worker non disconnesso");
    return false;
  }

  start_attempted_ = true;
  worker_state_ = WorkerStatus::Connecting;

  lock.unlock();

  worker_thread_ = std::thread(
    [this]() {
      run();
    });

  lock.lock();

  state_condition_.wait(
    lock, [this] {
      return worker_state_ == WorkerStatus::Ready ||
      worker_state_ == WorkerStatus::Error ||
      worker_state_ == WorkerStatus::Stopping ||
      worker_state_ == WorkerStatus::Disconnected;
    });


  if (worker_state_ == WorkerStatus::Ready) {
    RCLCPP_INFO(workerLogger(), "Worker pronto");
    lock.unlock();
    return true;
  }

  if (worker_state_ == WorkerStatus::Error) {

    RCLCPP_ERROR(
      workerLogger(),
      "Worker non avviato: errore durante la connessione o il controllo");
    lock.unlock();
    worker_thread_.join();
    return false;
  }

  return false;

}

void OpcuaWorker::stop()
{

  if ((worker_thread_.joinable())) {

    RCLCPP_INFO(workerLogger(), "Arresto worker richiesto");

    std::unique_lock<std::mutex> lock(mutex_);

    worker_state_ = WorkerStatus::Stopping;
    abort_requested_.store(true);

    command_manager_.shutdown();

    lock.unlock();

    while (true) {
      auto command = command_manager_.tryPop();

      if (!command) {

        break;
      }

      FrankaCommandResult result;
      result.command_id = command->command_id;
      result.status = FrankaCommandStatus::Cancelled;
      result.message = "Worker stoppato. Comando non eseguito";
      result.finished_at = std::chrono::steady_clock::now();
      populateCommandTimings(*command, result, result.finished_at);
      recordCommandTimings(performance_monitor_, result);

      pushResult(std::move(result));
    }

    worker_thread_.join();

    lock.lock();
    worker_state_ = WorkerStatus::Disconnected;
    lock.unlock();

    result_condition_.notify_all();

    RCLCPP_INFO(workerLogger(), "Worker arrestato e disconnesso");

  }

}

bool OpcuaWorker::enqueueCommand(FrankaRobotCommand command)
{
  const auto command_id = command.command_id;
  std::lock_guard<std::mutex> lock(mutex_);
  if (worker_state_ != WorkerStatus::Ready) {
    RCLCPP_WARN(
      workerLogger(), "Comando %lu rifiutato: worker non pronto",
      static_cast<unsigned long>(command_id));
    return false;
  }

  if (stop_pending_) {
    RCLCPP_WARN(
      workerLogger(), "Comando %lu rifiutato: stop in corso",
      static_cast<unsigned long>(command_id));
    return false;
  }

  const bool accepted = command_manager_.enqueue(std::move(command));
  if (!accepted) {
    RCLCPP_WARN(
      workerLogger(), "Comando %lu rifiutato: coda piena o in shutdown",
      static_cast<unsigned long>(command_id));
  }
  return accepted;

}

bool OpcuaWorker::enqueuePriority(FrankaRobotCommand command)
{
  const auto command_id = command.command_id;
  const bool is_stop = command.type == FrankaCommandType::Stop;
  std::vector<FrankaRobotCommand> cancelled_commands;
  bool accepted = false;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (worker_state_ != WorkerStatus::Ready) {
      RCLCPP_WARN(
        workerLogger(), "Comando prioritario %lu rifiutato: worker non pronto",
        static_cast<unsigned long>(command_id));
      return false;
    }

    if (is_stop && stop_pending_) {
      RCLCPP_WARN(
        workerLogger(), "Comando stop %lu rifiutato: uno stop e' gia' pendente",
        static_cast<unsigned long>(command_id));
      return false;
    }

    if (is_stop) {
      auto cancelled = command_manager_.enqueueStop(std::move(command));
      accepted = cancelled.has_value();
      if (accepted) {
        cancelled_commands = std::move(*cancelled);
        stop_pending_ = true;
        stop_not_needed_ = false;
        abort_requested_.store(true);
      }
    } else {
      accepted = command_manager_.enqueuePriority(std::move(command));
    }
  }

  for (const auto & cancelled_command : cancelled_commands) {
    FrankaCommandResult result;
    result.command_id = cancelled_command.command_id;
    result.status = FrankaCommandStatus::Cancelled;
    result.message = "Comando annullato da STOP";
    result.finished_at = std::chrono::steady_clock::now();
    populateCommandTimings(cancelled_command, result, result.finished_at);
    recordCommandTimings(performance_monitor_, result);
    pushResult(std::move(result));
  }

  if (!accepted) {
    RCLCPP_WARN(
      workerLogger(), "Comando prioritario %lu rifiutato: coda piena o in shutdown",
      static_cast<unsigned long>(command_id));
  }
  return accepted;
}


void OpcuaWorker::run()
{

  RCLCPP_INFO(workerLogger(), "Connessione al robot OPC UA in corso");

  // Connessione al robot
  if (!franka_->connect()) {
    RCLCPP_ERROR(workerLogger(), "Connessione al robot OPC UA fallita");
    std::unique_lock<std::mutex> lock(mutex_);
    if (worker_state_ != WorkerStatus::Stopping) {
      worker_state_ = WorkerStatus::Error;
    }
    lock.unlock();

    state_condition_.notify_one();
    result_condition_.notify_all();
    return;
  }

  bool stopping = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping = worker_state_ == WorkerStatus::Stopping;
  }

  if (stopping) {

    RCLCPP_WARN(workerLogger(), "Arresto richiesto durante la connessione");
    franka_->disconnect();

    std::lock_guard<std::mutex> lock(mutex_);
    worker_state_ = WorkerStatus::Disconnected;
    state_condition_.notify_all();
    result_condition_.notify_all();
    return;
  }

  if (!franka_->requestControl(true)) {

    RCLCPP_ERROR(workerLogger(), "Acquisizione del control token fallita");

    franka_->disconnect();

    std::unique_lock<std::mutex> lock(mutex_);
    if (worker_state_ != WorkerStatus::Stopping) {
      worker_state_ = WorkerStatus::Error;
    }
    lock.unlock();

    state_condition_.notify_one();
    result_condition_.notify_all();
    return;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  if (worker_state_ == WorkerStatus::Stopping) {
    RCLCPP_WARN(workerLogger(), "Arresto richiesto durante l'acquisizione del control token");
    lock.unlock();

    franka_->releaseControl();
    franka_->disconnect();

    lock.lock();
    worker_state_ = WorkerStatus::Disconnected;
    lock.unlock();

    state_condition_.notify_all();
    result_condition_.notify_all();
    return;
  }

  worker_state_ = WorkerStatus::Ready;
  lock.unlock();
  state_condition_.notify_one();

  RCLCPP_INFO(workerLogger(), "Control token acquisito: worker pronto a eseguire comandi");


  while (true) {
    auto command = command_manager_.waitAndPop();

    if (!command) {
      RCLCPP_INFO(workerLogger(), "Shutdown della coda ricevuto");
      break;       //shutdown
    }

    const auto execution_started_at = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      workerLogger(), "Esecuzione comando %lu, tipo %d, parametro '%s'",
      static_cast<unsigned long>(command->command_id),
      static_cast<int>(command->type),
      command->parameter.c_str());

    auto result = executeCommand(*command);
    populateCommandTimings(*command, result, execution_started_at);
    recordCommandTimings(performance_monitor_, result);
    RCLCPP_INFO(
      workerLogger(), "Comando %lu terminato con stato %d: %s",
      static_cast<unsigned long>(result.command_id),
      static_cast<int>(result.status),
      result.message.c_str());
    pushResult(std::move(result));
  }

  RCLCPP_INFO(workerLogger(), "Rilascio control token e disconnessione dal robot");

  auto robotStat = robotStatus();

  // A failed status read is unknown, not proof that the robot is idle; try to stop conservatively.
  if (robotStat == nullptr || robotStat->is_running || robotStat->has_error) {
    bool success = franka_->stop();
    if (success) {
      RCLCPP_INFO(workerLogger(), "Task robot arrestato durante lo shutdown");
    } else {
      RCLCPP_ERROR(workerLogger(), "Impossibile arrestare il task robot durante lo shutdown");
    }
  }

  franka_->releaseControl();
  franka_->disconnect();

  lock.lock();
  worker_state_ = WorkerStatus::Disconnected;
  lock.unlock();

  state_condition_.notify_all();
  result_condition_.notify_all();
}


FrankaCommandResult OpcuaWorker::executeCommand(const FrankaRobotCommand & command)
{

  RCLCPP_DEBUG(
    workerLogger(), "Dispatch comando %lu",
    static_cast<unsigned long>(command.command_id));

  FrankaCommandResult result;
  result.command_id = command.command_id;
  result.status = FrankaCommandStatus::Rejected;

  bool success = false;
  std::string message;

  switch (command.type) {

    case FrankaCommandType::OpenBrakes:
      success = franka_->openBrakes();
      message = success ? "Brakes opened" : "Failed to open brakes";
      break;

    case FrankaCommandType::CloseBrakes:
      success = franka_->closeBrakes();
      message = success ? "Brakes closed" : "Failed to close brakes";
      break;

    case FrankaCommandType::Stop: {
        bool stop_needed = true;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stop_needed = !stop_not_needed_;
          if (!stop_needed) {
            stop_not_needed_ = false;
            stop_pending_ = false;
            abort_requested_.store(false);
          } else {
            abort_requested_.store(false);
          }
        }
        if (!stop_needed) {
          result.status = FrankaCommandStatus::NotNeeded;
          result.message = "Stop not needed: motion had already terminated";
          result.finished_at = std::chrono::steady_clock::now();
          return result;
        }
        RCLCPP_INFO(
          workerLogger(), "Comando stop %lu: invocazione di StopTask",
          static_cast<unsigned long>(command.command_id));
        success = franka_->stop();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stop_pending_ = false;
        }
        message = success ? "Robot stopped" : "Failed to stop robot";
        break;
      }

    case FrankaCommandType::ExecuteNamedTask:
      success = franka_->executeNamedTask(command.parameter);
      message = success ? "Task executed" : "Failed to execute task";
      break;

    case FrankaCommandType::MoveToNamedPose: {
        const MotionResult motion = franka_->moveToNamedPose(
          command.parameter,
          [this]() {return abort_requested_.load();});
        if (motion.outcome != MotionOutcome::Cancelled) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (stop_pending_ && abort_requested_.load()) {
            stop_not_needed_ = true;
          }
        }
        result.message = motion.message;
        switch (motion.outcome) {
          case MotionOutcome::Completed:
            result.status = FrankaCommandStatus::Completed;
            break;
          case MotionOutcome::Cancelled:
            result.status = FrankaCommandStatus::Cancelled;
            break;
          case MotionOutcome::Failed:
            result.status = FrankaCommandStatus::Failed;
            break;
          case MotionOutcome::Timeout:
            result.status = FrankaCommandStatus::Timeout;
            break;
        }
        result.finished_at = std::chrono::steady_clock::now();
        if (result.status == FrankaCommandStatus::Failed ||
          result.status == FrankaCommandStatus::Timeout)
        {
          RCLCPP_ERROR(
            workerLogger(), "Comando movimento %lu terminato con stato %d: %s",
            static_cast<unsigned long>(command.command_id),
            static_cast<int>(result.status), result.message.c_str());
        }
        return result;
      }

    case FrankaCommandType::SaveCurrentPose:
      success = pose_storage_ != nullptr && pose_storage_->savePose(command.parameter);
      message = success ? "Pose saved" : "Failed to save pose";
      break;

    case FrankaCommandType::SaveExplicitPose:
      success = pose_storage_ != nullptr && command.pose.has_value() &&
        pose_storage_->savePose(command.parameter, command.pose.value());
      message = success ? "Pose saved" : "Failed to save pose";
      break;

    case FrankaCommandType::ActivateFci:
      success = franka_->activateFCI();
      message = success ? "FCI activated" : "Failed to activate FCI";
      break;

    case FrankaCommandType::DeactivateFci:
      success = franka_->deactivateFCI();
      message = success ? "FCI deactivated" : "Failed to deactivate FCI";
      break;

    case FrankaCommandType::SwitchToExec:
      success = task_control_ != nullptr && task_control_->switchToExecution();
      message = success ? "Switch to Execution mode completed" :
        "Failed to switch to Execution mode";
      break;

    case FrankaCommandType::ShowAddressSpaceNodes: {
        auto rootNode = franka_->showAddressSpaceNodes();

        if (!rootNode) {
          success = false;
          message = "Address space nodes tree not loaded, something went wrong";

        } else {
          success = true;
          message = "Addrress space correctly explored";
          printTree(*rootNode);
        }
        break;
      }
    case FrankaCommandType::ShowTasks: {
        if (task_control_ != nullptr) {
          const std::vector<std::string> tasks_id = task_control_->showAvailableTasks();

          RCLCPP_INFO(workerLogger(), "Task disponibili: %zu", tasks_id.size());

          for (const auto & task_id : tasks_id) {
            RCLCPP_INFO(workerLogger(), "ID: %s", task_id.c_str());
          }

          result.task_ids = tasks_id;
          success = true;
        }
        message = success ? "Task list retrieved" : "Task control is unavailable";
        break;
      }

    default:
      result.status = FrankaCommandStatus::Rejected;
      result.finished_at = std::chrono::steady_clock::now();
      result.message = "Unknown command type";
      return result;
  }

  result.status = success ? FrankaCommandStatus::Completed : FrankaCommandStatus::Failed;
  result.finished_at = std::chrono::steady_clock::now();
  result.message = message;

  if (!success) {
    RCLCPP_ERROR(
      workerLogger(), "Comando %lu fallito: %s",
      static_cast<unsigned long>(command.command_id), message.c_str());
  }

  return result;

}


void OpcuaWorker::pushResult(FrankaCommandResult result)
{

  std::lock_guard<std::mutex> lock(mutex_);

  result_queue_.push(std::move(result));

  result_condition_.notify_one();
}

std::optional<FrankaCommandResult> OpcuaWorker::tryGetResult()
{

  std::lock_guard<std::mutex> lock(mutex_);

  if (result_queue_.empty()) {return std::nullopt;}

  FrankaCommandResult result = std::move(result_queue_.front());

  result_queue_.pop();

  return result;
}

std::optional<FrankaCommandResult> OpcuaWorker::waitAndPopResult()
{

  std::unique_lock<std::mutex> lock(mutex_);

  result_condition_.wait(
    lock, [this] {
      return !result_queue_.empty() || worker_state_ == WorkerStatus::Disconnected ||
      worker_state_ == WorkerStatus::Error;
    });

  if (result_queue_.empty()) {return std::nullopt;}

  FrankaCommandResult result = std::move(result_queue_.front());

  result_queue_.pop();

  return result;

}


WorkerStatus OpcuaWorker::status() const
{

  std::lock_guard<std::mutex> lock(mutex_);

  WorkerStatus state = worker_state_;

  return state;


}

std::unique_ptr<RobotStatus> OpcuaWorker::robotStatus()
{

  auto robotStatus = franka_->readStatus();

  return robotStatus;

}

void OpcuaWorker::printTree(const namespaceTree & node, std::size_t depth)
{
  std::cout << std::string(depth * 2, ' ')
            << node.browseName << " (" << node.displayName << ")\n";

  for (const namespaceTree & child : node.children) {
    printTree(child, depth + 1);
  }
}


}
