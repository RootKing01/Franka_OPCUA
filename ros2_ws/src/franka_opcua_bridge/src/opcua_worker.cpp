#include "franka_opcua_bridge/opcua_worker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <thread>

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
    lock,
    [this] {
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
  }

  // Non mantenere mutex_ durante join():
  // il thread worker potrebbe averne bisogno per terminare.
  lock.unlock();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
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
    RCLCPP_WARN(
      workerLogger(),
      "Arresto richiesto durante l'acquisizione del control token");

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

  RCLCPP_INFO(
    workerLogger(),
    "Control token acquisito: worker pronto a eseguire comandi");

  // --------------------------------------------------
  // Ciclo principale del worker
  // --------------------------------------------------

  while (true) {

    if (command_manager_.isShutdown()) {
      RCLCPP_INFO(workerLogger(), "Shutdown della coda ricevuto");
      break;
    }

    // ------------------------------------------------
    // Task complessa attiva: monitoraggio e STOP
    // ------------------------------------------------

    if (active_task_) {

      // Durante una task complessa preleviamo
      // esclusivamente i comandi prioritari.
      auto command = command_manager_.tryPopPriority();

      if (command) {

        const auto execution_started_at = std::chrono::steady_clock::now();

        if (command->type == FrankaCommandType::Stop) {

          RCLCPP_INFO(
            workerLogger(),
            "STOP ricevuto durante la task %s",
            active_task_->command.parameter.c_str());

          auto result = executeCommand(*command);

          populateCommandTimings(
            *command, result, execution_started_at);

          recordCommandTimings(performance_monitor_, result);

          const bool stop_success =
            result.status == FrankaCommandStatus::Completed;

          pushResult(std::move(result));

          if (stop_success && active_task_) {
            completeActiveTask(
              FrankaCommandStatus::Cancelled,
              "Task cancelled by STOP");
          }

        } else {

          FrankaCommandResult result;
          result.command_id = command->command_id;
          result.status = FrankaCommandStatus::Rejected;
          result.message =
            "Only STOP is allowed while a task is running";
          result.finished_at = std::chrono::steady_clock::now();

          populateCommandTimings(
            *command, result, execution_started_at);

          recordCommandTimings(performance_monitor_, result);
          pushResult(std::move(result));
        }

      } else {
        // Nessun comando prioritario: monitoriamo la task.
        processActiveTask();
      }

      std::this_thread::sleep_for(
        std::chrono::milliseconds(100));

      continue;
    }

    // ------------------------------------------------
    // Nessuna task complessa attiva
    // ------------------------------------------------

    auto command = command_manager_.waitAndPop();

    if (!command) {
      RCLCPP_INFO(workerLogger(), "Shutdown della coda ricevuto");
      break;
    }

    const auto execution_started_at = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      workerLogger(),
      "Esecuzione comando %lu, tipo %d, parametro '%s'",
      static_cast<unsigned long>(command->command_id),
      static_cast<int>(command->type),
      command->parameter.c_str());

    
    // Le task complesse vengono avviate in modalità asincrona.
    // Il worker continuerà a monitorarle nelle iterazioni successive.
    if (command->type == FrankaCommandType::ExecuteComplexTask) {
        startActiveTask(*command);
        continue;
      }

    // Gli altri comandi mantengono il comportamento precedente.
    // ExecuteNamedTask resta bloccante.

    auto result = executeCommand(*command);

    populateCommandTimings(
      *command, result, execution_started_at);

    recordCommandTimings(performance_monitor_, result);

    RCLCPP_INFO(
      workerLogger(),
      "Comando %lu terminato con stato %d: %s",
      static_cast<unsigned long>(result.command_id),
      static_cast<int>(result.status),
      result.message.c_str());

    pushResult(std::move(result));
  }

  // --------------------------------------------------
  // Shutdown
  // --------------------------------------------------

  RCLCPP_INFO(
    workerLogger(),
    "Rilascio control token e disconnessione dal robot");

  bool stop_attempted = false;

  if (active_task_) {

    auto robotStat = robotStatus();

    if (robotStat &&
        !robotStat->has_error &&
        !robotStat->is_running &&
        active_task_->execution_observed)
    {
      completeActiveTask(
        FrankaCommandStatus::Completed,
        "Task completed before worker shutdown");
    } else {

      const bool success = franka_->stop();
      stop_attempted = true;

      completeActiveTask(
        success ? FrankaCommandStatus::Cancelled :
                  FrankaCommandStatus::Failed,
        success ? "Task cancelled during worker shutdown" :
                  "Failed to stop task during worker shutdown");
    }
  }

  if (!stop_attempted) {

    auto robotStat = robotStatus();

    // Una lettura fallita non dimostra che il robot sia idle.
    if (robotStat == nullptr ||
        robotStat->is_running ||
        robotStat->has_error)
    {
      const bool success = franka_->stop();

      if (success) {
        RCLCPP_INFO(
          workerLogger(),
          "Task robot arrestato durante lo shutdown");
      } else {
        RCLCPP_ERROR(
          workerLogger(),
          "Impossibile arrestare il task robot durante lo shutdown");
      }
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

    case FrankaCommandType::ExecuteNamedTask: {
        const TaskExecutionResult task_result = franka_->executeNamedTask(
          command.parameter,
          [this]() {return abort_requested_.load();});
        if (task_result.state == TaskState::Completed) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (stop_pending_ && abort_requested_.load()) {
            stop_not_needed_ = true;
          }
        }
        result.message = task_result.message;
        switch (task_result.state) {
          case TaskState::Completed:
            result.status = FrankaCommandStatus::Completed;
            break;
          case TaskState::Cancelled:
            result.status = FrankaCommandStatus::Cancelled;
            break;
          case TaskState::Timeout:
            result.status = FrankaCommandStatus::Timeout;
            break;
          case TaskState::Failed:
          case TaskState::Unknown:
          case TaskState::Idle:
          case TaskState::Starting:
          case TaskState::Running:
          case TaskState::Stopping:
          default:
            result.status = FrankaCommandStatus::Failed;
            break;
        }
        result.finished_at = std::chrono::steady_clock::now();
        return result;
      }

    case FrankaCommandType::MoveToNamedPose: {
        const MotionResult motion = franka_->moveToNamedPose(
          command.task_name,
          command.parameter,
          command.task_poses,
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

bool OpcuaWorker::startActiveTask(const FrankaRobotCommand & command)
{
  if (active_task_.has_value()) {
    RCLCPP_WARN(workerLogger(), "Impossibile avviare una nuova task: una task e' gia' attiva");
    return false;
  }

  const auto started_at = std::chrono::steady_clock::now();

  if (!franka_->setIntegerVariable("decision_code", 0) ||
    !franka_->setIntegerVariable("process_state", 0))
  {
    FrankaCommandResult result;
    result.command_id = command.command_id;
    result.status = FrankaCommandStatus::Failed;
    result.message = "Failed to reset decision variables before starting task";
    result.finished_at = std::chrono::steady_clock::now();
    populateCommandTimings(command, result, started_at);
    recordCommandTimings(performance_monitor_, result);
    pushResult(std::move(result));
    return false;
  }

  const TaskExecutionResult task_result =
    franka_->startNamedTask(command.parameter);

  if (task_result.state != TaskState::Starting) {
    FrankaCommandResult result;
    result.command_id = command.command_id;
    result.status = FrankaCommandStatus::Failed;
    result.message = task_result.message;
    result.finished_at = std::chrono::steady_clock::now();

    populateCommandTimings(command, result, started_at);
    recordCommandTimings(performance_monitor_, result);
    pushResult(std::move(result));

    return false;
  }

  ActiveTask task;
  task.command = command;
  task.state = TaskState::Starting;
  task.phase = TaskPhase::Monitoring;
  task.started_at = started_at;
  task.execution_observed = false;

  active_task_ = std::move(task);

  RCLCPP_INFO(
    workerLogger(),
    "Task %s avviata, command_id=%lu",
    command.parameter.c_str(),
    static_cast<unsigned long>(command.command_id));

  return true;
}

void OpcuaWorker::monitorActiveTask()
{
  if (!active_task_) {
    return;
  }

  constexpr auto kExecutionTimeout = std::chrono::seconds(60);
  if (std::chrono::steady_clock::now() - active_task_->started_at >= kExecutionTimeout) {
    const bool stopped = franka_->stop();
    completeActiveTask(
      FrankaCommandStatus::Timeout,
      stopped ? "Timed out waiting for complex task completion; stop requested" :
      "Timed out waiting for complex task completion; stop failed");
    return;
  }

  auto status = franka_->readStatus();

  // Non siamo riusciti a leggere lo stato: non possiamo sapere
  // se il robot sia ancora occupato.
  if (!status) {
    RCLCPP_WARN_THROTTLE(
      workerLogger(),
      *rclcpp::Clock::make_shared(),
      5000,
      "Unable to read robot status while monitoring active task");

    return;
  }

  if (status->has_error) {
    completeActiveTask(
      FrankaCommandStatus::Failed,
      status->error_message.empty() ? "Robot reported an error during complex task" :
      status->error_message);
    return;
  }

  const std::string & expected_task_id = active_task_->command.parameter;

  // Il robot è in esecuzione.
  if (status->is_running) {

    // L'ID potrebbe non essere ancora disponibile durante
    // la fase iniziale di avvio.
    if (status->active_task_id.empty()) {
      return;
    }

    // Non è la task che abbiamo richiesto.
    // Non possiamo considerare conclusa la nostra task né
    // permettere al worker di avviare altri comandi.
    if (status->active_task_id != expected_task_id) {
      RCLCPP_ERROR_THROTTLE(
        workerLogger(),
        *rclcpp::Clock::make_shared(),
        5000,
        "Unexpected active task. Expected '%s', found '%s'",
        expected_task_id.c_str(),
        status->active_task_id.c_str());

      return;
    }

    active_task_->execution_observed = true;
    active_task_->state = TaskState::Running;

    return;
  }

  // Il robot non è in esecuzione.
  // Possiamo dichiarare completata la task soltanto se prima
  // abbiamo effettivamente osservato la sua esecuzione.
  if (active_task_->execution_observed) {
    completeActiveTask(
      FrankaCommandStatus::Completed,
      "Task completed successfully");
  }
}


void OpcuaWorker::completeActiveTask(
  FrankaCommandStatus status,
  const std::string & message)
{
  if (!active_task_) {
    return;
  }

  FrankaCommandResult result;
  result.command_id = active_task_->command.command_id;
  result.status = status;
  result.message = message;
  result.finished_at = std::chrono::steady_clock::now();

  populateCommandTimings(
    active_task_->command,
    result,
    active_task_->started_at);

  recordCommandTimings(performance_monitor_, result);

  RCLCPP_INFO(
    workerLogger(),
    "Task %s terminata con stato %d: %s",
    active_task_->command.parameter.c_str(),
    static_cast<int>(status),
    message.c_str());

  {
    std::lock_guard<std::mutex> lock(decision_mutex_);
    decision_queue_ = std::queue<FrankaDecisionRequest>{};
    outstanding_decision_.reset();
    pending_decision_.reset();
  }

  active_task_.reset();

  pushResult(std::move(result));
}

std::optional<FrankaDecisionRequest> OpcuaWorker::tryGetDecisionRequest()
{
  std::lock_guard<std::mutex> lock(decision_mutex_);

  if (decision_queue_.empty()) {
    return std::nullopt;
  }

  auto request = std::move(decision_queue_.front());
  decision_queue_.pop();

  return request;
}


bool OpcuaWorker::submitDecision(const FrankaDecisionResponse& response)
{
  std::lock_guard<std::mutex> lock(decision_mutex_);

  // Deve esserci una richiesta in attesa di risposta.
  if (!outstanding_decision_) {
    return false;
  }

  // La risposta deve riferirsi alla richiesta corretta.
  if (response.request_id != outstanding_decision_->request_id ||
      response.decision_id != outstanding_decision_->decision_id) {
    return false;
  }

  // Il codice scelto deve essere tra quelli proposti.
  const auto& options = outstanding_decision_->option_codes;

  if (std::find(options.begin(), options.end(), response.decision_code) ==
      options.end()) {
    return false;
  }

  // Non accettiamo una seconda risposta prima di aver gestito la prima.
  if (pending_decision_) {
    return false;
  }

  pending_decision_ = response;
  return true;
}

bool OpcuaWorker::pushDecisionRequest(FrankaDecisionRequest request)
{
  std::lock_guard<std::mutex> lock(decision_mutex_);

  // Non permettiamo più di una richiesta in attesa.
  if (outstanding_decision_) {
    return false;
  }

  outstanding_decision_ = request;
  decision_queue_.push(std::move(request));

  return true;
}

std::optional<FrankaDecisionResponse> OpcuaWorker::takePendingDecision()
{
  std::lock_guard<std::mutex> lock(decision_mutex_);

  return pending_decision_;
}

bool OpcuaWorker::completeDecision(
  uint64_t request_id,
  uint64_t decision_id)
{
  std::lock_guard<std::mutex> lock(decision_mutex_);

  if (!outstanding_decision_) {
    return false;
  }

  if (outstanding_decision_->request_id != request_id ||
      outstanding_decision_->decision_id != decision_id) {
    return false;
  }

  outstanding_decision_.reset();
  pending_decision_.reset();

  return true;
}


void OpcuaWorker::processActiveTask()
{
  if (!active_task_) {
    return;
  }

  constexpr char PROCESS_STATE_KEY[] = "process_state";
  constexpr char DECISION_CODE_KEY[] = "decision_code";

  constexpr int32_t WAITING_DECISION_STATE = 1;

  // 1. Monitoraggio dell'esecuzione della task.
  monitorActiveTask();

  if (!active_task_) {
    return;
  }

  // 2. Gestione di una risposta già ricevuta.
  if (active_task_->phase == TaskPhase::WaitingDecision) {
    auto response = takePendingDecision();

    if (!response) {
      return;
    }

    active_task_->phase = TaskPhase::ApplyingDecision;
  }

  // 3. Applicazione della decisione sul server OPC UA.
  if (active_task_->phase == TaskPhase::ApplyingDecision) {
    auto response = takePendingDecision();

    if (!response) {
      return;
    }

    if (!franka_->setIntegerVariable(
        DECISION_CODE_KEY,
        response->decision_code)) {
      return;
    }

    if (!completeDecision(
        response->request_id,
        response->decision_id)) {
      return;
    }

    active_task_->phase = TaskPhase::Monitoring;
    return;
  }

  // 4. Se non siamo in fase di monitoraggio, non procediamo.
  if (active_task_->phase != TaskPhase::Monitoring) {
    return;
  }

  // 5. Lettura dello stato del processo.
  int32_t process_state = 0;

  if (!franka_->readIntegerVariable(PROCESS_STATE_KEY, process_state)) {
    return;
  }

  if (process_state != WAITING_DECISION_STATE) {
    active_task_->decision_requested_for_checkpoint = false;
    return;
  }

  if (active_task_->decision_requested_for_checkpoint) {
    return;
  }

  if (!franka_->setIntegerVariable(DECISION_CODE_KEY, 0)) {
    return;
  }

  // 6. Preparazione della richiesta di decisione.
  FrankaDecisionRequest request;
  request.request_id = active_task_->command.command_id;
  request.decision_id = active_task_->next_decision_id;
  request.description = "Decisione richiesta durante l'esecuzione della task.";
  request.option_codes = {1, 2};
  request.option_labels = {
    "Opzione A",
    "Opzione B"
  };

  // 7. Inserimento della richiesta nella coda.
  if (!pushDecisionRequest(std::move(request))) {
    return;
  }

  // 8. Attesa della risposta dell'utente.
  ++active_task_->next_decision_id;
  active_task_->decision_requested_for_checkpoint = true;
  active_task_->phase = TaskPhase::WaitingDecision;
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
