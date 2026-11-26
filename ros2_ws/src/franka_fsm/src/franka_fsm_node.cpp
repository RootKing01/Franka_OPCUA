#include "franka_fsm/franka_fsm_node.hpp"

#include <array>
#include <algorithm>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <unistd.h>

namespace franka_fsm
{


FrankaFsmNode::FrankaFsmNode()
: Node("franka_fsm")
{
  RCLCPP_INFO(get_logger(), "Nodo FSM avviato.");

  command_publisher_ = create_publisher<franka_interfaces::msg::BridgeCommand>(
    "/franka_right/command", 10);

  result_subscription_ = create_subscription<franka_interfaces::msg::BridgeResult>(
    "/franka_right/command_result",
    10,
    [this](franka_interfaces::msg::BridgeResult::ConstSharedPtr msg) {
      RCLCPP_INFO(
        get_logger(),
        "Risultato ricevuto: request_id=%lu status=%u task_count=%zu message='%s'",
        static_cast<unsigned long>(msg->request_id),
        static_cast<unsigned int>(msg->status),
        msg->task_ids.size(),
        msg->message.c_str());

      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto pending = pending_commands_.find(msg->request_id);
      if (pending == pending_commands_.end()) {
        RCLCPP_DEBUG(
          get_logger(),
          "Risposta non associata a una richiesta FSM: request_id=%lu",
          static_cast<unsigned long>(msg->request_id));
        return;
      }

      if (msg->status == franka_interfaces::msg::BridgeResult::STATUS_ACCEPTED) {
        RCLCPP_DEBUG(
          get_logger(),
          "Richiesta request_id=%lu accettata; attendo esito terminale",
          static_cast<unsigned long>(msg->request_id));
        return;
      }

      const CommandType command = pending->second;
      pending_commands_.erase(pending);
      if (command == CommandType::ExecuteComplexTask) {
        std::queue<franka_interfaces::msg::DecisionRequest> remaining_decisions;
        while (!pending_decisions_.empty()) {
          auto request = std::move(pending_decisions_.front());
          pending_decisions_.pop();
          if (request.request_id != msg->request_id) {
            remaining_decisions.push(std::move(request));
          }
        }
        pending_decisions_ = std::move(remaining_decisions);
      }

      switch (msg->status) {
        case franka_interfaces::msg::BridgeResult::STATUS_COMPLETED:
          if (command == CommandType::ShowTasks) {
            available_task_ids_ = msg->task_ids;
            RCLCPP_INFO(
              get_logger(),
              "Lista task aggiornata: %zu task ricevute",
              available_task_ids_.size());
            for (const auto & task_id : available_task_ids_) {
              RCLCPP_INFO(get_logger(), "Task disponibile: %s", task_id.c_str());
            }
          }
          if (command == CommandType::ExecuteTask ||
            command == CommandType::ExecuteComplexTask)
          {
            task_active_ = false;
            state_ = FsmState::Completed;
            RCLCPP_INFO(
              get_logger(),
              "Task OPC UA completata");
          } else if (command == CommandType::MoveToNamedPose) {
            task_active_ = false;
            state_ = FsmState::Completed;
          } else if (command == CommandType::Stop) {
            task_active_ = false;
            state_ = FsmState::Interrupted;
            RCLCPP_INFO(get_logger(), "Task fermata; menu comandi riabilitato");
          } else {
            state_ = FsmState::Idle;
          }
          break;
        case franka_interfaces::msg::BridgeResult::STATUS_CANCELLED:
          task_active_ = false;
          state_ = FsmState::Interrupted;
          break;
        case franka_interfaces::msg::BridgeResult::STATUS_NOT_NEEDED:
          task_active_ = false;
          state_ = FsmState::Idle;
          break;
        case franka_interfaces::msg::BridgeResult::STATUS_REJECTED:
        case franka_interfaces::msg::BridgeResult::STATUS_FAILED:
        case franka_interfaces::msg::BridgeResult::STATUS_TIMEOUT:
        case franka_interfaces::msg::BridgeResult::STATUS_NOT_READY:
        case franka_interfaces::msg::BridgeResult::STATUS_DISCONNECTED:
        case franka_interfaces::msg::BridgeResult::STATUS_UNKNOWN:
        default:
          if (command == CommandType::ExecuteTask ||
          command == CommandType::ExecuteComplexTask ||
          command == CommandType::MoveToNamedPose)
          {
            task_active_ = false;
          }
          state_ = FsmState::Error;
          break;
      }

      RCLCPP_INFO(
        get_logger(),
        "Stato FSM aggiornato: %u",
        static_cast<unsigned int>(state_));
      state_condition_.notify_all();
    });

  decision_response_ = create_publisher<franka_interfaces::msg::DecisionResponse>(
    "/franka_right/decision_response", 10);

  decision_request_ = create_subscription<franka_interfaces::msg::DecisionRequest>(
    "/franka_right/decision_request",
    10,
    [this](franka_interfaces::msg::DecisionRequest::ConstSharedPtr msg) {
    RCLCPP_INFO(
    get_logger(),
    "[FSM] DecisionRequest ricevuta: request_id=%lu decision_id=%lu",
    static_cast<unsigned long>(msg->request_id),
    static_cast<unsigned long>(msg->decision_id));

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    const auto command = pending_commands_.find(msg->request_id);

    if (command == pending_commands_.end()) {
      RCLCPP_WARN(
        get_logger(),
        "[FSM] DecisionRequest ignorata: request_id=%lu non presente tra i comandi pendenti",
        static_cast<unsigned long>(msg->request_id));
      return;
    }

    if (command->second != CommandType::ExecuteComplexTask) {
      RCLCPP_WARN(
        get_logger(),
        "[FSM] DecisionRequest ignorata: request_id=%lu non associato a ExecuteComplexTask",
        static_cast<unsigned long>(msg->request_id));
      return;
    }

    pending_decisions_.push(*msg);

    RCLCPP_INFO(
      get_logger(),
      "[FSM] DecisionRequest accodata: request_id=%lu decision_id=%lu",
      static_cast<unsigned long>(msg->request_id),
      static_cast<unsigned long>(msg->decision_id));
  }

  state_condition_.notify_all();
  });
}


void FrankaFsmNode::runInteractiveMenu()
{
  std::string choice;
  while (rclcpp::ok()) {
    bool task_active = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_active = task_active_;
    }

    showMainMenu(task_active);

    if (!std::getline(std::cin, choice)) {
      break;
    }

    if (choice == "1") {
      showAvailableCommands();
      continue;
    }

    if (choice == "4") {
      answerPendingDecision();
      continue;
    }

    if (choice == "3") {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (task_active_) {
        std::cout << "Task ancora attiva: invia STOP prima di terminare.\n";
        continue;
      }
      break;
    }

    if (choice != "2") {
      std::cout << "Scelta non valida.\n";
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_active = task_active_;
    }
    showCommandMenu(task_active);

    auto request = readCommandRequest(task_active);
    if (!request.has_value()) {
      if (std::cin.eof()) {
        break;
      }
      continue;
    }

    switch (request->command) {
      case CommandType::Stop:
        sendCommand(
          request->command, request->task_name, request->parameter,
          request->single_operation_pose, request->pose);
        continue;
      default:
        if (checkFullQueue()) {
          std::cout << "Bridge's command queue is full. Wait a moment to send another command.\n ";
          continue;
        }

        const auto request_id = sendCommand(
          request->command, request->task_name, request->parameter,
          request->single_operation_pose, request->pose);
        if (request->command == CommandType::ExecuteComplexTask && request_id) {
          waitForComplexTask(*request_id);
        }
        continue;
    }


  }
}

bool FrankaFsmNode::checkFullQueue()
{

  bool full = false;
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (pending_commands_.size() >= 4) {full = true;}

  return full;


}


void FrankaFsmNode::showMainMenu(bool task_active) const
{
  std::cout << "\n=== Franka FSM ===\n"
            << "1. Mostra comandi\n"
            << "2. Invia comando\n"
            << "3. Termina\n"
            << "4. Rispondi a una decisione\n";


  if (task_active) {
    std::cout <<
      "Task attiva: invia STOP per troncare il task, invia comandi successivi (massimo 5)  o uscire.\n";
  }
  std::cout << "> " << std::flush;
}


void FrankaFsmNode::showAvailableCommands() const
{
  std::cout << "Comandi disponibili:\n"
            << " 1. Mostra task OPC UA\n"
            << " 2. Avvia task OPC UA per ID\n"
            << " 3. STOP\n"
            << " 4. Apri freni\n"
            << " 5. Chiudi freni\n"
            << " 6. Attiva FCI\n"
            << " 7. Disattiva FCI\n"
            << " 8. Passa a Execution mode\n"
            << " 9. Mostra address space\n"
            << "10. Muovi a posa nominata\n"
            << "11. Salva posa corrente\n"
            << "12. Salva posa esplicita\n"
            << "13. Avvia task complessa\n";
}


void FrankaFsmNode::showCommandMenu(bool task_active) const
{


  std::cout << " 1. Mostra task OPC UA\n"
            << " 2. Avvia task OPC UA\n"
            << " 3. STOP\n"
            << " 4. Apri freni\n"
            << " 5. Chiudi freni\n"
            << " 6. Attiva FCI\n"
            << " 7. Disattiva FCI\n"
            << " 8. Passa a Execution mode\n"
            << " 9. Mostra address space\n"
            << "10. Muovi a posa nominata\n"
            << "11. Salva posa corrente\n"
            << "12. Salva posa esplicita\n"
            << "13. Avvia task complessa\n"
            << " 0. Indietro\n> " << std::flush;
}


std::optional<FrankaFsmNode::CommandRequest> FrankaFsmNode::readCommandRequest(
  bool task_active)
{
  std::string choice;
  if (!std::getline(std::cin, choice)) {
    return std::nullopt;
  }

  if (choice == "0") {
    return std::nullopt;
  }

  CommandRequest request{CommandType::ShowTasks, "", "", {}, std::nullopt};
  if (choice == "1") {
    request.command = CommandType::ShowTasks;
  } else if (choice == "2") {
    request.command = CommandType::ExecuteTask;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!available_task_ids_.empty()) {
        std::cout << "Task note:\n";
        for (const auto & task_id : available_task_ids_) {
          std::cout << "  " << task_id << '\n';
        }
      }
    }
    std::cout << "ID task: " << std::flush;
    if (!std::getline(std::cin, request.parameter)) {
      return std::nullopt;
    }
  } else if (choice == "3") {
    request.command = CommandType::Stop;
  } else if (choice == "4") {
    request.command = CommandType::OpenBrakes;
  } else if (choice == "5") {
    request.command = CommandType::CloseBrakes;
  } else if (choice == "6") {
    request.command = CommandType::ActivateFci;
  } else if (choice == "7") {
    request.command = CommandType::DeactivateFci;
  } else if (choice == "8") {
    request.command = CommandType::SwitchToExec;
  } else if (choice == "9") {
    request.command = CommandType::ShowAddressSpaceNodes;
  } else if (choice == "10") {
    request.command = CommandType::MoveToNamedPose;
    std::cout << "Inserisci ID task: " << std::flush;

    if (!std::getline(std::cin, request.task_name)) {
      return std::nullopt;
    }

    std::cout << "Inserisci ID posa: " << std::flush;

    if (!std::getline(std::cin, request.parameter)) {
      return std::nullopt;
    }

    while (true) {
      std::cout << "Inserisci target_pose_variable o 0 per terminare: " << std::flush;
      std::string pose_id;
      if (!std::getline(std::cin, pose_id)) {
        return std::nullopt;
      }
      if (pose_id == "0") {
        break;
      }

      if (!pose_id.empty()) {
        request.single_operation_pose.push_back(pose_id);
      }
    }
  } else if (choice == "11") {
    std::cout << "ID posa: " << std::flush;
    request.command = CommandType::SaveCurrentPose;
    if (!std::getline(std::cin, request.parameter)) {
      return std::nullopt;
    }
  } else if (choice == "12") {
    request.command = CommandType::SaveExplicitPose;
    std::cout << "ID posa: " << std::flush;
    if (!std::getline(std::cin, request.parameter)) {
      return std::nullopt;
    }

    std::cout << "Inserisci x y z qx qy qz qw: " << std::flush;
    std::string pose_input;
    if (!std::getline(std::cin, pose_input)) {
      return std::nullopt;
    }
    std::istringstream pose_stream(pose_input);
    std::array<double, 7> values{};
    std::string extra_value;
    if (!(pose_stream >> values[0] >> values[1] >> values[2] >>
      values[3] >> values[4] >> values[5] >> values[6]) ||
      (pose_stream >> extra_value))
    {
      std::cout << "Formato posa non valido: servono 7 valori numerici.\n";
      return std::nullopt;
    }

    geometry_msgs::msg::Pose explicit_pose;
    explicit_pose.position.x = values[0];
    explicit_pose.position.y = values[1];
    explicit_pose.position.z = values[2];
    explicit_pose.orientation.x = values[3];
    explicit_pose.orientation.y = values[4];
    explicit_pose.orientation.z = values[5];
    explicit_pose.orientation.w = values[6];
    request.pose = explicit_pose;
  
  }else if (choice == "13") {
    request.command = CommandType::ExecuteComplexTask;
    std::cout << "ID task complessa: " << std::flush;
    if (!std::getline(std::cin, request.parameter)) {
      return std::nullopt;
    }
  } else {
    std::cout << "Comando non valido.\n";
    return std::nullopt;
  }

  return request;
}

bool FrankaFsmNode::answerPendingDecision()
{
  franka_interfaces::msg::DecisionRequest request;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (pending_decisions_.empty()) {
      std::cout << "Nessuna decisione in attesa.\n";
      return false;
    }
    request = pending_decisions_.front();
  }

  std::cout << request.description << "\n";
  for (std::size_t index = 0; index < request.option_codes.size(); ++index) {
    std::cout << "  " << request.option_codes[index];
    if (index < request.option_labels.size()) {
      std::cout << ". " << request.option_labels[index];
    }
    std::cout << '\n';
  }

  std::string input;
  int32_t decision_code = 0;
  while (true) {
    std::cout << "Codice decisione: " << std::flush;
    if (!std::getline(std::cin, input)) {
      return false;
    }
    std::istringstream input_stream(input);
    std::string extra_value;
    if (input == "s" || input == "S") {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!pending_decisions_.empty() &&
          pending_decisions_.front().request_id == request.request_id &&
          pending_decisions_.front().decision_id == request.decision_id)
        {
          pending_decisions_.pop();
        }
      }
      sendCommand(CommandType::Stop);
      std::cout << "STOP inviato.\n";
      return true;
    }
    if (input_stream >> decision_code && !(input_stream >> extra_value) &&
      std::find(request.option_codes.begin(), request.option_codes.end(), decision_code) !=
      request.option_codes.end())
    {
      break;
    }
    std::cout << "Codice non valido. Scegli uno dei codici mostrati.\n";
  }

  franka_interfaces::msg::DecisionResponse response;
  response.request_id = request.request_id;
  response.decision_id = request.decision_id;
  response.decision_code = decision_code;
  decision_response_->publish(response);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!pending_decisions_.empty() &&
      pending_decisions_.front().request_id == request.request_id &&
      pending_decisions_.front().decision_id == request.decision_id)
    {
      pending_decisions_.pop();
    }
  }
  std::cout << "Decisione inviata.\n";
  return false;
}

void FrankaFsmNode::waitForComplexTask(std::uint64_t request_id)
{
  std::cout << "Task complessa in esecuzione. Inserisci 's' per inviare STOP.\n";
  bool stop_sent = false;

  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      state_condition_.wait_for(
        lock,
        std::chrono::milliseconds(100),
        [this, request_id]() {
          return pending_commands_.find(request_id) == pending_commands_.end() ||
                 !pending_decisions_.empty();
        });

      if (pending_commands_.find(request_id) == pending_commands_.end()) {
        return;
      }

      if (!pending_decisions_.empty()) {
        lock.unlock();
        stop_sent = answerPendingDecision() || stop_sent;
        continue;
      }
    }

    if (stop_sent) {
      continue;
    }

    pollfd input_fd{STDIN_FILENO, POLLIN, 0};
    const int poll_result = ::poll(&input_fd, 1, 0);
    if (poll_result > 0 && (input_fd.revents & (POLLIN | POLLHUP))) {
      std::string input;
      if (!std::getline(std::cin, input) || input == "s" || input == "S") {
        sendCommand(CommandType::Stop);
        stop_sent = true;
        std::cout << "STOP inviato; attendo la fine della task.\n";
      } else {
        std::cout << "Task ancora attiva. Inserisci 's' per inviare STOP.\n";
      }
    }
  }
}

std::string FrankaFsmNode::operationsPosePrint(const std::vector<std::string> & operationPose)
{
  std::ostringstream operations;
  for (std::size_t i = 0; i < operationPose.size(); ++i) {
    if (i > 0) {
      operations << ", ";
    }
    operations << operationPose[i];
  }
  const std::string operations_text = operations.str();

  return operations_text;

}

std::optional<std::uint64_t> FrankaFsmNode::sendCommand(
  CommandType command,
  const std::string & taskName,
  const std::string & parameter,
  const std::vector<std::string> & operationPose,
  const std::optional<geometry_msgs::msg::Pose> & pose)
{

  const bool requires_parameter =
    command == CommandType::ExecuteTask ||
    command == CommandType::ExecuteComplexTask ||
    command == CommandType::MoveToNamedPose ||
    command == CommandType::SaveCurrentPose ||
    command == CommandType::SaveExplicitPose;
  if (requires_parameter && parameter.empty()) {
    RCLCPP_WARN(get_logger(), "Parametro richiesto per il comando mancante");
    return std::nullopt;
  }

  const bool requires_pose_operation = command == CommandType::MoveToNamedPose;
  if (requires_pose_operation && taskName.empty()) {
    RCLCPP_WARN(get_logger(), "Nessun task selezionato.");
    return std::nullopt;
  }
  if (requires_pose_operation && operationPose.empty()) {
    RCLCPP_WARN(get_logger(), "Nessuna posa per operazioni del task inserita.");
    return std::nullopt;
  }
  if (command == CommandType::SaveExplicitPose && !pose.has_value()) {
    RCLCPP_WARN(get_logger(), "Posa esplicita mancante");
    return std::nullopt;
  }

  franka_interfaces::msg::BridgeCommand message;
  message.request_id = next_request_id_++;
  message.parameter = parameter;
  if (pose.has_value()) {
    message.pose = pose.value();
  }

  switch (command) {
    case CommandType::ShowTasks:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_SHOW_TASKS;
      break;
    case CommandType::ExecuteTask:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_EXECUTE_TASK;
      break;
    case CommandType::ExecuteComplexTask:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_EXECUTE_COMPLEX_TASK;
      break;
    case CommandType::Stop:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_STOP;
      break;
    case CommandType::OpenBrakes:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_OPEN_BRAKES;
      break;
    case CommandType::CloseBrakes:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_CLOSE_BRAKES;
      break;
    case CommandType::ActivateFci:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_ACTIVATE_FCI;
      break;
    case CommandType::DeactivateFci:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_DEACTIVATE_FCI;
      break;
    case CommandType::SwitchToExec:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_SWITCH_TO_EXEC;
      break;
    case CommandType::ShowAddressSpaceNodes:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_SHOW_ADDRESS_SPACE_NODES;
      break;
    case CommandType::MoveToNamedPose:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_MOVE_TO_NAMED_POSE;
      message.task_name = taskName;
      message.single_operation_pose = operationPose;
      break;
    case CommandType::SaveCurrentPose:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_SAVE_CURRENT_POSE;
      break;
    case CommandType::SaveExplicitPose:
      message.command = franka_interfaces::msg::BridgeCommand::COMMAND_SAVE_EXPLICIT_POSE;
      break;
  }

  const auto request_id = message.request_id;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_commands_[request_id] = command;
    state_ = FsmState::WaitingForResult;
    if (command == CommandType::ExecuteTask ||
      command == CommandType::ExecuteComplexTask ||
      command == CommandType::MoveToNamedPose)
    {
      task_active_ = true;
    }
  }

  std::string operations_text = operationsPosePrint(operationPose);
  RCLCPP_DEBUG(
    get_logger(),
    "Invio comando=%u request_id=%lu task_id='%s', parameter='%s', operation_poses= '[%s]'",
    static_cast<unsigned int>(message.command),
    static_cast<unsigned long>(request_id),
    message.task_name.c_str(),
    message.parameter.c_str(),
    operations_text.c_str());
  command_publisher_->publish(message);
  RCLCPP_INFO(
    get_logger(), "Comando pubblicato con request_id=%lu",
    static_cast<unsigned long>(request_id));
  return request_id;
}


} //namespace franka_fsm
