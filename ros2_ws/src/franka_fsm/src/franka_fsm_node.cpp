#include "franka_fsm/franka_fsm_node.hpp"

namespace franka_fsm{


FrankaFsmNode::FrankaFsmNode(): Node("franka_fsm")
{
    RCLCPP_INFO(get_logger(), "Nodo FSM avviato.");

    command_publisher_ = create_publisher<franka_interfaces::msg::BridgeCommand>(
        "/franka_right/command", 10);

    result_subscription_ = create_subscription<franka_interfaces::msg::BridgeResult>(
        "/franka_right/command_result",
        10,
        [this](franka_interfaces::msg::BridgeResult::ConstSharedPtr msg){
            RCLCPP_INFO(
                get_logger(),
                "Risultato ricevuto: request_id=%lu status=%u task_count=%zu message='%s'",
                static_cast<unsigned long>(msg->request_id),
                static_cast<unsigned int>(msg->status),
                msg->task_ids.size(),
                msg->message.c_str());

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
                        command == CommandType::MoveToNamedPose) {
                        state_ = FsmState::Completed;
                    } else {
                        state_ = FsmState::Idle;
                    }
                    break;
                case franka_interfaces::msg::BridgeResult::STATUS_CANCELLED:
                    state_ = FsmState::Interrupted;
                    break;
                case franka_interfaces::msg::BridgeResult::STATUS_NOT_NEEDED:
                    state_ = FsmState::Idle;
                    break;
                case franka_interfaces::msg::BridgeResult::STATUS_REJECTED:
                case franka_interfaces::msg::BridgeResult::STATUS_FAILED:
                case franka_interfaces::msg::BridgeResult::STATUS_TIMEOUT:
                case franka_interfaces::msg::BridgeResult::STATUS_NOT_READY:
                case franka_interfaces::msg::BridgeResult::STATUS_DISCONNECTED:
                case franka_interfaces::msg::BridgeResult::STATUS_UNKNOWN:
                default:
                    state_ = FsmState::Error;
                    break;
            }

            RCLCPP_INFO(
                get_logger(),
                "Stato FSM aggiornato: %u",
                static_cast<unsigned int>(state_));
        });
}


std::optional<std::uint64_t> FrankaFsmNode::sendCommand(
    CommandType command,
    const std::string & parameter,
    const std::optional<geometry_msgs::msg::Pose> & pose)
{
    const bool requires_parameter =
        command == CommandType::ExecuteTask ||
        command == CommandType::MoveToNamedPose ||
        command == CommandType::SaveCurrentPose ||
        command == CommandType::SaveExplicitPose;
    if (requires_parameter && parameter.empty()) {
        RCLCPP_WARN(get_logger(), "Parametro richiesto per il comando mancante");
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
            break;
        case CommandType::SaveCurrentPose:
            message.command = franka_interfaces::msg::BridgeCommand::COMMAND_SAVE_CURRENT_POSE;
            break;
        case CommandType::SaveExplicitPose:
            message.command = franka_interfaces::msg::BridgeCommand::COMMAND_SAVE_EXPLICIT_POSE;
            break;
    }

    const auto request_id = message.request_id;
    pending_commands_[request_id] = command;
    state_ = FsmState::WaitingForResult;
    RCLCPP_DEBUG(
        get_logger(),
        "Invio comando=%u request_id=%lu parameter='%s'",
        static_cast<unsigned int>(message.command),
        static_cast<unsigned long>(request_id),
        message.parameter.c_str());
    command_publisher_->publish(message);
    RCLCPP_INFO(
        get_logger(), "Comando pubblicato con request_id=%lu",
        static_cast<unsigned long>(request_id));
    return request_id;
}


} //namespace franka_fsm