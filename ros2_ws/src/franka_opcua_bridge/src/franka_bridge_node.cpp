#include "franka_opcua_bridge/franka_bridge_node.hpp"

#include "franka_opcua_bridge/opcua_client.hpp"
#include "franka_opcua_bridge/franka_robot.hpp"

#include <functional>
#include <memory>
#include <string>
#include <atomic>
#include <cstdint>
#include <inttypes.h>
#include <chrono>
#include <cmath>
#include <optional>
#include <sstream>
#include <vector>

namespace franka_opcua_bridge
{

namespace
{

std::uint8_t commandStatusValue(FrankaCommandStatus status)
{
  switch (status) {
    case FrankaCommandStatus::Accepted:
      return franka_interfaces::msg::BridgeResult::STATUS_ACCEPTED;
    case FrankaCommandStatus::Completed:
      return franka_interfaces::msg::BridgeResult::STATUS_COMPLETED;
    case FrankaCommandStatus::Rejected:
      return franka_interfaces::msg::BridgeResult::STATUS_REJECTED;
    case FrankaCommandStatus::Failed:
      return franka_interfaces::msg::BridgeResult::STATUS_FAILED;
    case FrankaCommandStatus::Cancelled:
      return franka_interfaces::msg::BridgeResult::STATUS_CANCELLED;
    case FrankaCommandStatus::NotNeeded:
      return franka_interfaces::msg::BridgeResult::STATUS_NOT_NEEDED;
    case FrankaCommandStatus::Timeout:
      return franka_interfaces::msg::BridgeResult::STATUS_TIMEOUT;
    case FrankaCommandStatus::NotReady:
      return franka_interfaces::msg::BridgeResult::STATUS_NOT_READY;
    case FrankaCommandStatus::Disconnected:
      return franka_interfaces::msg::BridgeResult::STATUS_DISCONNECTED;
  }
  return franka_interfaces::msg::BridgeResult::STATUS_UNKNOWN;
}

}  // namespace

FrankaBridgeNode::FrankaBridgeNode(const std::string & robot_id)
: Node("franka_" + robot_id + "_opcua_bridge")
{
  RCLCPP_INFO(
    this->get_logger(),
    "Franka OPC UA bridge avviato per robot '%s'",
    robot_id.c_str());

  // Creazione client OPC UA
  auto client = std::make_unique<OpcuaClient>();

  // Creazione astrazione Franka
  auto robot = std::make_unique<FrankaRobot>(std::move(client));
  auto * pose_storage = static_cast<IPoseStorage *>(robot.get());
  auto * task_control = static_cast<ITaskControl *>(robot.get());

  //Creazione del worker
  worker_ = std::make_unique<OpcuaWorker>(
    std::move(robot), pose_storage, 5, task_control);

  //Avvio del worker
  if (!worker_->start()) {

    throw std::runtime_error("Avvio del worker fallito");
  }

  RCLCPP_INFO(this->get_logger(), "Controllo del robot ottenuto");

  // Topic dei comandi
  const std::string topic_name = "/franka_" + robot_id + "/command";

  command_subscription_ =
    this->create_subscription<franka_interfaces::msg::BridgeCommand>(
    topic_name,
    10,
    std::bind(
      &FrankaBridgeNode::commandCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "In ascolto sul topic '%s'",
    topic_name.c_str());

  // Topic dei risultati
  const std::string result_topic = "/franka_" + robot_id + "/command_result";

  result_publisher_ =
    this->create_publisher<franka_interfaces::msg::BridgeResult>(result_topic, 10);

  RCLCPP_INFO(
    this->get_logger(),
    "Risultati in scrittura sul topic '%s'",
    result_topic.c_str());

  result_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {publishPendingResults();});


}


void FrankaBridgeNode::publishPendingResults()
{
  while (auto result = worker_->tryGetResult()) {
    franka_interfaces::msg::BridgeResult message_result;
    message_result.request_id = result->command_id;
    message_result.status = commandStatusValue(result->status);
    message_result.message = result->message;
    message_result.task_ids = result->task_ids;
    result_publisher_->publish(message_result);
    RCLCPP_INFO(
      get_logger(),
      "Risultato pubblicato: request_id=%lu status=%u task_count=%zu message='%s'",
      static_cast<unsigned long>(message_result.request_id),
      static_cast<unsigned int>(message_result.status),
      message_result.task_ids.size(),
      message_result.message.c_str());
  }
}


void FrankaBridgeNode::commandCallback(
  const franka_interfaces::msg::BridgeCommand::SharedPtr msg)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Comando ricevuto: request_id=%lu command=%u parameter='%s'",
    static_cast<unsigned long>(msg->request_id),
    static_cast<unsigned int>(msg->command),
    msg->parameter.c_str());

  FrankaCommandType command_type;
  bool priority = false;
  std::string parameter;

  switch (msg->command) {
    case franka_interfaces::msg::BridgeCommand::COMMAND_OPEN_BRAKES:
      command_type = FrankaCommandType::OpenBrakes;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_CLOSE_BRAKES:
      command_type = FrankaCommandType::CloseBrakes;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_ACTIVATE_FCI:
      command_type = FrankaCommandType::ActivateFci;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_DEACTIVATE_FCI:
      command_type = FrankaCommandType::DeactivateFci;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_SWITCH_TO_EXEC:
      command_type = FrankaCommandType::SwitchToExec;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_SHOW_ADDRESS_SPACE_NODES:
      command_type = FrankaCommandType::ShowAddressSpaceNodes;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_SHOW_TASKS:
      command_type = FrankaCommandType::ShowTasks;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_EXECUTE_TASK:
      if (msg->parameter.empty()) {
        RCLCPP_WARN(get_logger(), "request_id=%lu: ID della task mancante",
          static_cast<unsigned long>(msg->request_id));
        return;
      }
      command_type = FrankaCommandType::ExecuteNamedTask;
      parameter = msg->parameter;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_STOP:
      command_type = FrankaCommandType::Stop;
      priority = true;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_MOVE_TO_NAMED_POSE:
      if (msg->parameter.empty()) {
        RCLCPP_WARN(get_logger(), "request_id=%lu: ID della posa mancante",
          static_cast<unsigned long>(msg->request_id));
        return;
      }
      command_type = FrankaCommandType::MoveToNamedPose;
      parameter = msg->parameter;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_SAVE_CURRENT_POSE:
      if (msg->parameter.empty()) {
        RCLCPP_WARN(get_logger(), "request_id=%lu: ID della posa da salvare mancante",
          static_cast<unsigned long>(msg->request_id));
        return;
      }
      command_type = FrankaCommandType::SaveCurrentPose;
      parameter = msg->parameter;
      break;
    case franka_interfaces::msg::BridgeCommand::COMMAND_SAVE_EXPLICIT_POSE:
      if (msg->parameter.empty()) {
        RCLCPP_WARN(get_logger(), "request_id=%lu: ID della posa da salvare mancante",
          static_cast<unsigned long>(msg->request_id));
        return;
      }
      command_type = FrankaCommandType::SaveExplicitPose;
      parameter = msg->parameter;
      break;
    default:
      RCLCPP_WARN(get_logger(), "request_id=%lu: comando ROS non riconosciuto",
        static_cast<unsigned long>(msg->request_id));
      return;
  }

  FrankaRobotCommand robot_command;
  robot_command.command_id = msg->request_id;
  robot_command.type = command_type;
  robot_command.parameter = parameter;
  if (msg->command == franka_interfaces::msg::BridgeCommand::COMMAND_SAVE_EXPLICIT_POSE) {
    robot_command.pose = msg->pose;
  }
  robot_command.created_at = std::chrono::steady_clock::now();

  const auto request_id = msg->request_id;
  const bool enqueued = priority ?
    worker_->enqueuePriority(std::move(robot_command)) :
    worker_->enqueueCommand(std::move(robot_command));

  if (enqueued) {
    RCLCPP_INFO(get_logger(), "Comando accodato con request_id=%lu",
      static_cast<unsigned long>(request_id));
  } else {
    RCLCPP_ERROR(get_logger(), "Comando rifiutato dal worker, request_id=%lu",
      static_cast<unsigned long>(request_id));
  }
}

FrankaBridgeNode::~FrankaBridgeNode()
{
  if (worker_) {
    result_timer_.reset();
    worker_->stop();
    publishPendingResults();
  }
}


}  // namespace franka_opcua_bridge


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {

    auto node = std::make_shared<franka_opcua_bridge::FrankaBridgeNode>("right");

    rclcpp::spin(node);

  } catch (const std::exception & e) {

    RCLCPP_ERROR(rclcpp::get_logger("franka_opcua_bridge"), "Avvio fallito: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
