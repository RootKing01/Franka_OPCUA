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

namespace franka_opcua_bridge
{

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

  //Creazione del worker
  worker_ = std::make_unique<OpcuaWorker>(std::move(robot), 5);

  //Avvio del worker
  if (!worker_->start()) {

    throw std::runtime_error("Avvio del worker fallito");
  }

  RCLCPP_INFO(this->get_logger(), "Controllo del robot ottenuto");

  // Topic dei comandi
  const std::string topic_name = "/franka_" + robot_id + "/command";

  command_subscription_ =
    this->create_subscription<std_msgs::msg::String>(
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
}


void FrankaBridgeNode::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  auto enqueueCommand = [this](FrankaCommandType type, const std::string & parameter = "") {

      FrankaRobotCommand robot_command;
      robot_command.command_id = next_command_id_.fetch_add(1);
      robot_command.type = type;
      robot_command.parameter = parameter;
      robot_command.created_at = std::chrono::steady_clock::now();

      const auto command_id = robot_command.command_id;

      if (worker_->enqueueCommand(std::move(robot_command))) {
        RCLCPP_INFO(
          this->get_logger(),
          "Comando %lu accodato",
          command_id);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Comando %lu rifiutato dal worker",
          command_id);
      }
    };

    auto enqueuePriority = [this](FrankaCommandType type, const std::string & parameter = "") {

      FrankaRobotCommand robot_command;
      robot_command.command_id = next_command_id_.fetch_add(1);
      robot_command.type = type;
      robot_command.parameter = parameter;
      robot_command.created_at = std::chrono::steady_clock::now();

      const auto command_id = robot_command.command_id;

      if (worker_->enqueuePriority(std::move(robot_command))) {
        RCLCPP_INFO(
          this->get_logger(),
          "Comando %lu accodato",
          command_id);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Comando %lu rifiutato dal worker",
          command_id);
      }
    };


  RCLCPP_INFO(
    this->get_logger(),
    "Comando ricevuto: '%s'",
    msg->data.c_str());

  const std::string command = msg->data;

  if (command == "open_brakes") {

    enqueueCommand(FrankaCommandType::OpenBrakes);

  } else if (command == "close_brakes") {

    enqueueCommand(FrankaCommandType::CloseBrakes);

  } else if (command == "activate_FCI") {

    enqueueCommand(FrankaCommandType::ActivateFci);

  } else if (command == "deactivate_FCI") {

    enqueueCommand(FrankaCommandType::DeactivateFci);

  } else if (command == "stop") {

    enqueuePriority(FrankaCommandType::Stop);

  } else if (command.rfind("move_to_pose:", 0) == 0) {

    const std::string pose_id = command.substr(std::string("move_to_pose:").size());

    if (pose_id.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Nome della posa mancante");
    } else {
      enqueueCommand(FrankaCommandType::MoveToNamedPose, pose_id);
    }

  } else {

    RCLCPP_WARN(
      this->get_logger(),
      "Comando non riconosciuto: '%s'",
      command.c_str());
  }
}

FrankaBridgeNode::~FrankaBridgeNode()
{
  if (worker_) {
    worker_->stop();
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
