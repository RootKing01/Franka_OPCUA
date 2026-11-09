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

const char * commandStatusName(FrankaCommandStatus status)
{
  switch (status) {
    case FrankaCommandStatus::Accepted:
      return "Accepted";
    case FrankaCommandStatus::Completed:
      return "Completed";
    case FrankaCommandStatus::Rejected:
      return "Rejected";
    case FrankaCommandStatus::Failed:
      return "Failed";
    case FrankaCommandStatus::Cancelled:
      return "Cancelled";
    case FrankaCommandStatus::Timeout:
      return "Timeout";
    case FrankaCommandStatus::NotReady:
      return "NotReady";
    case FrankaCommandStatus::Disconnected:
      return "Disconnected";
  }
  return "Unknown";
}

bool parseExplicitPoseCommand(
  const std::string & command,
  std::string & pose_id,
  geometry_msgs::msg::Pose & pose)
{
  constexpr const char * kPrefix = "save_pose:";
  const std::string payload = command.substr(std::string(kPrefix).size());

  std::stringstream stream(payload);
  std::vector<std::string> fields;
  std::string field;
  while (std::getline(stream, field, ':')) {
    fields.push_back(field);
  }

  if (fields.size() != 8 || fields[0].empty()) {
    return false;
  }

  try {
    double values[7];
    for (std::size_t index = 0; index < 7; ++index) {
      std::size_t parsed_characters = 0;
      values[index] = std::stod(fields[index + 1], &parsed_characters);
      if (parsed_characters != fields[index + 1].size()) {
        return false;
      }
    }

    for (const double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }

    pose_id = fields[0];
    pose.position.x = values[0];
    pose.position.y = values[1];
    pose.position.z = values[2];
    pose.orientation.x = values[3];
    pose.orientation.y = values[4];
    pose.orientation.z = values[5];
    pose.orientation.w = values[6];
    return true;
  } catch (const std::exception &) {
    return false;
  }
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

  // Topic dei risultati
  const std::string result_topic = "/franka_" + robot_id + "/command_result";

  result_publisher_ = this->create_publisher<std_msgs::msg::String>(result_topic, 10);

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
    std_msgs::msg::String message_result;
    message_result.data =
      "command_id=" + std::to_string(result->command_id) +
      ";status=" + commandStatusName(result->status) +
      ";message=" + result->message;
    result_publisher_->publish(message_result);
  }
}



void FrankaBridgeNode::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  auto enqueueCommand = [this](
    FrankaCommandType type,
    const std::string & parameter = "",
    const std::optional<geometry_msgs::msg::Pose> & pose = std::nullopt) {

      FrankaRobotCommand robot_command;
      robot_command.command_id = next_command_id_.fetch_add(1);
      robot_command.type = type;
      robot_command.parameter = parameter;
      robot_command.pose = pose;
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
  
  }else if(command == "show_tasks"){

    enqueueCommand(FrankaCommandType::ShowTasks);

  }else if(command == "switch_exec"){

    enqueueCommand(FrankaCommandType::SwitchToExec);

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

  } else if (command.rfind("save_pose:", 0) == 0) {

    const std::string payload = command.substr(std::string("save_pose:").size());

    if (payload.empty()) {
      RCLCPP_WARN(this->get_logger(), "Nome della posa da salvare mancante");
    } else if (payload.find(':') == std::string::npos) {
      enqueueCommand(FrankaCommandType::SaveCurrentPose, payload);
    } else {
      std::string pose_id;
      geometry_msgs::msg::Pose pose;
      if (!parseExplicitPoseCommand(command, pose_id, pose)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Formato posa non valido. Atteso save_pose:id:x:y:z:qx:qy:qz:qw");
      } else {
        enqueueCommand(FrankaCommandType::SaveExplicitPose, pose_id, pose);
      }
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
