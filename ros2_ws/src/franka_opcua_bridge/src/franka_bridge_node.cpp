#include "franka_opcua_bridge/franka_bridge_node.hpp"

#include "franka_opcua_bridge/opcua_client.hpp"

#include <functional>
#include <memory>
#include <string>

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
    franka_ = std::make_unique<FrankaRobot>(
        std::move(client));

    // Connessione al robot
    if (!franka_->connect())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Connessione al robot fallita");

        throw std::runtime_error(
            "Impossibile connettersi al robot tramite OPC UA");
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Connessione OPC UA stabilita");

    // Topic dei comandi
    const std::string topic_name =
        "/franka_" + robot_id + "/command";

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


void FrankaBridgeNode::commandCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Comando ricevuto: '%s'",
        msg->data.c_str());

    const std::string command = msg->data;

    if (command == "open_brakes")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Richiesta: apertura freni");

        if (franka_->openBrakes())
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Freni aperti correttamente");
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Errore nell'apertura dei freni");
        }
    }
    else if (command == "close_brakes")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Richiesta: chiusura freni");

        if (franka_->closeBrakes())
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Freni chiusi correttamente");
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Errore nella chiusura dei freni");
        }
    }
    else if (command == "activate_FCI")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Richiesta: attivazione FCI");

        if (franka_->activateFCI())
        {
            RCLCPP_INFO(
                this->get_logger(),
                "FCI attivato correttamente");
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Errore nell'attivazione FCI");
        }
    }
    else if (command == "deactivate_FCI")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Richiesta: disattivazione FCI");

        if (franka_->deactivateFCI())
        {
            RCLCPP_INFO(
                this->get_logger(),
                "FCI disattivato correttamente");
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Errore nella disattivazione FCI");
        }
    }
    else if (command == "stop")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Richiesta: stop");

        if (franka_->stop())
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Stop eseguito correttamente");
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Errore durante lo stop");
        }
    }
    else
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Comando non riconosciuto: '%s'",
            command.c_str());
    }
}

}  // namespace franka_opcua_bridge


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<franka_opcua_bridge::FrankaBridgeNode>("right");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}