#include "rclcpp/rclcpp.hpp"

class FrankaBridgeNode : public rclcpp::Node
{
public:
  FrankaBridgeNode()
  : Node("franka_opcua_bridge")
  {
    RCLCPP_INFO(this->get_logger(), "franka_opcua_bridge avviato correttamente");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrankaBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
