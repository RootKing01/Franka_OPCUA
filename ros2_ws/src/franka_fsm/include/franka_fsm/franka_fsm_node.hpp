#ifndef FRANKA_FSM__FRANKA_FSM_NODE_HPP_
#define FRANKA_FSM__FRANKA_FSM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "franka_interfaces/msg/bridge_command.hpp"
#include "franka_interfaces/msg/bridge_result.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>


namespace franka_fsm{

class FrankaFsmNode : public rclcpp::Node{

public:

    FrankaFsmNode();
    
    ~FrankaFsmNode() override = default;



private:

    rclcpp::Publisher<franka_interfaces::msg::BridgeCommand>::SharedPtr command_publisher_;
    rclcpp::Subscription<franka_interfaces::msg::BridgeResult>::SharedPtr result_subscription_;
    std::uint64_t next_request_id_{1};
    std::vector<std::string> available_task_ids_;

    enum class CommandType
    {
        ShowTasks,
        ExecuteTask,
        Stop,
        OpenBrakes,
        CloseBrakes,
        ActivateFci,
        DeactivateFci,
        SwitchToExec,
        ShowAddressSpaceNodes,
        MoveToNamedPose,
        SaveCurrentPose,
        SaveExplicitPose
    };

    std::unordered_map<std::uint64_t, CommandType> pending_commands_;

    enum class FsmState
    {
        Idle,
        WaitingForResult,
        Picking,
        BlockReady,
        Moving,
        Completed,
        Interrupted,
        Error
    };

    enum class BridgeStatus
    {
        Unknown, 
        Accepted,
        Completed,
        Rejected,
        Failed,
        Cancelled,
        NotNeeded,
        Timeout,
        NotReady,
        Disconnected
    };

    FsmState state_{FsmState::Idle};

    std::optional<std::uint64_t> sendCommand(
        CommandType command,
        const std::string & parameter = "",
        const std::optional<geometry_msgs::msg::Pose> & pose = std::nullopt);



};

}  // namespace franka_fsm


#endif