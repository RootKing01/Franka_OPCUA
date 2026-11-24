#ifndef FRANKA_OPCUA_BRIDGE__ACTIVE_TASK_
#define FRANKA_OPCUA_BRIDGE__ACTIVE_TASK_

#include <cstdint>
#include <string>
#include <chrono>

#include "franka_opcua_bridge/interfaces/task_state.hpp"
#include "franka_opcua_bridge/commands/franka_command.hpp"

namespace franka_opcua_bridge{

    enum class TaskPhase
    {
        Monitoring,
        WaitingDecision,
        ApplyingDecision
    };

    struct ActiveTask {
        
        FrankaRobotCommand command;
        TaskState state{TaskState::Starting};
        TaskPhase phase{TaskPhase::Monitoring};

        std::chrono::steady_clock::time_point started_at;
        bool execution_observed{false};
        bool decision_requested_for_checkpoint{false};
        std::uint64_t next_decision_id{1};
    };

}

#endif