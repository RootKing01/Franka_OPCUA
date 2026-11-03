#ifndef FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_MANAGER_HPP_
#define FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_MANAGER_HPP_

#include "franka_opcua_bridge/commands/franka_command.hpp"
#include <queue>
#include <mutex>
#include <cstddef>
#include <condition_variable>
#include <optional>

namespace franka_opcua_bridge{

class FrankaCommandManager{

public:

    explicit FrankaCommandManager(std::size_t capacity);

    bool enqueue(FrankaRobotCommand command);

    std::optional<FrankaRobotCommand> waitAndPop();
    std::optional<FrankaRobotCommand> tryPop();

    void shutdown();
    bool isShutdown() const;

    std::size_t size() const;
    bool empty() const;

private:
    
    std::queue<FrankaRobotCommand> command_queue;
    mutable std::mutex mutex_;
    std::size_t capacity_;
    bool shutdown_requested_ = false;
    std::condition_variable condition_;

};


}


#endif