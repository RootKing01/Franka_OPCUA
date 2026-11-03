#ifndef FRANKA_OPCUA_BRIDGE__OPCUA_WORKER_HPP_
#define FRANKA_OPCUA_BRIDGE__OPCUA_WORKER_HPP_


#include "franka_opcua_bridge/interfaces/i_franka.hpp"
#include "franka_opcua_bridge/commands/franka_command_manager.hpp"
#include "franka_opcua_bridge/commands/franka_command_result.hpp"

#include <thread>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <cstddef>

namespace franka_opcua_bridge{

enum class WorkerStatus{

    Disconnected,
    Connecting,
    Ready,
    Error,
    Stopping

};


class OpcuaWorker{

public:

    OpcuaWorker(std::unique_ptr<IFranka> robot, std::size_t command_capacity);
    
    ~OpcuaWorker();

    bool start();
    void stop();

    bool enqueueCommand(FrankaRobotCommand command);

    std::optional<FrankaCommandResult> tryGetResult();
    std::optional<FrankaCommandResult> waitAndPopResult();

    WorkerStatus status() const;


private:

    std::unique_ptr<IFranka> franka_;
    FrankaCommandManager command_manager_;
    std::queue<FrankaCommandResult> result_queue_;
    mutable std::mutex mutex_;
    std::condition_variable result_condition_;
    std::condition_variable state_condition_;
    std::thread worker_thread_;
    WorkerStatus worker_state_;


    void run();
    
    FrankaCommandResult executeCommand(const FrankaRobotCommand & command);
    
    void pushResult(FrankaCommandResult result);

};





}

#endif