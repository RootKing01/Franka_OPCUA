#include "franka_opcua_bridge/commands/franka_command_manager.hpp"
#include <stdexcept>
#include <utility>

namespace franka_opcua_bridge{



FrankaCommandManager::FrankaCommandManager(std::size_t capacity)
: capacity_(capacity),
  shutdown_requested_(false)
{
    if (capacity_ == 0){
        throw std::invalid_argument("Command queue capacity must be greater than zero");
    }

}

bool FrankaCommandManager::enqueue(FrankaRobotCommand command){
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if ( shutdown_requested_ ) return false;

    if( command_queue.size() >= capacity_ ) return false;

    command_queue.push(std::move(command));

    condition_.notify_one();

    return true;

}


//Interrompe il consumo normale durante lo shutdown
std::optional<FrankaRobotCommand> FrankaCommandManager::waitAndPop(){

    std::unique_lock<std::mutex> lock(mutex_);

    condition_.wait(lock, [this]{
        return !command_queue.empty() || shutdown_requested_;
    });

    if( shutdown_requested_ ) return std::nullopt;

    FrankaRobotCommand command = std::move(command_queue.front());
    command_queue.pop();

    return command;

}

//Permette di recuperare i comandi pendenti per marcarli come CANCELLED
std::optional<FrankaRobotCommand> FrankaCommandManager::tryPop(){

    std::lock_guard<std::mutex> lock(mutex_);

    if(command_queue.empty()) return std::nullopt;

    FrankaRobotCommand command = std::move(command_queue.front());
    command_queue.pop();

    return command;


}

void FrankaCommandManager::shutdown(){

    std::lock_guard<std::mutex> lock(mutex_);
    
    shutdown_requested_ = true;

    condition_.notify_all();
}

bool FrankaCommandManager::isShutdown() const{

    std::lock_guard<std::mutex> lock(mutex_);

    return shutdown_requested_;
}

std::size_t FrankaCommandManager::size() const{

    std::lock_guard<std::mutex> lock(mutex_);

    return command_queue.size();

}


bool FrankaCommandManager::empty() const{

    std::lock_guard<std::mutex> lock(mutex_);

    if( command_queue.size() == 0 ) return true;

    return false;
}


}