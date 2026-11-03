#include "franka_opcua_bridge/opcua_worker.hpp"

#include <stdexcept>

namespace franka_opcua_bridge{

OpcuaWorker::OpcuaWorker(std::unique_ptr<IFranka> robot, std::size_t command_capacity)
: franka_(std::move(robot)),
  command_manager_(command_capacity),
  worker_state_(WorkerStatus::Disconnected){
    if(franka_ == nullptr) throw std::invalid_argument("Robot cannot be null");
  }


OpcuaWorker::~OpcuaWorker(){
    stop();
}

bool OpcuaWorker::start(){

    if( worker_thread_.joinable()) return false;
    
    std::unique_lock<std::mutex> lock(mutex_);

    if( worker_state_ != WorkerStatus::Disconnected ) return false;

    worker_state_ = WorkerStatus::Connecting;

    lock.unlock();

    worker_thread_ = std::thread([this](){
        run();
    });

    lock.lock();

    state_condition_.wait(lock, [this]{
        return worker_state_ == WorkerStatus::Ready  || 
               worker_state_ == WorkerStatus::Error ||
               worker_state_ == WorkerStatus::Stopping ||
               worker_state_ == WorkerStatus::Disconnected;
    });


    if( worker_state_ == WorkerStatus::Ready)
    {
        lock.unlock();
        return true;
    } 

    if( worker_state_ == WorkerStatus::Error){
        
        lock.unlock();
        worker_thread_.join();
        return false;
    }

    return false;

}

void OpcuaWorker::stop(){

    if((worker_thread_.joinable())){

        std::unique_lock<std::mutex> lock(mutex_);
        worker_state_ = WorkerStatus::Stopping;
        lock.unlock();

        command_manager_.shutdown();

        worker_thread_.join();

        lock.lock();
        worker_state_ = WorkerStatus::Disconnected;
        lock.unlock();

        result_condition_.notify_all();

    }

}

bool OpcuaWorker::enqueueCommand(FrankaRobotCommand command){

    std::unique_lock<std::mutex> lock(mutex_);
    
    if(worker_state_ != WorkerStatus::Ready){
        
        lock.unlock();
        return false;
    }

    lock.unlock();
    return command_manager_.enqueue(command);

}


void OpcuaWorker::run(){

    // Connessione al robot
    if (!franka_->connect()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (worker_state_ != WorkerStatus::Stopping) {
            worker_state_ = WorkerStatus::Error;
        }
        lock.unlock();
        
        state_condition_.notify_one();
        result_condition_.notify_all();
        return;
    }

    bool stopping = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping = worker_state_ == WorkerStatus::Stopping;
    }

    if (stopping) {
        franka_->disconnect();

        std::lock_guard<std::mutex> lock(mutex_);
        worker_state_ = WorkerStatus::Disconnected;
        state_condition_.notify_all();
        result_condition_.notify_all();
        return;
    }

    if (!franka_->requestControl(true)) {
    
        franka_->disconnect();
        
        std::unique_lock<std::mutex> lock(mutex_);
        if (worker_state_ != WorkerStatus::Stopping) {
            worker_state_ = WorkerStatus::Error;
        }
        lock.unlock();
        
        state_condition_.notify_one();
        result_condition_.notify_all();
        return;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    if (worker_state_ == WorkerStatus::Stopping) {
        lock.unlock();
        franka_->releaseControl();
        franka_->disconnect();

        lock.lock();
        worker_state_ = WorkerStatus::Disconnected;
        lock.unlock();

        state_condition_.notify_all();
        result_condition_.notify_all();
        return;
    }

    worker_state_ = WorkerStatus::Ready;
    lock.unlock();
    state_condition_.notify_one();
    

    while(true)
    {
        auto command = command_manager_.waitAndPop();

        if( !command){
            break; //shutdown
        }

        auto result = executeCommand(*command);
        pushResult(std::move(result));
    }
    
    franka_->releaseControl();
    franka_->disconnect();

    lock.lock();
    worker_state_ = WorkerStatus::Disconnected;
    lock.unlock();

    state_condition_.notify_all();
    result_condition_.notify_all();
}


FrankaCommandResult OpcuaWorker::executeCommand(const FrankaRobotCommand & command){

    FrankaCommandResult result;
    result.command_id = command.command_id;
    result.status = FrankaCommandStatus::Rejected;

    bool success = false;
    std::string message;

    switch(command.type){

        case FrankaCommandType::OpenBrakes:
            success = franka_->openBrakes();
            message = success ? "Brakes opened" : "Failed to open brakes";
            break;
        
        case FrankaCommandType::CloseBrakes:
            success = franka_->closeBrakes();
            message = success ? "Brakes closed" : "Failed to close brakes";
            break;

        case FrankaCommandType::Stop:
            success = franka_->stop();
            message = success ? "Robot stopped" : "Failed to stop robot";
            break;

        case FrankaCommandType::ExecuteNamedTask:
            success = franka_->executeNamedTask(command.parameter);
            message = success ? "Task executed" : "Failed to execute task";
            break;

        case FrankaCommandType::MoveToNamedPose:
            success = franka_->moveToNamedPose(command.parameter);
            message = success ? "Pose reached" : "Failed to reach pose";
            break;
        
        case FrankaCommandType::ActivateFci:
            success = franka_->activateFCI();
            message = success ? "FCI activated" : "Failed to activate FCI";
            break;

        case FrankaCommandType::DeactivateFci:
            success = franka_->deactivateFCI();
            message = success ? "FCI deactivated" : "Failed to deactivate FCI";
            break;
        
        default:
            result.status = FrankaCommandStatus::Rejected;
            result.finished_at = std::chrono::steady_clock::now();
            result.message = "Unknown command type";
            return result;
    }

    result.status = success ? FrankaCommandStatus::Completed : FrankaCommandStatus::Failed;
    result.finished_at = std::chrono::steady_clock::now();
    result.message = message;

    return result;

}


void OpcuaWorker::pushResult(FrankaCommandResult result){

    std::lock_guard<std::mutex> lock(mutex_);

    result_queue_.push(std::move(result));

    result_condition_.notify_one();
}

std::optional<FrankaCommandResult> OpcuaWorker::tryGetResult(){
    
    std::lock_guard<std::mutex> lock(mutex_);

    if( result_queue_.empty()) return std::nullopt;

    FrankaCommandResult result = std::move(result_queue_.front());

    result_queue_.pop();

    return result;
}

std::optional<FrankaCommandResult> OpcuaWorker::waitAndPopResult(){

    std::unique_lock<std::mutex> lock(mutex_);

    result_condition_.wait(lock, [this]{
        return !result_queue_.empty() || worker_state_ == WorkerStatus::Disconnected
                                      || worker_state_ == WorkerStatus::Error;
    });

    if( result_queue_.empty()) return std::nullopt;

    FrankaCommandResult result = std::move(result_queue_.front());

    result_queue_.pop();

    return result;

}


WorkerStatus OpcuaWorker::status() const{

    std::lock_guard<std::mutex> lock(mutex_);

    WorkerStatus state = worker_state_;

    return state;


}




}