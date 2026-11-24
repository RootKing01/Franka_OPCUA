#ifndef FRANKA_OPCUA_BRIDGE__OPCUA_WORKER_HPP_
#define FRANKA_OPCUA_BRIDGE__OPCUA_WORKER_HPP_


#include "franka_opcua_bridge/interfaces/i_franka.hpp"
#include "franka_opcua_bridge/interfaces/i_pose_storage.hpp"
#include "franka_opcua_bridge/interfaces/i_task_control.hpp"
#include "franka_opcua_bridge/commands/franka_command_manager.hpp"
#include "franka_opcua_bridge/commands/franka_command_result.hpp"
#include "franka_opcua_bridge/performance_monitor.hpp"
#include "franka_opcua_bridge/events/franka_decision.hpp"
#include "franka_opcua_bridge/active_task.hpp"

#include <thread>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <cstddef>
#include <atomic>
#include <chrono>
#include <string>
#include <vector>

namespace franka_opcua_bridge
{

enum class WorkerStatus
{

  Disconnected,
  Connecting,
  Ready,
  Error,
  Stopping

};

class OpcuaWorker
{

public:
  OpcuaWorker(
    std::unique_ptr<IFranka> robot,
    IPoseStorage * pose_storage,
    std::size_t command_capacity,
    ITaskControl * task_control = nullptr,
    std::shared_ptr<PerformanceMonitor> performance_monitor = {});

  ~OpcuaWorker();

  bool start();
  void stop();

  bool enqueueCommand(FrankaRobotCommand command);
  bool enqueuePriority(FrankaRobotCommand command);

  std::optional<FrankaCommandResult> tryGetResult();
  void printTree(const namespaceTree & node, std::size_t depth = 0);

  // Il bridge preleva le richieste di decisione generate dal worker.
  std::optional<FrankaDecisionRequest> tryGetDecisionRequest();

  // Il bridge consegna al worker la risposta ricevuta dalla FSM.
  bool submitDecision(const FrankaDecisionResponse& response);

  WorkerStatus status() const;

private:
  std::unique_ptr<IFranka> franka_;
  IPoseStorage * pose_storage_;
  ITaskControl * task_control_;
  std::shared_ptr<PerformanceMonitor> performance_monitor_;
  FrankaCommandManager command_manager_;
  std::queue<FrankaCommandResult> result_queue_;
  mutable std::mutex mutex_;
  std::condition_variable result_condition_;
  std::condition_variable state_condition_;
  std::thread worker_thread_;
  WorkerStatus worker_state_;
  bool start_attempted_{false};
  bool stop_pending_{false};
  bool stop_not_needed_{false};
  std::atomic_bool abort_requested_{false};

  std::optional<ActiveTask> active_task_;

  // Richieste di decisione in uscita verso il bridge.
  std::queue<FrankaDecisionRequest> decision_queue_;

  // Risposta ricevuta dal bridge, in attesa di essere elaborata.
  std::optional<FrankaDecisionResponse> pending_decision_;

  //rappresenta il checkpoint per il quale stiamo aspettando una risposta
  std::optional<FrankaDecisionRequest> outstanding_decision_;

  // Protegge la comunicazione tra worker e callback ROS2.
  mutable std::mutex decision_mutex_;


  void run();

  FrankaCommandResult executeCommand(const FrankaRobotCommand & command);

  void pushResult(FrankaCommandResult result);

  bool pushDecisionRequest(FrankaDecisionRequest request);

  std::unique_ptr<RobotStatus> robotStatus();

  bool startActiveTask(const FrankaRobotCommand & command);

  void monitorActiveTask();

  void completeActiveTask(FrankaCommandStatus status, const std::string & message);

  bool completeDecision(uint64_t request_id, uint64_t decision_id);

  std::optional<FrankaDecisionResponse> takePendingDecision();

  void processActiveTask();

};


}

#endif
