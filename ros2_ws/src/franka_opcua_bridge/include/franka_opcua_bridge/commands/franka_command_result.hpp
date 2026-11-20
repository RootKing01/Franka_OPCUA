#ifndef FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_RESULT_HPP_
#define FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_RESULT_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace franka_opcua_bridge
{


enum class FrankaCommandStatus
{

  Accepted,    //Vuol dire che il comando è stato accettato dalla coda, non che sia stato eseguito
  Completed,
  Rejected,
  Failed,
  Cancelled,
  NotNeeded,
  Timeout,
  NotReady,
  Disconnected

};


struct FrankaCommandResult
{

  std::uint64_t command_id;
  FrankaCommandStatus status;
  std::string message;
  std::vector<std::string> task_ids;
  std::chrono::steady_clock::time_point finished_at;
  std::chrono::steady_clock::time_point received_at{};
  std::chrono::nanoseconds queue_wait_duration{0};
  std::chrono::nanoseconds execution_duration{0};
  std::chrono::nanoseconds processing_duration{0};

};

}

#endif
