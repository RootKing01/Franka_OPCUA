#ifndef FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_RESULT_HPP_
#define FRANKA_OPCUA_BRIDGE__COMMANDS__FRANKA_COMMAND_RESULT_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace franka_opcua_bridge
{


enum class FrankaCommandStatus
{

  Accepted,    //Vuol dire che il comando è stato accettato dalla coda, non che sia stato eseguito
  Completed,
  Rejected,
  Failed,
  Cancelled,
  Timeout,
  NotReady,
  Disconnected

};


struct FrankaCommandResult
{

  std::uint64_t command_id;
  FrankaCommandStatus status;
  std::string message;
  std::chrono::steady_clock::time_point finished_at;

};

}

#endif
