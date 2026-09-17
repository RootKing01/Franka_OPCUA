#ifndef FRANKA_OPCUA_BRIDGE__I_OPCUA_PROTOCOL_CLIENT_
#define FRANKA_OPCUA_BRIDGE__I_OPCUA_PROTOCOL_CLIENT_

#include "franka_opcua_bridge/i_protocol_client.hpp"

namespace franka_opcua_bridge
{

class IOpcUaProtocolClient : public IProtocolClient
{

public:
  virtual ~IOpcUaProtocolClient() = default;

  //browse_path: es: {{"Robot", "ExecutionControl"}, "OpenBrakes"}
  virtual CallResult callMethod(

    const std::vector<std::string> & object_browse_path,
    const std::string & method_name,
    const std::vector<Value> & args) = 0;

  virtual bool readValue(

    const std::vector<std::string> & variable_browse_path,
    Value & out_value) = 0;

  virtual bool writeValue(

    const std::vector<std::string> & variable_browse_path,
    const Value & out_value) = 0;


};


}

#endif
