#ifndef FRANKA_OPCUA_BRIDGE__OPCUA_CLIENT_
#define FRANKA_OPCUA_BRIDGE__OPCUA_CLIENT_

#include "franka_opcua_bridge/i_opcua_protocol_client.hpp"
#include <open62541/ua_client.h>
#include <open62541/ua_config_default.h>
#include <open62541/ua_client_highlevel.h>
#include <open62541/ua_log_stdout.h>

#include <opc_ua_service_types_generated.h>
#include <stdlib.h>

namespace franka_opcua_bridge
{


class OpcuaClient : public IOpcUaProtocolClient
{

public:
  CallResult callMethod(

    const std::vector<std::string> & object_browse_path,
    const std::string & method_name,
    const std::vector<Value> & args) override;

  bool readValue(

    const std::vector<std::string> & variable_browse_path,
    Value & out_value) override;

  bool writeValue(

    const std::vector<std::string> & variable_browse_path,
    const Value & out_value) override;

private:
  UA_NodeId TranslateBrowsePathtoNodeId(UA_Client * client, std::vector<std::string> browse_path);

  void writeKeyIntPair(UA_Client * client, std::string key, int value);

  UA_Int32 readKeyIntPair(UA_Client * client, std::string key);

};


}


#endif
