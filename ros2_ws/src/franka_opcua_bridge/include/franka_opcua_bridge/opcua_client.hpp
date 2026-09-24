#ifndef FRANKA_OPCUA_BRIDGE__OPCUA_CLIENT_
#define FRANKA_OPCUA_BRIDGE__OPCUA_CLIENT_

#include "franka_opcua_bridge/i_opcua_protocol_client.hpp"
#include "franka_opcua_bridge/opcua_value.hpp"

#include <open62541/client.h>
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>
#include <open62541/plugin/log_stdout.h>

#include <opc_ua_service_types_generated.h>
#include <stdlib.h>
#include <cstring>

namespace franka_opcua_bridge
{


class OpcuaClient : public IOpcUaProtocolClient
{

public:
  OpcuaClient() = default;

  //Disabilita copia
  OpcuaClient(const OpcuaClient &) = delete;
  OpcuaClient & operator=(const OpcuaClient &) = delete;
  
  bool connect() override;
  bool disconnect() override;
  bool isConnected() const override;

  //Il distruttore fa cleanup
  ~OpcuaClient() override;


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
  std::string endpoint_;
  std::string user_;
  UA_Client * client_ = nullptr;

  // -- Conversione in lettura

  std::string UA_StringConversion(const UA_String & string);

  KeyIntPairValue convertKeyIntPair(const UA_KeyIntPair & pair);

  KeyPosePairValue convertKeyPosePair(const UA_KeyPosePair & pair);
  
  ExecutionStatusValue convertExecutionStatus(const UA_ExecutionStatus & status);

  // -- Fine --

  // -- Conversione in scrittura

  UA_KeyIntPair convertToUaKeyIntPair(const KeyIntPairValue & pair);

  UA_KeyPosePair convertToUaKeyPosePair(const KeyPosePairValue & pair);

  // -- Fine --

  // -- Conversione Value -> UA_Variant -- 
  bool valueToUaVariant(const Value &value, UA_Variant &variant);

  // -- Conversione UA_Variant -> Value --
  bool UaVariantToValue(const UA_Variant &variant, Value &value);

  UA_NodeId TranslateBrowsePathtoNodeId(UA_Client * client, std::vector<std::string> browse_path);

  void writeKeyIntPair(UA_Client * client, std::string key, int value);

  UA_Int32 readKeyIntPair(UA_Client * client, std::string key);

  bool setEndpointAndUser(std::string endpoint, std::string user);


};


}


#endif
