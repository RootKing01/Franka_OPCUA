#ifndef FRANKA_OPCUA_BRIDGE__OPCUA_CLIENT_
#define FRANKA_OPCUA_BRIDGE__OPCUA_CLIENT_

#include "franka_opcua_bridge/interfaces/i_opcua_protocol_client.hpp"
#include "franka_opcua_bridge/opcua_value.hpp"
#include "franka_opcua_bridge/opcua_namespace_tree.hpp"

#include <open62541/client.h>
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>
#include <open62541/plugin/log_stdout.h>

#include <opc_ua_service_types_generated.h>
#include <stdlib.h>
#include <cstring>
#include <optional>

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
  std::optional<namespaceTree> exploreAddressSpace() override;

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

  NodeKind toNodeKind(UA_NodeClass nodeClass);
private:
  std::string endpoint_;
  std::string user_;
  UA_Client * client_ = nullptr;
  UA_DataTypeArray custom_data_types_{
    nullptr,
    UA_OPC_UA_SERVICE_TYPES_COUNT,
    UA_OPC_UA_SERVICE_TYPES
  };

  // -- Conversione in lettura

  std::string UA_StringConversion(const UA_String & string);

  KeyIntPairValue convertKeyIntPair(const UA_KeyIntPair & pair);

  KeyPosePairValue convertKeyPosePair(const UA_KeyPosePair & pair);

  ExecutionStatusValue convertExecutionStatus(const UA_ExecutionStatus & status);

  // -- Fine --

  // -- Conversione in scrittura

  UA_KeyIntPair convertToUaKeyIntPair(const KeyIntPairValue & pair);

  UA_KeyPosePair convertToUaKeyPosePair(const KeyPosePairValue & pair);

  UA_ExecutionStatus convertToUaExecutionStatus(const ExecutionStatusValue & execStatus);

  // -- Fine --

  // -- Conversione Value -> UA_Variant --
  bool valueToUaVariant(const Value & value, UA_Variant & variant);

  // -- Conversione UA_Variant -> Value --
  bool UaVariantToValue(const UA_Variant & variant, Value & value);

  bool TranslateBrowsePathtoNodeId(
    UA_Client * client,
    const std::vector<std::string> & browse_path,
    UA_NodeId & out_node_id);

  void writeKeyIntPair(UA_Client * client, std::string key, int value);

  UA_Int32 readKeyIntPair(UA_Client * client, std::string key);

  bool setEndpointAndUser(std::string endpoint, std::string user);


};


}


#endif
