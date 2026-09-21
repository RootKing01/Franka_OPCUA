#include "franka_opcua_bridge/opcua_client.hpp"
#include <open62541/client.h>
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>
#include <open62541/plugin/log_stdout.h>

#include <opc_ua_service_types_generated.h>
#include "franka_opcua_bridge/opcua_value.hpp"

#include <string>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <cstdlib>


namespace franka_opcua_bridge
{

bool OpcuaClient::setEndpointAndUser(std::string endpoint, std::string user)
{

  //Punto da controllare: se char * occupa un'aria di memoria troppo grande per string?
  //E' possibile che possa rappresentare un'area di memoria troppo grande? .c_str() esegue questo tipo di controlli?

  char * env_endpoint = getenv(endpoint.c_str());

  if (env_endpoint == nullptr) {

    std::cout << "Errore nella lettura dell'endpoint.";

    return false;
  }

  endpoint_ = std::string(env_endpoint);

  char * env_user = getenv(user.c_str());

  if (env_user == nullptr) {

    std::cout << "Errore nella lettura dello username.";

    return false;
  }

  user_ = std::string(env_user);

  std::cout << "Endpoint ed username inizializzati correttamente dalle variabili d'ambiente.";
  return true;

}

bool OpcuaClient::connect()
{

  char * env_password = nullptr;

  bool setted = setEndpointAndUser("FRANKA_ENDPOINT", "FRANKA_USER");

  if (!setted) {return false;}


  //Alloca il client
  client_ = UA_Client_new();

  //configurazione timeout/buffer di default
  UA_ClientConfig_setDefault(UA_Client_getConfig(client_));

  env_password = getenv("FRANKA_PASS");

  if (env_password == nullptr) {
    std::cout << "Errore nel recupero della password da variabile d'ambiente.";
    UA_Client_delete(client_);
    return false;
  }

  UA_StatusCode success = UA_Client_connectUsername(
    client_, endpoint_.c_str(),
    user_.c_str(), env_password);

  //Pulizia passsword
  explicit_bzero(env_password, strlen(env_password));

  if (success != UA_STATUSCODE_GOOD) {

    UA_Client_delete(client_);
    return false;

  }

  return true;

}


bool OpcuaClient::disconnect()
{

  if (client_ == nullptr) {return true;}

  UA_Client_disconnect(client_);
  UA_Client_delete(client_);

  client_ = nullptr;

  return true;
}

bool OpcuaClient::isConnected() const
{
  return client_ != nullptr;
}

CallResult OpcuaClient::callMethod(
  const std::vector<std::string> & object_browse_path,
  const std::string & method_name,
  const std::vector<Value> & args)
{
  (void)object_browse_path;
  (void)method_name;
  (void)args;
  // TODO: non ancora implementato
  return CallResult{};
}

bool OpcuaClient::readValue(
  const std::vector<std::string> & variable_browse_path,
  Value & out_value)
{
  (void)variable_browse_path;
  (void)out_value;
  // TODO: non ancora implementato
  return false;
}

bool OpcuaClient::writeValue(
  const std::vector<std::string> & variable_browse_path,
  const Value & out_value)
{
  (void)variable_browse_path;
  (void)out_value;
  // TODO: non ancora implementato
  return false;
}

OpcuaClient::~OpcuaClient()
{
  disconnect();
}


/* Code from Franka Emika OPC UA Service 7.0.1 - Simple C++ Client

UA_NodeId TranslateBrowsePathtoNodeId(UA_Client * client, std::vector<std::string> browse_path)
{
  UA_BrowsePath ua_browse_path;
  UA_BrowsePath_init(&ua_browse_path);
  ua_browse_path.startingNode = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
  ua_browse_path.relativePath.elements = (UA_RelativePathElement *)UA_Array_new(
    browse_path.size(), &UA_TYPES[UA_TYPES_RELATIVEPATHELEMENT]);
  ua_browse_path.relativePath.elementsSize = browse_path.size();

  for (size_t i = 0; i < browse_path.size(); i++) {
    UA_RelativePathElement * elem = &ua_browse_path.relativePath.elements[i];
    elem->targetName = UA_QUALIFIEDNAME_ALLOC(2, browse_path[i].c_str());
  }

  UA_TranslateBrowsePathsToNodeIdsRequest request;
  UA_TranslateBrowsePathsToNodeIdsRequest_init(&request);
  request.browsePaths = &ua_browse_path;
  request.browsePathsSize = 1;

  UA_TranslateBrowsePathsToNodeIdsResponse response =
    UA_Client_Service_translateBrowsePathsToNodeIds(client, request);

  UA_NodeId node_id = response.results[0].targets[0].targetId.nodeId;

  UA_BrowsePath_deleteMembers(&ua_browse_path);
  UA_TranslateBrowsePathsToNodeIdsResponse_deleteMembers(&response);

  return node_id;
}

void writeKeyIntPair(UA_Client * client, std::string key, int value)
{
  UA_NodeId object_id = TranslateBrowsePathtoNodeId(
    client, std::vector<std::string>{"Robot",
      "KeyValueMaps", "KeyIntMap"});
  UA_NodeId replace_id = TranslateBrowsePathtoNodeId(
    client, std::vector<std::string>{"Robot",
      "KeyValueMaps", "KeyIntMap", "Replace"});

  UA_String argString = UA_String_fromChars(const_cast<char *>(key.c_str()));
  UA_KeyIntPair my_value;
  my_value.key = argString;
  my_value.value = value;

  UA_ExtensionObject eo;
  UA_ExtensionObject_init(&eo);
  eo.encoding = UA_EXTENSIONOBJECT_DECODED;
  eo.content.decoded.data = &my_value;
  eo.content.decoded.type = &OPC_UA_SERVICE_TYPES[OPC_UA_SERVICE_TYPES_KEYINTPAIR];

  UA_Variant input;
  UA_Variant_init(&input);
  UA_Variant_setScalarCopy(&input, &eo, &UA_TYPES[UA_TYPES_EXTENSIONOBJECT]);

  UA_Variant * output;
  size_t outputSize;
  UA_StatusCode retval = UA_Client_call(
    client, object_id, replace_id, 1, &input, &outputSize,
    &output);
  if (retval != UA_STATUSCODE_GOOD) {
    UA_LOG_INFO(
      UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
      "[OPC UA Client] WriteKeyIntPair: Method call was unsuccessful!");
    return;
  }
  UA_Array_delete(output, outputSize, &UA_TYPES[UA_TYPES_VARIANT]);
}

UA_Int32 readKeyIntPair(UA_Client * client, std::string key)
{
  UA_NodeId object_id = TranslateBrowsePathtoNodeId(
    client, std::vector<std::string>{"Robot",
      "KeyValueMaps", "KeyIntMap"});
  UA_NodeId read_id = TranslateBrowsePathtoNodeId(
    client, std::vector<std::string>{"Robot",
      "KeyValueMaps", "KeyIntMap", "Read"});

  UA_String argString = UA_String_fromChars(const_cast<char *>(key.c_str()));
  UA_KeyIntPair ret;

  UA_Variant input;
  UA_Variant_init(&input);
  UA_Variant_setScalarCopy(&input, &argString, &UA_TYPES[UA_TYPES_STRING]);

  UA_Variant * output;
  size_t outputSize;
  UA_StatusCode retval =
    UA_Client_call(client, object_id, read_id, 1, &input, &outputSize, &output);
  if (retval != UA_STATUSCODE_GOOD) {
    UA_LOG_INFO(
      UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
      "[OPC UA Client] WriteKeyIntPair: Method call was unsuccessful!");
    return 0;
  }

  UA_Int32 value = *(static_cast<UA_Int32 *>(output->data));
  UA_Array_delete(output, outputSize, &UA_TYPES[UA_TYPES_VARIANT]);

  return value;
}

*/


}
