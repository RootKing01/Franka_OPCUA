#include "franka_opcua_bridge/opcua_client.hpp"
#include <open62541/ua_client.h>
#include <open62541/ua_config_default.h>
#include <open62541/ua_client_highlevel.h>
#include <open62541/ua_log_stdout.h>

#include <opc_ua_service_types_generated.h>

#include <string>
#include <vector>

#include <stdlib.h>

namespace franka_opcua_bridge
{

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

}
