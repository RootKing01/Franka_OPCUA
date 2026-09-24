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
#include <algorithm>


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
    client_ = nullptr;
    return false;
  }

  size_t pw_len = strlen(env_password);
  std::vector<char> pwd_copy(env_password, env_password+pw_len);

  UA_StatusCode success = UA_Client_connectUsername(
    client_, endpoint_.c_str(),
    user_.c_str(), pwd_copy.data());

  //Pulizia passsword
  explicit_bzero(pwd_copy.data(), pwd_copy.size());

  if (success != UA_STATUSCODE_GOOD) {

    UA_Client_delete(client_);
    client_ = nullptr;
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

  
  UA_NodeId object_id = TranslateBrowsePathtoNodeId(client_, object_browse_path);

  std::vector<std::string> method_path = object_browse_path;
  method_path.push_back(method_name);

  UA_NodeId method_id = TranslateBrowsePathtoNodeId(client_, method_path);

  size_t inputSize = args.size();

  UA_Variant* inputs = static_cast<UA_Variant*>(UA_Array_new(args.size(), &UA_TYPES[UA_TYPES_VARIANT]));

  for(size_t i = 0; i < inputSize; i++)
  {
    bool success = valueToUaVariant(args[i], inputs[i]);

    if (!success)
    {
      UA_Array_delete(inputs, inputSize, &UA_TYPES[UA_TYPES_VARIANT]);
      return CallResult{};
    }
  }

  size_t outputSize = 0;
  UA_Variant *output = nullptr;
  CallResult result;

  UA_StatusCode status = UA_Client_call(client_, object_id, method_id, inputSize, inputs, &outputSize, &output);

  if (status != UA_STATUSCODE_GOOD)
  {
    UA_Array_delete(inputs, inputSize, &UA_TYPES[UA_TYPES_VARIANT]);
    return CallResult{};
  } 

  result.ok = true;
  
  for (size_t i = 0; i < outputSize; ++i)
  {
    Value tmp;
    bool success = UaVariantToValue(output[i], tmp);

    if (!success)
    {
      UA_Array_delete(output, outputSize, &UA_TYPES[UA_TYPES_VARIANT]);
      UA_Array_delete(inputs, inputSize, &UA_TYPES[UA_TYPES_VARIANT]);
      return CallResult{};
    } 

    result.output_values.emplace_back(tmp);
  }

  UA_Array_delete(output, outputSize, &UA_TYPES[UA_TYPES_VARIANT]);
  UA_Array_delete(inputs, inputSize, &UA_TYPES[UA_TYPES_VARIANT]);


  return result;

}

bool OpcuaClient::readValue(
  const std::vector<std::string> & variable_browse_path,
  Value & out_value)
{ 
  UA_Variant variantOutput;
  UA_Variant_init(&variantOutput);

  UA_NodeId nodePath_id = TranslateBrowsePathtoNodeId(client_, variable_browse_path);

  UA_StatusCode status = UA_Client_readValueAttribute(client_, nodePath_id, &variantOutput);

  if (status != UA_STATUSCODE_GOOD) return false;

  bool success = UaVariantToValue(variantOutput, out_value);
  UA_Variant_clear(&variantOutput);

  return success;
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

// -- Conversione Value -> UA_Variant --

bool OpcuaClient::valueToUaVariant(const Value &value, UA_Variant &variant)
{
   if (value.is<bool>())
    {
    
      bool boolValue = value.as<bool>();
      return UA_Variant_setScalarCopy(&variant, &boolValue, &UA_TYPES[UA_TYPES_BOOLEAN]) == UA_STATUSCODE_GOOD;
      
    }
    else if (value.is<int32_t>())
    {
      int32_t intValue = value.as<int32_t>();
      return UA_Variant_setScalarCopy(&variant, &intValue, &UA_TYPES[UA_TYPES_INT32]) == UA_STATUSCODE_GOOD;

    }
    else if (value.is<double>())
    {
      double DoubleValue = value.as<double>();
      return UA_Variant_setScalarCopy(&variant, &DoubleValue, &UA_TYPES[UA_TYPES_DOUBLE]) == UA_STATUSCODE_GOOD;

    }
    else
    {
      if (value.is<std::string>())
      {
        UA_String tmp = UA_String_fromChars(value.as<std::string>().c_str());
        UA_StatusCode status = UA_Variant_setScalarCopy(&variant, &tmp, &UA_TYPES[UA_TYPES_STRING]);
        UA_String_clear(&tmp);

        return status == UA_STATUSCODE_GOOD;

      }
      else if (value.is<std::vector<double>>())
      {
        const auto &VectorValues = value.as<std::vector<double>>();
        UA_Double * tmp = static_cast<UA_Double*>(UA_Array_new(VectorValues.size(), &UA_TYPES[UA_TYPES_DOUBLE]));

        if (tmp == nullptr && !VectorValues.empty()) return false;

        for (size_t j = 0; j < VectorValues.size(); ++j)
        {
          tmp[j] = VectorValues[j];
        }

        UA_StatusCode status = UA_Variant_setArrayCopy(&variant, tmp, VectorValues.size(), &UA_TYPES[UA_TYPES_DOUBLE]);

        UA_Array_delete(tmp, VectorValues.size(), &UA_TYPES[UA_TYPES_DOUBLE]);

        return status == UA_STATUSCODE_GOOD;
      }
    }

    return false;
}

// -- Conversione UA_Variant -> Value --

bool OpcuaClient::UaVariantToValue(const UA_Variant &variant, Value &value)
{

  if (variant.type == &UA_TYPES[UA_TYPES_BOOLEAN])
    {
      value = Value(*static_cast<UA_Boolean*>(variant.data));
      return true;
    }
    
  if (variant.type == &UA_TYPES[UA_TYPES_INT32])
  {
    value = Value(*static_cast<UA_Int32*>(variant.data));
    return true;
  }
    
  if (variant.type == &UA_TYPES[UA_TYPES_DOUBLE] && UA_Variant_isScalar(&variant))
  {
    value = Value(*static_cast<UA_Double*>(variant.data));
    return true;
  }
    
  if (variant.type == &UA_TYPES[UA_TYPES_STRING])
  {
    UA_String * strValue = static_cast<UA_String*>(variant.data);
      
    std::string outputString(reinterpret_cast<char*>(strValue->data), strValue->length);
  
    value = Value(outputString);

    return true;
  }
    
  if (variant.type == &UA_TYPES[UA_TYPES_DOUBLE] && !UA_Variant_isScalar(&variant))
  {
    UA_Double * data = static_cast<UA_Double*>(variant.data);

    std::vector<double> doubleValues(data, data + variant.arrayLength);

    value = Value(doubleValues);
    return true;
  }

  if (variant.type == &UA_TYPES[UA_TYPES_EXTENSIONOBJECT] && UA_Variant_isScalar(&variant))
  {
    UA_ExtensionObject *extension = static_cast<UA_ExtensionObject*>(variant.data);

    if (extension->encoding == UA_EXTENSIONOBJECT_DECODED || extension->encoding == UA_EXTENSIONOBJECT_DECODED_NODELETE)
    {
      if (extension->content.decoded.type == &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYINTPAIR])
      {
        UA_KeyIntPair *intKeyPair = static_cast<UA_KeyIntPair *>(extension->content.decoded.data);

        value = Value(convertKeyIntPair(*intKeyPair));

        return true;

      }
      else if (extension->content.decoded.type == &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYPOSEPAIR])
      {
        UA_KeyPosePair * keyPose = static_cast<UA_KeyPosePair*>(extension->content.decoded.data);

        value = Value(convertKeyPosePair(*keyPose));
        
        return true;

      }
      else if (extension->content.decoded.type == &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_EXECUTIONSTATUS])
      {
        UA_ExecutionStatus * execStatus = static_cast<UA_ExecutionStatus*>(extension->content.decoded.data);

        value = Value(convertExecutionStatus(*execStatus));

        return true;

      }
      else
      {
        return false;
      }
    }
  }

    return false;

}



// -- Conversioni in lettura --

std::string OpcuaClient::UA_StringConversion(const UA_String & string){

  if (string.data == nullptr ) return "";

  return std::string(reinterpret_cast<char*>(string.data), string.length);

 }

 KeyIntPairValue OpcuaClient::convertKeyIntPair(const UA_KeyIntPair & pair){

  return KeyIntPairValue{UA_StringConversion(pair.key), pair.value};

 }

 KeyPosePairValue OpcuaClient::convertKeyPosePair(const UA_KeyPosePair & pair){

  if (pair.value != nullptr)
  {
    return KeyPosePairValue{UA_StringConversion(pair.key), std::vector<double>(pair.value, pair.value + pair.valueSize)};
  }
  
  return KeyPosePairValue{UA_StringConversion(pair.key), std::vector<double>{}};
 
}

ExecutionStatusValue OpcuaClient::convertExecutionStatus(const UA_ExecutionStatus & status){

    return ExecutionStatusValue{status.hasError, 
                                status.isRunning, 
                                UA_StringConversion(status.errorMessage), 
                                UA_StringConversion(status.activeTaskName),
                                UA_StringConversion(status.activeTaskId)};

}

// -- Fine conversioni in lettura --


// -- Conversione in scrittura --

UA_KeyIntPair OpcuaClient::convertToUaKeyIntPair(const KeyIntPairValue & pair){

  UA_KeyIntPair result;
  result.key = UA_String_fromChars(pair.key.c_str());
  result.value = pair.value;

  return result;

}


UA_KeyPosePair OpcuaClient::convertToUaKeyPosePair(const KeyPosePairValue & pair){

  UA_KeyPosePair result;
  result.key = UA_String_fromChars(pair.key.c_str());
  result.valueSize = pair.value.size();

  //Nota il static_cast: UA_Array_new ritorna void* (è generica, funziona per qualunque tipo), quindi va castata al tipo puntatore giusto — qui serve un cast 
  //esplicito, ma static_cast (non reinterpret_cast) perché convertiamo da/verso void*, un caso che il compilatore sa gestire in modo "controllato" 
  //(è la conversione standard prevista per puntatori generici, diversa dal caso uint8_t*↔char* di prima).

  result.value = static_cast<UA_Double *>( UA_Array_new(pair.value.size(), &UA_TYPES[UA_TYPES_DOUBLE]));

  if (result.value == nullptr)
  {
    result.valueSize = 0;
    
    // O setto a 0 la dimensione per indicare l'errore durante l'allocazione, oppure sollevo eccezione
    //throw std::bad_alloc();
  }
  else 
  {
    std::copy(pair.value.begin(), pair.value.end(), result.value);
  }

  return result;
}

// -- Fine conversione in scrittura --

OpcuaClient::~OpcuaClient()
{
  disconnect();
}

UA_NodeId OpcuaClient::TranslateBrowsePathtoNodeId(UA_Client * client, std::vector<std::string> browse_path)
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

  UA_BrowsePath_clear(&ua_browse_path);
  UA_TranslateBrowsePathsToNodeIdsResponse_clear(&response);

  return node_id;
}

/*
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
