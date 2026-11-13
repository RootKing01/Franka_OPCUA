#include "franka_opcua_bridge/opcua_client.hpp"
#include <open62541/client.h>
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>
#include <open62541/plugin/log_stdout.h>

#include <opc_ua_service_types_generated.h>
#include "franka_opcua_bridge/opcua_value.hpp"
#include "ua_types_encoding_binary.h"

#include <string>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <cstdlib>
#include <algorithm>
#include <functional>
#include <set>
#include <utility>


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

  std::cerr << "[OPCUA] connect() eseguita" << std::endl;

  char * env_password = nullptr;

  bool setted = setEndpointAndUser("FRANKA_ENDPOINT", "FRANKA_USER");

  if (!setted) {return false;}


  //Alloca il client
  client_ = UA_Client_new();

  //configurazione timeout/buffer di default
  UA_ClientConfig * config = UA_Client_getConfig(client_);

  UA_ClientConfig_setDefault(config);

  config->customDataTypes = &custom_data_types_;

  std::cerr << "[OPCUA] custom types registrati: "
            << config->customDataTypes
            << ", count="
            << config->customDataTypes->typesSize
            << std::endl;

  config->timeout = 10000;

  env_password = getenv("FRANKA_PASS");

  if (env_password == nullptr) {
    std::cout << "Errore nel recupero della password da variabile d'ambiente.";
    UA_Client_delete(client_);
    client_ = nullptr;
    return false;
  }

  size_t pw_len = strlen(env_password);
  std::vector<char> pwd_copy(env_password, env_password + pw_len + 1);

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


  UA_NodeId object_id;
  UA_NodeId_init(&object_id);

  if (!TranslateBrowsePathtoNodeId(client_, object_browse_path, object_id)) {return CallResult{};}

  std::vector<std::string> method_path = object_browse_path;
  method_path.push_back(method_name);

  UA_NodeId method_id;
  UA_NodeId_init(&method_id);

  if (!TranslateBrowsePathtoNodeId(client_, method_path, method_id)) {
    UA_NodeId_clear(&object_id);
    return CallResult{};
  }

  size_t inputSize = args.size();

  UA_Variant * inputs =
    static_cast<UA_Variant *>(UA_Array_new(args.size(), &UA_TYPES[UA_TYPES_VARIANT]));

  if (inputs == nullptr && inputSize > 0) {
    UA_NodeId_clear(&object_id);
    UA_NodeId_clear(&method_id);
    return CallResult{};
  }

  for (size_t i = 0; i < inputSize; i++) {
    bool success = valueToUaVariant(args[i], inputs[i]);

    if (!success) {
      UA_Array_delete(inputs, inputSize, &UA_TYPES[UA_TYPES_VARIANT]);
      UA_NodeId_clear(&object_id);
      UA_NodeId_clear(&method_id);
      return CallResult{};
    }
  }

  size_t outputSize = 0;
  UA_Variant * output = nullptr;
  CallResult result;

  UA_StatusCode status = UA_Client_call(
    client_, object_id, method_id, inputSize, inputs,
    &outputSize, &output);

  if (status != UA_STATUSCODE_GOOD) {
    std::cout << "[OPCUA] UA_Client_call failed: " << UA_StatusCode_name(status) << "(" << status <<
      ")" << std::endl;
    result.error_message = std::string(UA_StatusCode_name(status)) + "(" +
      std::to_string(status) + ")";

    UA_Array_delete(inputs, inputSize, &UA_TYPES[UA_TYPES_VARIANT]);
    UA_NodeId_clear(&object_id);
    UA_NodeId_clear(&method_id);

    return result;
  }

  result.ok = true;

  for (size_t i = 0; i < outputSize; ++i) {
    Value tmp;
    bool success = UaVariantToValue(output[i], tmp);

    if (!success) {
      UA_Array_delete(output, outputSize, &UA_TYPES[UA_TYPES_VARIANT]);
      UA_Array_delete(inputs, inputSize, &UA_TYPES[UA_TYPES_VARIANT]);
      UA_NodeId_clear(&object_id);
      UA_NodeId_clear(&method_id);
      return CallResult{};
    }

    result.output_values.emplace_back(tmp);
  }

  UA_Array_delete(output, outputSize, &UA_TYPES[UA_TYPES_VARIANT]);
  UA_Array_delete(inputs, inputSize, &UA_TYPES[UA_TYPES_VARIANT]);

  UA_NodeId_clear(&object_id);
  UA_NodeId_clear(&method_id);


  return result;

}

bool OpcuaClient::readValue(
  const std::vector<std::string> & variable_browse_path,
  Value & out_value)
{
  UA_Variant variantOutput;
  UA_Variant_init(&variantOutput);

  UA_NodeId nodePath_id;
  UA_NodeId_init(&nodePath_id);

  if (!TranslateBrowsePathtoNodeId(client_, variable_browse_path, nodePath_id)) {return false;}

  UA_StatusCode status = UA_Client_readValueAttribute(client_, nodePath_id, &variantOutput);

  std::cerr << "[OPCUA] readValue: "
            << UA_StatusCode_name(status)
            << " (" << status << ")"
            << std::endl;

  if (status != UA_STATUSCODE_GOOD) {
    UA_NodeId_clear(&nodePath_id);
    return false;
  }

  if (variantOutput.type == &UA_TYPES[UA_TYPES_EXTENSIONOBJECT] &&
    UA_Variant_isScalar(&variantOutput))
  {
    const auto * extension = static_cast<const UA_ExtensionObject *>(variantOutput.data);

    std::cerr << "[OPCUA] ExtensionObject encoding="
              << static_cast<int>(extension->encoding) << std::endl;

    if (extension->encoding == UA_EXTENSIONOBJECT_ENCODED_BYTESTRING) {
      const UA_NodeId & type_id = extension->content.encoded.typeId;
      std::cerr << "[OPCUA] typeId namespace=" << type_id.namespaceIndex;
      if (type_id.identifierType == UA_NODEIDTYPE_NUMERIC) {
        std::cerr << ", numeric=" << type_id.identifier.numeric;
      }
      std::cerr << std::endl;
    }
  }

  bool success = UaVariantToValue(variantOutput, out_value);

  std::cerr << "[OPCUA] UaVariantToValue success=" << success << std::endl;
  UA_Variant_clear(&variantOutput);

  UA_NodeId_clear(&nodePath_id);

  return success;
}

bool OpcuaClient::writeValue(
  const std::vector<std::string> & variable_browse_path,
  const Value & out_value)
{

  UA_Variant variantInput;
  UA_Variant_init(&variantInput);

  UA_NodeId nodePath_id;
  UA_NodeId_init(&nodePath_id);

  if (!TranslateBrowsePathtoNodeId(client_, variable_browse_path, nodePath_id)) {return false;}

  bool conversion = valueToUaVariant(out_value, variantInput);

  if (!conversion) {
    UA_NodeId_clear(&nodePath_id);
    return false;
  }

  UA_StatusCode status = UA_Client_writeValueAttribute(client_, nodePath_id, &variantInput);

  UA_Variant_clear(&variantInput);
  UA_NodeId_clear(&nodePath_id);

  return status == UA_STATUSCODE_GOOD;
}

// -- Conversione Value -> UA_Variant --

bool OpcuaClient::valueToUaVariant(const Value & value, UA_Variant & variant)
{
  if (value.is<bool>()) {

    bool boolValue = value.as<bool>();
    return UA_Variant_setScalarCopy(
      &variant, &boolValue,
      &UA_TYPES[UA_TYPES_BOOLEAN]) == UA_STATUSCODE_GOOD;

  } else if (value.is<int32_t>()) {
    int32_t intValue = value.as<int32_t>();
    return UA_Variant_setScalarCopy(
      &variant, &intValue,
      &UA_TYPES[UA_TYPES_INT32]) == UA_STATUSCODE_GOOD;

  } else if (value.is<double>()) {
    double DoubleValue = value.as<double>();
    return UA_Variant_setScalarCopy(
      &variant, &DoubleValue,
      &UA_TYPES[UA_TYPES_DOUBLE]) == UA_STATUSCODE_GOOD;

  } else {
    if (value.is<std::string>()) {
      UA_String tmp = UA_String_fromChars(value.as<std::string>().c_str());
      UA_StatusCode status = UA_Variant_setScalarCopy(&variant, &tmp, &UA_TYPES[UA_TYPES_STRING]);
      UA_String_clear(&tmp);

      return status == UA_STATUSCODE_GOOD;

    } else if (value.is<std::vector<double>>()) {
      const auto & VectorValues = value.as<std::vector<double>>();
      UA_Double * tmp =
        static_cast<UA_Double *>(UA_Array_new(VectorValues.size(), &UA_TYPES[UA_TYPES_DOUBLE]));

      if (tmp == nullptr && !VectorValues.empty()) {return false;}

      for (size_t j = 0; j < VectorValues.size(); ++j) {
        tmp[j] = VectorValues[j];
      }

      UA_StatusCode status =
        UA_Variant_setArrayCopy(&variant, tmp, VectorValues.size(), &UA_TYPES[UA_TYPES_DOUBLE]);

      UA_Array_delete(tmp, VectorValues.size(), &UA_TYPES[UA_TYPES_DOUBLE]);

      return status == UA_STATUSCODE_GOOD;
    } else if (value.is<KeyIntPairValue>()) {
      UA_KeyIntPair uaKeyInt = convertToUaKeyIntPair(value.as<KeyIntPairValue>());

      UA_ExtensionObject uaKeyIntExt;
      UA_ExtensionObject_init(&uaKeyIntExt);


      uaKeyIntExt.encoding = UA_EXTENSIONOBJECT_DECODED;
      uaKeyIntExt.content.decoded.type =
        &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYINTPAIR];
      uaKeyIntExt.content.decoded.data = &uaKeyInt;

      UA_StatusCode status =
        UA_Variant_setScalarCopy(&variant, &uaKeyIntExt, &UA_TYPES[UA_TYPES_EXTENSIONOBJECT]);

      UA_clear(&uaKeyInt, &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYINTPAIR]);

      return status == UA_STATUSCODE_GOOD;

    } else if (value.is<KeyPosePairValue>()) {
      UA_KeyPosePair uaKeyPose = convertToUaKeyPosePair(value.as<KeyPosePairValue>());

      UA_ExtensionObject uaKeyPoseExt;
      UA_ExtensionObject_init(&uaKeyPoseExt);


      uaKeyPoseExt.encoding = UA_EXTENSIONOBJECT_DECODED;
      uaKeyPoseExt.content.decoded.type =
        &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYPOSEPAIR];
      uaKeyPoseExt.content.decoded.data = &uaKeyPose;

      UA_StatusCode status =
        UA_Variant_setScalarCopy(&variant, &uaKeyPoseExt, &UA_TYPES[UA_TYPES_EXTENSIONOBJECT]);

      UA_clear(&uaKeyPose, &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYPOSEPAIR]);

      return status == UA_STATUSCODE_GOOD;

    } else if (value.is<ExecutionStatusValue>()) {
      UA_ExecutionStatus uaExecStat = convertToUaExecutionStatus(value.as<ExecutionStatusValue>());

      UA_ExtensionObject uaExecStatExt;
      UA_ExtensionObject_init(&uaExecStatExt);

      uaExecStatExt.encoding = UA_EXTENSIONOBJECT_DECODED;
      uaExecStatExt.content.decoded.type =
        &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_EXECUTIONSTATUS];
      uaExecStatExt.content.decoded.data = &uaExecStat;

      UA_StatusCode status =
        UA_Variant_setScalarCopy(&variant, &uaExecStatExt, &UA_TYPES[UA_TYPES_EXTENSIONOBJECT]);

      UA_clear(&uaExecStat, &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_EXECUTIONSTATUS]);

      return status == UA_STATUSCODE_GOOD;
    }

  }

  return false;
}

// -- Conversione UA_Variant -> Value --


bool OpcuaClient::UaVariantToValue(const UA_Variant & variant, Value & value)
{

  std::cerr << "[OPCUA] UaVariantToValue: type="
            << (variant.type != nullptr ? variant.type->typeName : "null")
            << ", scalar=" << UA_Variant_isScalar(&variant)
            << ", arrayLength=" << variant.arrayLength
            << ", data=" << variant.data << std::endl;

  if (variant.type == &UA_TYPES[UA_TYPES_EXTENSIONOBJECT] &&
    variant.data != nullptr)
  {
    const auto * extension = static_cast<const UA_ExtensionObject *>(variant.data);
    std::cerr << "[OPCUA] ExtensionObject encoding="
              << static_cast<int>(extension->encoding) << std::endl;
  }

  if (variant.type == &UA_TYPES[UA_TYPES_BOOLEAN]) {
    value = Value(*static_cast<UA_Boolean *>(variant.data));
    return true;
  }

  if (variant.type == &UA_TYPES[UA_TYPES_INT32]) {
    value = Value(*static_cast<UA_Int32 *>(variant.data));
    return true;
  }

  if (variant.type == &UA_TYPES[UA_TYPES_DOUBLE] && UA_Variant_isScalar(&variant)) {
    value = Value(*static_cast<UA_Double *>(variant.data));
    return true;
  }

  if (variant.type == &UA_TYPES[UA_TYPES_STRING]) {
    if (variant.data == nullptr) {
      return false;
    }

    if (UA_Variant_isScalar(&variant)) {
      const auto * ua_string = static_cast<const UA_String *>(variant.data);
      value = Value(UA_StringConversion(*ua_string));
      return true;
    }

    const auto * ua_strings = static_cast<const UA_String *>(variant.data);
    std::vector<std::string> task_ids;
    task_ids.reserve(variant.arrayLength);

    for (size_t i = 0; i < variant.arrayLength; ++i) {
      task_ids.push_back(UA_StringConversion(ua_strings[i]));
    }

    value = Value(std::move(task_ids));
    return true;
  }

  if (variant.type == &UA_TYPES[UA_TYPES_DOUBLE] && !UA_Variant_isScalar(&variant)) {
    UA_Double * data = static_cast<UA_Double *>(variant.data);

    std::vector<double> doubleValues(data, data + variant.arrayLength);

    value = Value(doubleValues);
    return true;
  }

  if (variant.data != nullptr && UA_Variant_isScalar(&variant)) {
    if (variant.type == &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYINTPAIR]) {
      value = Value(
        convertKeyIntPair(
          *static_cast<const UA_KeyIntPair *>(variant.data)));
      return true;
    }

    if (variant.type == &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYPOSEPAIR]) {
      value = Value(
        convertKeyPosePair(
          *static_cast<const UA_KeyPosePair *>(variant.data)));
      return true;
    }

    if (variant.type == &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_EXECUTIONSTATUS]) {
      value = Value(
        convertExecutionStatus(
          *static_cast<const UA_ExecutionStatus *>(variant.data)));
      return true;
    }
  }

  if (variant.type == &UA_TYPES[UA_TYPES_EXTENSIONOBJECT] && UA_Variant_isScalar(&variant)) {
    UA_ExtensionObject * extension = static_cast<UA_ExtensionObject *>(variant.data);

    if (extension->encoding == UA_EXTENSIONOBJECT_DECODED ||
      extension->encoding == UA_EXTENSIONOBJECT_DECODED_NODELETE)
    {
      if (extension->content.decoded.type ==
        &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYINTPAIR])
      {
        UA_KeyIntPair * intKeyPair = static_cast<UA_KeyIntPair *>(extension->content.decoded.data);

        value = Value(convertKeyIntPair(*intKeyPair));

        return true;

      } else if (extension->content.decoded.type ==
        &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYPOSEPAIR])
      {
        UA_KeyPosePair * keyPose = static_cast<UA_KeyPosePair *>(extension->content.decoded.data);

        value = Value(convertKeyPosePair(*keyPose));

        return true;

      } else if (extension->content.decoded.type ==
        &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_EXECUTIONSTATUS])
      {
        UA_ExecutionStatus * execStatus =
          static_cast<UA_ExecutionStatus *>(extension->content.decoded.data);

        value = Value(convertExecutionStatus(*execStatus));

        return true;

      } else {
        return false;
      }
    }

    if (extension->encoding == UA_EXTENSIONOBJECT_ENCODED_BYTESTRING) {
      const UA_NodeId & encoding_id = extension->content.encoded.typeId;
      const UA_ByteString & body = extension->content.encoded.body;

      if (body.length == 0 || body.data == nullptr) {
        std::cerr << "[OPCUA] Encoded ExtensionObject has empty body" << std::endl;
        return false;
      }

      std::cerr << "[OPCUA] Encoded ExtensionObject typeId namespace="
                << encoding_id.namespaceIndex
                << ", numeric="
                << (encoding_id.identifierType == UA_NODEIDTYPE_NUMERIC ?
      encoding_id.identifier.numeric : 0)
                << ", bodyLength=" << body.length << std::endl;

      const UA_DataType * type = nullptr;
      for (size_t i = 0; i < UA_OPC_UA_SERVICE_TYPES_COUNT; ++i) {
        if (UA_NodeId_equal(
            &encoding_id,
            &UA_OPC_UA_SERVICE_TYPES[i].binaryEncodingId))
        {
          type = &UA_OPC_UA_SERVICE_TYPES[i];
          break;
        }
      }

      if (type == nullptr) {
        std::cerr << "[OPCUA] No custom type matches binaryEncodingId" << std::endl;
        return false;
      }

      if (type == &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYINTPAIR]) {
        UA_KeyIntPair decoded;
        UA_init(&decoded, type);
        size_t offset = 0;
        UA_StatusCode status = UA_decodeBinary(
          &body, &offset, &decoded, type, &custom_data_types_);

        std::cerr << "[OPCUA] Decoded KeyIntPair status="
                  << UA_StatusCode_name(status)
                  << ", offset=" << offset << std::endl;

        if (status == UA_STATUSCODE_GOOD && offset == body.length) {
          value = Value(convertKeyIntPair(decoded));
          UA_clear(&decoded, type);
          return true;
        }

        UA_clear(&decoded, type);
        return false;
      }

      if (type == &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_KEYPOSEPAIR]) {
        UA_KeyPosePair decoded;
        UA_init(&decoded, type);
        size_t offset = 0;
        UA_StatusCode status = UA_decodeBinary(
          &body, &offset, &decoded, type, &custom_data_types_);

        std::cerr << "[OPCUA] Decoded KeyPosePair status="
                  << UA_StatusCode_name(status)
                  << ", offset=" << offset << std::endl;

        if (status == UA_STATUSCODE_GOOD && offset == body.length) {
          value = Value(convertKeyPosePair(decoded));
          UA_clear(&decoded, type);
          return true;
        }

        UA_clear(&decoded, type);
        return false;
      }

      if (type == &UA_OPC_UA_SERVICE_TYPES[UA_OPC_UA_SERVICE_TYPES_EXECUTIONSTATUS]) {
        UA_ExecutionStatus decoded;
        UA_init(&decoded, type);
        size_t offset = 0;
        UA_StatusCode status = UA_decodeBinary(
          &body, &offset, &decoded, type, &custom_data_types_);

        std::cerr << "[OPCUA] Decoded ExecutionStatus status="
                  << UA_StatusCode_name(status)
                  << ", offset=" << offset << std::endl;

        if (status == UA_STATUSCODE_GOOD && offset == body.length) {
          value = Value(convertExecutionStatus(decoded));
          UA_clear(&decoded, type);
          return true;
        }

        UA_clear(&decoded, type);
        return false;
      }

      return false;
    }
  }

  return false;

}


// -- Conversioni in lettura --

std::string OpcuaClient::UA_StringConversion(const UA_String & string)
{

  if (string.data == nullptr) {return "";}

  return std::string(reinterpret_cast<char *>(string.data), string.length);

}

KeyIntPairValue OpcuaClient::convertKeyIntPair(const UA_KeyIntPair & pair)
{

  return KeyIntPairValue{UA_StringConversion(pair.key), pair.value};

}

KeyPosePairValue OpcuaClient::convertKeyPosePair(const UA_KeyPosePair & pair)
{

  if (pair.value != nullptr) {
    return KeyPosePairValue{UA_StringConversion(pair.key), std::vector<double>(
        pair.value,
        pair.value +
        pair.valueSize)};
  }

  return KeyPosePairValue{UA_StringConversion(pair.key), std::vector<double>{}};

}

ExecutionStatusValue OpcuaClient::convertExecutionStatus(const UA_ExecutionStatus & status)
{

  return ExecutionStatusValue{status.hasError,
    status.isRunning,
    UA_StringConversion(status.errorMessage),
    UA_StringConversion(status.activeTaskName),
    UA_StringConversion(status.activeTaskId)};

}

// -- Fine conversioni in lettura --


// -- Conversione in scrittura --

UA_KeyIntPair OpcuaClient::convertToUaKeyIntPair(const KeyIntPairValue & pair)
{

  UA_KeyIntPair result;
  result.key = UA_String_fromChars(pair.key.c_str());
  result.value = pair.value;

  return result;

}

UA_KeyPosePair OpcuaClient::convertToUaKeyPosePair(const KeyPosePairValue & pair)
{

  UA_KeyPosePair result;
  result.key = UA_String_fromChars(pair.key.c_str());
  result.valueSize = pair.value.size();

  //Nota il static_cast: UA_Array_new ritorna void* (è generica, funziona per qualunque tipo), quindi va castata al tipo puntatore giusto — qui serve un cast
  //esplicito, ma static_cast (non reinterpret_cast) perché convertiamo da/verso void*, un caso che il compilatore sa gestire in modo "controllato"
  //(è la conversione standard prevista per puntatori generici, diversa dal caso uint8_t*↔char* di prima).

  result.value =
    static_cast<UA_Double *>( UA_Array_new(pair.value.size(), &UA_TYPES[UA_TYPES_DOUBLE]));

  if (result.value == nullptr) {
    result.valueSize = 0;

    // O setto a 0 la dimensione per indicare l'errore durante l'allocazione, oppure sollevo eccezione
    //throw std::bad_alloc();
  } else {
    std::copy(pair.value.begin(), pair.value.end(), result.value);
  }

  return result;
}

UA_ExecutionStatus OpcuaClient::convertToUaExecutionStatus(const ExecutionStatusValue & execStatus)
{
  UA_ExecutionStatus status;

  status.hasError = static_cast<UA_Boolean>(execStatus.has_error);
  status.isRunning = static_cast<UA_Boolean>(execStatus.is_running);
  status.errorMessage = UA_String_fromChars(execStatus.error_message.c_str());
  status.activeTaskName = UA_String_fromChars(execStatus.active_task_name.c_str());
  status.activeTaskId = UA_String_fromChars(execStatus.active_task_id.c_str());

  return status;
}

// -- Fine conversione in scrittura --

OpcuaClient::~OpcuaClient()
{
  disconnect();
}

bool OpcuaClient::TranslateBrowsePathtoNodeId(
  UA_Client * client,
  const std::vector<std::string> & browse_path,
  UA_NodeId & out_node_id)
{
  if (client == nullptr || browse_path.empty()) {return false;}

  UA_BrowsePath ua_browse_path;
  UA_BrowsePath_init(&ua_browse_path);
  ua_browse_path.startingNode = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
  ua_browse_path.relativePath.elements = (UA_RelativePathElement *)UA_Array_new(
    browse_path.size(), &UA_TYPES[UA_TYPES_RELATIVEPATHELEMENT]);
  ua_browse_path.relativePath.elementsSize = browse_path.size();

  if (ua_browse_path.relativePath.elements == nullptr) {
    UA_BrowsePath_clear(&ua_browse_path);
    return false;
  }

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

  if (response.responseHeader.serviceResult != UA_STATUSCODE_GOOD ||
    response.resultsSize == 0 || response.results == nullptr ||
    response.results[0].statusCode != UA_STATUSCODE_GOOD ||
    response.results[0].targetsSize == 0 || response.results[0].targets == nullptr)
  {
    UA_BrowsePath_clear(&ua_browse_path);
    UA_TranslateBrowsePathsToNodeIdsResponse_clear(&response);
    return false;
  }

  UA_StatusCode status = UA_NodeId_copy(
    &response.results[0].targets[0].targetId.nodeId,
    &out_node_id);

  if (status != UA_STATUSCODE_GOOD) {
    UA_NodeId_clear(&out_node_id);
    UA_BrowsePath_clear(&ua_browse_path);
    UA_TranslateBrowsePathsToNodeIdsResponse_clear(&response);
    return false;
  }

  UA_BrowsePath_clear(&ua_browse_path);
  UA_TranslateBrowsePathsToNodeIdsResponse_clear(&response);

  return true;
}

NodeKind OpcuaClient::toNodeKind(UA_NodeClass nodeClass)
{
  switch (nodeClass) {
    case UA_NODECLASS_UNSPECIFIED:
      return NodeKind::Unspecified;
    case UA_NODECLASS_OBJECT:
      return NodeKind::Object;
    case UA_NODECLASS_VARIABLE:
      return NodeKind::Variable;
    case UA_NODECLASS_METHOD:
      return NodeKind::Method;
    case UA_NODECLASS_OBJECTTYPE:
      return NodeKind::ObjectType;
    case UA_NODECLASS_VARIABLETYPE:
      return NodeKind::VariableType;
    case UA_NODECLASS_REFERENCETYPE:
      return NodeKind::ReferenceType;
    case UA_NODECLASS_DATATYPE:
      return NodeKind::DataType;
    case UA_NODECLASS_VIEW:
      return NodeKind::View;
    default:
      return NodeKind::Unknown;
  }
}


std::optional<namespaceTree> OpcuaClient::exploreAddressSpace()
{
  if (client_ == nullptr) {
    std::cerr << "[OPCUA browse] Impossibile iniziare: client non connesso o nullo"
              << std::endl;
    return std::nullopt;
  }

  std::cerr << "[OPCUA browse] Avvio esplorazione da RootFolder (ns=0;i=84)"
            << std::endl;

  auto nodeIdToString = [](const UA_NodeId & node_id) {
      UA_String encoded;
      UA_String_init(&encoded);
      const UA_StatusCode status = UA_NodeId_print(&node_id, &encoded);
      if (status != UA_STATUSCODE_GOOD) {
        UA_String_clear(&encoded);
        return std::string{};
      }

      std::string result(reinterpret_cast<const char *>(encoded.data), encoded.length);
      UA_String_clear(&encoded);
      return result;
    };

  std::set<std::string> visited;
  std::function<bool(const UA_NodeId &, namespaceTree &)> browseNode;
  std::function<bool(const UA_BrowseResult &, namespaceTree &)> appendReferences;

  appendReferences = [&](const UA_BrowseResult & result, namespaceTree & parent) {
      if (result.referencesSize > 0 && result.references == nullptr) {
        std::cerr << "[OPCUA browse] Risposta incoerente: referencesSize="
                  << result.referencesSize << " ma references e' nullo per il nodo "
                  << parent.nodeId << std::endl;
        return false;
      }

      for (size_t index = 0; index < result.referencesSize; ++index) {
        const UA_ReferenceDescription & reference = result.references[index];
        namespaceTree child;
        child.browseName = UA_StringConversion(reference.browseName.name);
        child.displayName = UA_StringConversion(reference.displayName.text);
        child.nodekind = toNodeKind(reference.nodeClass);
        child.referenceType = nodeIdToString(reference.referenceTypeId);
        child.nodeId = nodeIdToString(reference.nodeId.nodeId);

        if (child.nodeId.empty()) {
          std::cerr << "[OPCUA browse] Conversione NodeId fallita per un riferimento figlio di "
                    << parent.nodeId << std::endl;
          return false;
        }

        UA_NodeId child_node_id;
        UA_NodeId_init(&child_node_id);
        UA_StatusCode status = UA_NodeId_copy(&reference.nodeId.nodeId, &child_node_id);
        if (status != UA_STATUSCODE_GOOD) {
          std::cerr << "[OPCUA browse] Copia NodeId fallita per " << child.nodeId
                    << ": " << UA_StatusCode_name(status) << std::endl;
          return false;
        }

        const bool local_node = reference.nodeId.serverIndex == 0 &&
          reference.nodeId.namespaceUri.length == 0;
        const bool first_visit = visited.insert(child.nodeId).second;
        if (local_node && first_visit && !browseNode(child_node_id, child)) {
          std::cerr << "[OPCUA browse] Esplorazione ricorsiva fallita sul nodo "
                    << child.nodeId << " (" << child.browseName << ")"
                    << std::endl;
          UA_NodeId_clear(&child_node_id);
          return false;
        }

        UA_NodeId_clear(&child_node_id);
        parent.children.push_back(std::move(child));
      }

      return true;
    };

  browseNode = [&](const UA_NodeId & node_id, namespaceTree & node) {
      UA_BrowseRequest request;
      UA_BrowseRequest_init(&request);
      request.nodesToBrowse = static_cast<UA_BrowseDescription *>(
        UA_Array_new(1, &UA_TYPES[UA_TYPES_BROWSEDESCRIPTION]));
      if (request.nodesToBrowse == nullptr) {
        std::cerr << "[OPCUA browse] Allocazione BrowseDescription fallita per il nodo "
                  << nodeIdToString(node_id) << std::endl;
        UA_BrowseRequest_clear(&request);
        return false;
      }

      request.nodesToBrowseSize = 1;
      UA_BrowseDescription_init(&request.nodesToBrowse[0]);
      UA_StatusCode status = UA_NodeId_copy(&node_id, &request.nodesToBrowse[0].nodeId);
      if (status != UA_STATUSCODE_GOOD) {
        std::cerr << "[OPCUA browse] Copia NodeId nella richiesta fallita per "
                  << nodeIdToString(node_id) << ": " << UA_StatusCode_name(status)
                  << std::endl;
        UA_BrowseRequest_clear(&request);
        return false;
      }

      request.nodesToBrowse[0].browseDirection = UA_BROWSEDIRECTION_FORWARD;
      request.nodesToBrowse[0].referenceTypeId =
        UA_NODEID_NUMERIC(0, UA_NS0ID_HIERARCHICALREFERENCES);
      request.nodesToBrowse[0].includeSubtypes = true;
      request.nodesToBrowse[0].nodeClassMask = 0;
      request.nodesToBrowse[0].resultMask = UA_BROWSERESULTMASK_ALL;

      UA_BrowseResponse response = UA_Client_Service_browse(client_, request);
      if (response.responseHeader.serviceResult != UA_STATUSCODE_GOOD) {
        std::cerr << "[OPCUA browse] Browse Service fallito per "
                  << nodeIdToString(node_id) << ": "
                  << UA_StatusCode_name(response.responseHeader.serviceResult)
                  << std::endl;
        UA_BrowseResponse_clear(&response);
        UA_BrowseRequest_clear(&request);
        return false;
      }
      if (response.resultsSize == 0 || response.results == nullptr) {
        std::cerr << "[OPCUA browse] Browse non ha restituito un risultato per "
                  << nodeIdToString(node_id) << " (resultsSize="
                  << response.resultsSize << ")" << std::endl;
        UA_BrowseResponse_clear(&response);
        UA_BrowseRequest_clear(&request);
        return false;
      }
      if (response.results[0].statusCode != UA_STATUSCODE_GOOD) {
        std::cerr << "[OPCUA browse] Browse del nodo " << nodeIdToString(node_id)
                  << " fallito: "
                  << UA_StatusCode_name(response.results[0].statusCode) << std::endl;
        UA_BrowseResponse_clear(&response);
        UA_BrowseRequest_clear(&request);
        return false;
      }

      UA_BrowseNextResponse next_response;
      UA_BrowseNextResponse_init(&next_response);
      bool has_next_response = false;
      const UA_BrowseResult * current_result = &response.results[0];

      while (true) {
        const UA_BrowseResult & result = *current_result;
        if (!appendReferences(result, node)) {
          if (has_next_response) {
            UA_BrowseNextResponse_clear(&next_response);
          } else {
            UA_BrowseResponse_clear(&response);
          }
          UA_BrowseRequest_clear(&request);
          return false;
        }

        if (result.continuationPoint.length == 0) {
          break;
        }

        UA_BrowseNextRequest next_request;
        UA_BrowseNextRequest_init(&next_request);
        next_request.continuationPoints = static_cast<UA_ByteString *>(
          UA_Array_new(1, &UA_TYPES[UA_TYPES_BYTESTRING]));
        if (next_request.continuationPoints == nullptr) {
          std::cerr << "[OPCUA browse] Allocazione BrowseNext fallita per il nodo "
                    << nodeIdToString(node_id) << std::endl;
          UA_BrowseNextRequest_clear(&next_request);
          if (has_next_response) {
            UA_BrowseNextResponse_clear(&next_response);
          } else {
            UA_BrowseResponse_clear(&response);
          }
          UA_BrowseRequest_clear(&request);
          return false;
        }

        next_request.continuationPointsSize = 1;
        status = UA_ByteString_copy(
          &result.continuationPoint, &next_request.continuationPoints[0]);
        if (status != UA_STATUSCODE_GOOD) {
          std::cerr << "[OPCUA browse] Copia continuation point fallita per il nodo "
                    << nodeIdToString(node_id) << ": " << UA_StatusCode_name(status)
                    << std::endl;
          UA_BrowseNextRequest_clear(&next_request);
          if (has_next_response) {
            UA_BrowseNextResponse_clear(&next_response);
          } else {
            UA_BrowseResponse_clear(&response);
          }
          UA_BrowseRequest_clear(&request);
          return false;
        }

        if (has_next_response) {
          UA_BrowseNextResponse_clear(&next_response);
        } else {
          UA_BrowseResponse_clear(&response);
        }
        next_response = UA_Client_Service_browseNext(client_, next_request);
        UA_BrowseNextRequest_clear(&next_request);
        has_next_response = true;

        if (next_response.responseHeader.serviceResult != UA_STATUSCODE_GOOD ||
          next_response.resultsSize == 0 || next_response.results == nullptr ||
          next_response.results[0].statusCode != UA_STATUSCODE_GOOD)
        {
          if (next_response.responseHeader.serviceResult != UA_STATUSCODE_GOOD) {
            std::cerr << "[OPCUA browse] BrowseNext Service fallito per "
                      << nodeIdToString(node_id) << ": "
                      << UA_StatusCode_name(next_response.responseHeader.serviceResult)
                      << std::endl;
          } else if (next_response.resultsSize == 0 || next_response.results == nullptr) {
            std::cerr << "[OPCUA browse] BrowseNext non ha restituito un risultato per "
                      << nodeIdToString(node_id) << std::endl;
          } else {
            std::cerr << "[OPCUA browse] BrowseNext del nodo " << nodeIdToString(node_id)
                      << " fallito: "
                      << UA_StatusCode_name(next_response.results[0].statusCode)
                      << std::endl;
          }
          UA_BrowseNextResponse_clear(&next_response);
          UA_BrowseRequest_clear(&request);
          return false;
        }

        current_result = &next_response.results[0];
      }

      if (has_next_response) {
        UA_BrowseNextResponse_clear(&next_response);
      } else {
        UA_BrowseResponse_clear(&response);
      }
      UA_BrowseRequest_clear(&request);
      return true;
    };

  UA_NodeId root_id = UA_NODEID_NUMERIC(0, UA_NS0ID_ROOTFOLDER);
  namespaceTree root;
  root.browseName = "RootFolder";
  root.displayName = "Root";
  root.nodekind = NodeKind::Object;
  root.nodeId = nodeIdToString(root_id);
  if (root.nodeId.empty()) {
    std::cerr << "[OPCUA browse] Conversione del NodeId di RootFolder fallita"
              << std::endl;
    return std::nullopt;
  }

  visited.insert(root.nodeId);
  if (!browseNode(root_id, root)) {
    std::cerr << "[OPCUA browse] Esplorazione interrotta; nodi visitati prima dell'errore: "
              << visited.size() << std::endl;
    return std::nullopt;
  }

  std::cerr << "[OPCUA browse] Esplorazione completata; nodi visitati: "
            << visited.size() << std::endl;
  return root;
}

}
