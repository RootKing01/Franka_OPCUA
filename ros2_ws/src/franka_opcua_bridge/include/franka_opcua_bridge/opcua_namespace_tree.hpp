#ifndef FRANKA_OPCUA_BRIDGE__OPCUA_NAMESPACE_TREE_
#define FRANKA_OPCUA_BRIDGE__OPCUA_NAMESPACE_TREE_

#include <string>

namespace franka_opcua_bridge{


struct namespaceTree{

    std::string browseName;


};

enum class NodeKind
{
  Unspecified,
  Object,
  Variable,
  Method,
  ObjectType,
  VariableType,
  ReferenceType,
  DataType,
  View,
  Unknown   // fallback di sicurezza, per valori futuri non ancora previsti dallo standard
};



}

#endif