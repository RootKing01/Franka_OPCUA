#ifndef FRANKA_OPCUA_BRIDGE__OPCUA_NAMESPACE_TREE_
#define FRANKA_OPCUA_BRIDGE__OPCUA_NAMESPACE_TREE_

#include <string>
#include <vector>

namespace franka_opcua_bridge
{

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


struct namespaceTree
{

  std::string browseName;
  std::string displayName;
  std::string nodeId;
  NodeKind nodekind = NodeKind::Unknown;
  std::string referenceType; //Facoltativo: tipo di relazione con il padre
  std::vector<namespaceTree> children;

};




}

#endif
