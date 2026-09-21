#ifndef FRANKA_OPCUA_BRIDGE__OPCUA_VALUE_HPP_
#define FRANKA_OPCUA_BRIDGE__OPCUA_VALUE_HPP_

#include <map>
#include <string>
#include <vector>
#include <variant>
#include <cstdint> //aggiunta come fix oer int32_t, da controllare

namespace franka_opcua_bridge{


// -- Tipi Franka-specific: mappano le struct custom del NodeSet OPC UA --

struct KeyIntPairValue
{
  std::string key;
  int32_t value;

};

struct KeyPosePairValue
{
  std::string key;
  std::vector<double> value;
};

struct ExecutionStatusValue
{
  bool has_error;
  bool is_running;
  std::string error_message;
  std::string active_task_name;
  std::string active_task_id;
};


// Rappresenta un valore che puo' viaggiare da/verso il server OPC UA.

class Value
{

public:
  using Struct = std::map<std::string, Value>;
  using Variant = std::variant<
    bool,
    int32_t,
    double,
    std::string,
    std::vector<double>,
    Struct>;


  Value() = default;
  Value(Variant v)
  : data_(std::move(v)) {}                            //NOLINT (Conversione implicita voluta)

  template<typename T>
  bool is() const {return std::holds_alternative<T>(data_);}

  template<typename T>
  const T & as() const {return std::get<T>(data_);}

  const Variant & raw() const {return data_;}

private:
  Variant data_;

};


struct CallResult
{

  bool ok = false;
  std::string error_message;
  std::vector<Value> output_values;       //Alcuni metodi OPC UA restituiscono dei valori

};




}



#endif
