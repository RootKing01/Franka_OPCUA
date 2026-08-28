#ifndef FRANKA_OPCUA_BRIDGE__I_PROTOCOL_CLIENT_HPP_
#define FRANKA_OPCUA_BRIDGE__I_PROTOCOL_CLIENT_HPP_

#include <map>
#include <string>
#include <vector>
#include <variant>

namespace franka_opcua_bridge{

    // Rappresenta un valore che puo' viaggiare da/verso il server OPC UA.
    // Il caso "Struct" e' generico apposta: IProtocolClient non conosce
    // KeyPosePair/KeyIntPair, sa solo che esiste un tipo custom identificato
    // per nome, con campi con nome. Sara' OpcUaClient (concreto) a sapere
    // come serializzare una data struct nel corretto ExtensionObject.
    class Value{

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
            Value(Variant v) : data_(std::move(v)) {} //NOLINT (Conversione implicita voluta)

            template<typename T>
            bool is() const {return std::holds_alternative<T>(data_);}

            template<typename T>
            const T & as() const {return std::get<T>(data_);}

            const Variant & raw() const {return data_;}


        
        private:

            Variant data_;

    };


    struct CallResult{

        bool ok = false;
        std::string error_message;
        std::vector<Value> output_values; //Alcuni metodi OPC UA restituiscono dei valori 

    };



}