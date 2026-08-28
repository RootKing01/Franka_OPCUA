#ifndef FRANKA_OPCUA_BRIDGE__I_PROTOCOL_CLIENT_HPP_
#define FRANKA_OPCUA_BRIDGE__I_PROTOCOL_CLIENT_HPP_

#include <map>
#include <string>
#include <vector>
#include <variant>
#include <cstdint> //aggiunta come fix oer int32_t, da controllare

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

    // Interfaccia astratta per un client di un protocollo di comunicazione
    // industriale generico (OPC UA, ma potenzialmente altro in futuro).
    // Non conosce nulla di "robot", "Franka", "freni" - solo concetti
    // di comunicazione: connessione, nodi identificati per percorso,
    // chiamata di metodi, lettura/scrittura di variabili.

    class IProtocolClient{

        public:
            virtual ~IProtocolClient() = default;

            virtual bool connect(

                const std::string & endpoint,
                const std::string & username,
                const std::string & password)  = 0; //Attenzione al discorso password, controlla durante implementazione
            
            virtual void disconnect() = 0;
            
            virtual bool isConnected() const = 0;
            
            

            //browse_path: es: {"Robot", "ExecutionControl", "OpenBrakes"}
            virtual CallResult callMethod(
                const std::vector<std::string> & object_browse_path,
                const std::string & method_name,
                const std::vector<Value> & args) = 0;

            virtual bool readValue(
                const std::vector<std::string> & variable_browse_path,
                Value & out_value) = 0;

            virtual bool writeValue(
                const std::vector<std::string> & variable_browse_path,
                const Value & out_value) = 0;


        protected:
                IProtocolClient() = default;



    };



} //end namespace

#endif  // end FRANKA_OPCUA_BRIDGE__I_PROTOCOL_CLIENT_HPP_