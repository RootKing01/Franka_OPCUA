#ifndef FRANKA_OPCUA_BRIDGE__FAKE_ROBOT_HPP_
#define FRANKA_OPCUA_BRIDGE__FAKE_ROBOT_HPP_

#include "franka_opcua_bridge/i_protocol_client.hpp"

namespace franka_opcua_bridge{


    struct CallRecord{
        std::vector<std::string> object_path;
        std::string name_method;
        std::vector<Value> args;
    };

    class FakeProtocolClient : public IProtocolClient{

        public:
            
            bool connect(
                const std::string & endpoint,
                const std::string & username,
                const std::string & password) override;
            
            void disconnect() override;

            bool isConnected() const override;

            CallResult callMethod(const std::vector<std::string> & object_browse_path,
                const std::string & method_name,
                const std::vector<Value> & args) override;

            bool readValue( 
                const std::vector<std::string> & variable_browse_path,
                Value & out_value) override;
            
            bool writeValue( 
                 const std::vector<std::string> & variable_browse_path,
                 const Value & out_value) override;

        private:

            static std::string pathToKey(const std::vector<std::string> & path);
            bool connected_ = false;
            std::vector<CallRecord> call_log_;
            std::map<std::string, Value> variables_;   // simula lo storage del server
    };


}


#endif