#include "franka_opcua_bridge/fake_protocol_client.hpp"



namespace franka_opcua_bridge{


  bool FakeProtocolClient::isConnected() const{ return connected_;}

  bool FakeProtocolClient::connect(
               const std::string & endpoint,
               const std::string & username,
               const std::string & password){

        if (endpoint != "" &&
            username != "" &&
            password != ""){
            
              connected_ = true;
              return true;
              }

        return false;

  }

  void FakeProtocolClient::disconnect(){connected_ = false;}


  CallResult FakeProtocolClient::callMethod(const std::vector<std::string> & object_browse_path,
        const std::string & method_name,
        const std::vector<Value> & args){
                
        CallResult result;

        if (connected_ == false){
            result.ok=false;
            result.error_message = "Errore: non  connesso";
            return result;
            }
            
        result.ok = true;
        result.output_values = args;
        return result; 
        
    }


    bool FakeProtocolClient::readValue(const std::vector<std::string> & variable_browse_path, Value & out_value){




    }

    std::string FakeProtocolClient::pathToKey(const std::vector<std::string> & path){
        
        std::string key;
        for (const auto & p : path) {key += "/" + p;}
        return key;
}

  }