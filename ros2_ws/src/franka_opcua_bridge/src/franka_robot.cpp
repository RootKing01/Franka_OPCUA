#include "franka_opcua_bridge/franka_robot.hpp"
#include <chrono>
#include <thread>


namespace franka_opcua_bridge{

    const std::vector<std::string> FrankaRobot::kExecutionControlPath = {"Robot", "ExecutionControl"};


    FrankaRobot::FrankaRobot(std::unique_ptr<IProtocolClient> client, 
                         std::string endpoint,
                         std::string user,
                         std::string password) : 
                         
                         client_(std::move(client)),
                         endpoint_(std::move(endpoint)),
                         user_(std::move(user)),
                         password_(std::move(password)) {};


    
    bool FrankaRobot::connect(){

        return client_->connect(endpoint_, user_, password_);
    }

    void FrankaRobot::disconnect(){

        return client_->disconnect();
    }

    bool FrankaRobot::openBrakes(){

        CallResult result = client_->callMethod(kExecutionControlPath, "OpenBrakes", {});

        return result.ok;

    }

    bool FrankaRobot::closeBrakes(){

        CallResult result = client_->callMethod(kExecutionControlPath, "CloseBrakes", {});

        return result.ok;
    }

    bool FrankaRobot::areBrakesOpen(){

        CallResult result = client_->callMethod(kExecutionControlPath, "BrakesOpen", {});

        return result.ok;
    }


    bool FrankaRobot::stop(){

        CallResult result = client_->callMethod(kExecutionControlPath, "StopTask", {});

        return result. ok;
    }


    bool FrankaRobot::requestControl(){
        
        Value out_value;
        std::vector<std::string> browse_path = kExecutionControlPath;
        std::string method = "ControlTokenOwner";

        CallResult result = client_->callMethod(kExecutionControlPath, "RequestControlToken", {});
        
        if (result.ok == false) return false; 

        browse_path.push_back(method);
        
        std::string myUser = user_+ " @ OPC UA ";

       
        
        //Tentativi di accesso al token per il controllo
        for(int i = 0; i <=10; i++){

            client_->readValue(browse_path, out_value );
             
            if(out_value.is<std::string>() && out_value.as<std::string>().compare(0, myUser.size(), myUser) == 0){

                return true;
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));

        }

        return false;
        

       
    }

    
    bool FrankaRobot::releaseControl(){

        CallResult result = client_->callMethod(kExecutionControlPath, "FreeControlToken", {});

        return result.ok;
    }



}