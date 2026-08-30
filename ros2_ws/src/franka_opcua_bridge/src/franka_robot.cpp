#include "franka_opcua_bridge/franka_robot.hpp"


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

    bool FrankaRobot::disconnect(){

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
        std::string met = "ControlTokenOwner";

        kExecutionControlPath.push_back(met);
        client_->readValue(kExecutionControl, out_value );

        std::string owner = out_value.as<std::string>();

        


    }




}