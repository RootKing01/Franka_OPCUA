#include "franka_opcua_bridge/franka_robot.hpp"


namespace franka_opcua_bridge{

    const std::vector<std::string> FrankaRobot::kExecutionControlPath = {"Robot", "ExecutionControl"};


    explicit FrankaRobot(std::unique_ptr<IProtocolClient> client): client_(std::move(client)){};
    

    bool openBrakes(){

        CallResult result = client_->callMethod(kExecutionControlPath, "OpenBrakes", {});

        return result.ok;

    }

    bool closeBrakes(){

        CallResult result = client_->callMethod(kExecutionControlPath, "CloseBrakes", {});

        return result.ok;
    }

    bool areBrakesOpen(){


    }


    bool stop(){

        CallResult result = client_->callMethod(kExecutionControlPath, "StopTask", {});

        return result. ok;
    }




}