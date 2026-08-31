#include "franka_opcua_bridge/franka_robot.hpp"
#include <chrono>
#include <thread>


namespace franka_opcua_bridge
{

const std::vector<std::string> FrankaRobot::kExecutionControlPath = {"Robot", "ExecutionControl"};


FrankaRobot::FrankaRobot(
  std::unique_ptr<IProtocolClient> client,
  std::string endpoint,
  std::string user,
  std::string password) :

client_(std::move(client)),
endpoint_(std::move(endpoint)),
user_(std::move(user)),
password_(std::move(password)) {}


bool FrankaRobot::connect()
{

  return client_->connect(endpoint_, user_, password_);
}

void FrankaRobot::disconnect()
{

  return client_->disconnect();
}

bool FrankaRobot::openBrakes()
{

  CallResult result = client_->callMethod(kExecutionControlPath, "OpenBrakes", {});

  return result.ok;

}

bool FrankaRobot::closeBrakes()
{

  CallResult result = client_->callMethod(kExecutionControlPath, "CloseBrakes", {});

  return result.ok;
}

bool FrankaRobot::areBrakesOpen()
{

  CallResult result = client_->callMethod(kExecutionControlPath, "BrakesOpen", {});

  return result.ok;
}


bool FrankaRobot::stop()
{

  CallResult result = client_->callMethod(kExecutionControlPath, "StopTask", {});

  return result.ok;
}


bool FrankaRobot::requestControl()
{

  Value out_value;
  std::vector<std::string> browse_path = kExecutionControlPath;
  std::string method = "ControlTokenActive";

  CallResult result = client_->callMethod(kExecutionControlPath, "RequestControlToken", {});

  if (result.ok == false) {return false;}

  browse_path.push_back(method);


  //Tentativi di accesso al token per il controllo
  for (int i = 0; i <= 10; i++) {


    if (client_->readValue(
        browse_path,
        out_value) && out_value.is<bool>() && out_value.as<bool>())
    {

      return true;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

  }

  return false;


}


bool FrankaRobot::releaseControl()
{

  CallResult result = client_->callMethod(kExecutionControlPath, "FreeControlToken", {});

  return result.ok;
}

bool FrankaRobot::executeNamedTask(const std::string & task_id)
{

  std::vector<Value> args;
  Value elem(std::string(task_id));

  args.push_back(elem);

  CallResult result = client_->callMethod(kExecutionControlPath, "StartTask", args);

  return result.ok;


}


template<typename T>
T FrankaRobot::extractField(const Value::Struct & fields, const std::string & key, const T & default_value){

    Value out_value;
    auto elem = fields.find(key);

    if (elem != fields.end()) {      //.end() una "sentinella" che segna "oltre l'ultimo elemento", non un elemento vero

    //trovato elem->first è la chiave, elem->second è il value
    out_value = elem->second;

    if (out_value.is<T>()) return out_value.as<T>(); else return default_value;

    } else return default_value;

}


std::unique_ptr<RobotStatus> FrankaRobot::readStatus()
{
    auto robotStatus = std::make_unique<FrankaRobotStatus>();
    Value output_value;
    std::string method = "ExecutionStatus";
    std::vector<std::string> path = kExecutionControlPath;
    auto error_status = std::make_unique<FrankaRobotStatus>();

    error_status->error_message="Struttura di default. Errore nella lettura dei dati sullo stato.";
    error_status->has_error=true;

    path.push_back(method);

    bool success = client_->readValue(path, output_value);

    if (success)
    {
        if(output_value.is<Value::Struct>()){

            Value::Struct fields = output_value.as<Value::Struct>();

            robotStatus->has_error =  extractField(fields, "HasError", false);
            robotStatus->is_running = extractField(fields, "IsRunning", false);
            robotStatus->error_message = extractField(fields, "ErrorMessage", std::string(""));
            robotStatus->active_task_name = extractField(fields, "ActiveTaskName", std::string(""));
            robotStatus->active_task_id = extractField(fields, "ActiveTaskId", std::string(""));
           
            return robotStatus;
            
        } else return error_status;

    } else return error_status;

}

}
