#include "franka_opcua_bridge/franka_robot.hpp"
#include <chrono>
#include <thread>
#include "geometry_msgs/msg/pose.hpp"
#include <Eigen/Dense>

namespace franka_opcua_bridge
{

const std::vector<std::string> FrankaRobot::kExecutionControlPath = {"Robot", "ExecutionControl"};
const std::vector<std::string> kPoseMapPath = {"Robot","KeyValueMaps"};


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


//Legge in radianti la rotazione di ogni giunto del braccio, non end-effector
std::vector<double> FrankaRobot::readJointAngles(){

    Value jointAngles;
    std::vector<std::string> path = kExecutionControlPath;
    std::string method = "JointAngles";

    path.push_back(method);
    
    bool success = client_->readValue(path, jointAngles);

    if (success && jointAngles.is<std::vector<double>>()){

        return jointAngles.as<std::vector<double>>();
    
    } else return {}; //Ritorna il vettore vuoto

}

//Legge lo stato cartesiano dell'end-effector (la pinza)
 geometry_msgs::msg::Pose FrankaRobot::readCartesianPose(){

    Value output_value;
    std::vector<std::string> path = kExecutionControlPath;
    std::string method = "CartesianPose";
    Eigen::Matrix4d matrix;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Pose fallback;
    fallback.orientation.w = 1.0;
    
    path.push_back(method);

    bool success = client_->readValue(path, output_value);

    if(success && output_value.is<std::vector<double>>() && output_value.as<std::vector<double>>().size() == 16){

        const std::vector<double>& values = output_value.as<std::vector<double>>();
        
        //16 double restituiti da Franka server in ordine column-major (quello che ci serve per Eigen)
        
        matrix = Eigen::Map<Eigen::Matrix4d>(values.data()); //values.data(): puntatore al primo elemento dei double.

        //Estrazione della matrice di rotazione 3x3
        const Eigen::Matrix3d rotation = matrix.block<3, 3>(0,0);

        //Conversione della matrice di rotazione in quaternione

        Eigen::Quaterniond quaternion(rotation); 

        // Posizione
        pose.position.x = matrix(0, 3);
        pose.position.y = matrix(1, 3);
        pose.position.z = matrix(2, 3);

        // Orientamento
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();

        return pose;
        
    }

    return fallback; 


 }

 bool FrankaRobot::moveToNamedPose(const std::string & pose_id){

  std::vector<std::string> pathKeyPose = kPoseMapPath;
  std::vector<double> values;
  std::vector<Value> args;
  CallResult result, resultReplace;
  std::string lastPathElem = "KeyPoseMap";

  pathKeyPose.push_back(lastPathElem);
  args.push_back(Value(pose_id));

  result = client_->callMethod(pathKeyPose, "Read", args);

  if(result.ok){

    if(result.output_values.empty()) return false;

    values = result.output_values[0].as<std::vector<double>>();
    
    resultReplace = client_->callMethod(pathKeyPose, "Replace", values.as<std::map<std::string, Value>());

    if(resultReplace.ok){

      return this->executeNamedTask("opcua_goto");
    }

  } else return false;

 }

}
