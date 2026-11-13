#include "franka_opcua_bridge/franka_robot.hpp"
#include "franka_opcua_bridge/opcua_value.hpp"

#include <chrono>
#include <thread>
#include <iostream>
#include "geometry_msgs/msg/pose.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace franka_opcua_bridge
{

const std::vector<std::string> FrankaRobot::kExecutionControlPath = {"Robot", "ExecutionControl"};
const std::vector<std::string> kPoseMapPath = {"Robot", "KeyValueMaps"};


FrankaRobot::FrankaRobot(
  std::unique_ptr<IOpcUaProtocolClient> client,
  std::chrono::milliseconds execution_timeout)
: client_(std::move(client)),
  execution_timeout_(execution_timeout) {}


bool FrankaRobot::connect()
{
  return client_->connect();
}

bool FrankaRobot::disconnect()
{

  return client_->disconnect();
}

bool FrankaRobot::isConnected() const
{

  return client_->isConnected();

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
  Value out_value;
  std::vector<std::string> path = kExecutionControlPath;
  path.push_back("BrakesOpen");

  bool success = client_->readValue(path, out_value);
  return success && out_value.is<bool>() && out_value.as<bool>();
}


bool FrankaRobot::stop()
{

  CallResult result = client_->callMethod(kExecutionControlPath, "StopTask", {});

  std::cerr << "[STOP] StopTask result=" << (result.ok ? "success" : "failure");
  if (!result.ok) {
    std::cerr << ", OPC UA error='" << result.error_message << "'";
  }
  std::cerr << std::endl;

  return result.ok;
}


bool FrankaRobot::requestControl(bool force)
{

  Value out_value;
  std::vector<std::string> browse_path = kExecutionControlPath;
  std::string method = "ControlTokenActive";

  CallResult result = client_->callMethod(
    kExecutionControlPath, "RequestControlToken", {Value(
        force)});
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
  Value elem{std::string(task_id)}; //Value elem(std::string(task_id))->"Vexing Parse:"Questo è un caso classico e famoso in C++
                                    //(chiamato "most vexing parse"): il compilatore interpreta questa sintassi come la dichiarazione
                                    //di una funzione chiamata elem, che prende un std::string e restituisce un Value — non come la
                                    //costruzione di una variabile. È ambiguo per il linguaggio, anche se l'intento è chiaro a noi.


  args.push_back(elem);

  CallResult result = client_->callMethod(kExecutionControlPath, "StartTask", args);

  return result.ok;


}


/* Codice non più necessario, lo tengo per sicurezza.

template<typename T>
T FrankaRobot::extractField(
  const Value::Struct & fields, const std::string & key,
  const T & default_value)
{

  Value out_value;
  auto elem = fields.find(key);

  if (elem != fields.end()) {        //.end() una "sentinella" che segna "oltre l'ultimo elemento", non un elemento vero

    //trovato elem->first è la chiave, elem->second è il value
    out_value = elem->second;

    if (out_value.is<T>()) {return out_value.as<T>();} else {return default_value;}

  } else {return default_value;}

}
*/

std::unique_ptr<RobotStatus> FrankaRobot::readStatus()
{
  auto robotStatus = std::make_unique<FrankaRobotStatus>();
  Value output_value;
  std::string method = "ExecutionStatus";
  std::vector<std::string> path = kExecutionControlPath;

  auto error_status = std::make_unique<FrankaRobotStatus>();

  error_status->error_message = "Struttura di default. Errore nella lettura dei dati sullo stato.";
  error_status->has_error = true;

  path.push_back(method);

  bool success = client_->readValue(path, output_value);

  std::cerr << "[STATUS] read success=" << success
            << ", is_execution_status="
            << output_value.is<ExecutionStatusValue>()
            << std::endl;

  if (success) {
    if (output_value.is<ExecutionStatusValue>()) {

      const ExecutionStatusValue & status = output_value.as<ExecutionStatusValue>();

      robotStatus->has_error = status.has_error;
      robotStatus->is_running = status.is_running;
      robotStatus->error_message = status.error_message;
      robotStatus->active_task_name = status.active_task_name;
      robotStatus->active_task_id = status.active_task_id;

      std::cerr << "[STATUS] has_error=" << robotStatus->has_error
                << ", is_running=" << robotStatus->is_running
                << ", active_task_name='" << robotStatus->active_task_name
                << "', active_task_id='" << robotStatus->active_task_id
                << "', error_message='" << robotStatus->error_message
                << "'" << std::endl;

      return robotStatus;

    } else {return error_status;}

  } else {return error_status;}

}


//Legge in radianti la rotazione di ogni giunto del braccio, non end-effector
std::vector<double> FrankaRobot::readJointAngles()
{

  Value jointAngles;
  std::vector<std::string> path = kExecutionControlPath;
  std::string method = "JointAngles";

  path.push_back(method);

  bool success = client_->readValue(path, jointAngles);

  if (success && jointAngles.is<std::vector<double>>()) {

    return jointAngles.as<std::vector<double>>();

  } else {return {};}  //Ritorna il vettore vuoto

}

//Legge lo stato cartesiano dell'end-effector (la pinza)
geometry_msgs::msg::Pose FrankaRobot::readCartesianPose()
{
  geometry_msgs::msg::Pose fallback;
  fallback.orientation.w = 1.0;

  auto pose = tryReadCartesianPose();

  if (!pose) {return fallback;}

  return *pose;


}

std::vector<double> FrankaRobot::readTorque()
{

  std::vector<std::string> path = kExecutionControlPath;
  std::string method = "EstimatedTorques";
  Value output_value;

  path.push_back(method);

  bool success = client_->readValue(path, output_value);

  if (success && output_value.is<std::vector<double>>()) {
    return output_value.as<std::vector<double>>();
  }


  return {};


}

geometry_msgs::msg::Wrench FrankaRobot::readWrench()
{

  Value output_value;
  std::vector<std::string> path = kExecutionControlPath;
  std::string method = "EstimatedForces";

  geometry_msgs::msg::Wrench wrench;
  geometry_msgs::msg::Wrench fallback_wrench;


  path.push_back(method);

  bool success = client_->readValue(path, output_value);

  if (success && output_value.is<std::vector<double>>() &&
    output_value.as<std::vector<double>>().size() == 6)
  {

    auto estimatedForce = output_value.as<std::vector<double>>();

    wrench.force.x = estimatedForce[0];
    wrench.force.y = estimatedForce[1];
    wrench.force.z = estimatedForce[2];

    wrench.torque.x = estimatedForce[3];
    wrench.torque.y = estimatedForce[4];
    wrench.torque.z = estimatedForce[5];

    return wrench;

  } else {return fallback_wrench;}

}

MotionResult FrankaRobot::moveToNamedPose(
  const std::string & pose_id,
  const std::function<bool()> & should_abort)
{
  std::vector<std::string> pathKeyPose = kPoseMapPath;
  std::vector<Value> args;
  CallResult result;

  pathKeyPose.push_back("KeyPoseMap");

  // Read(pose_id)
  args.push_back(Value(pose_id));

  result = client_->callMethod(pathKeyPose, "Read", args);

  std::cerr << "[MOVE] Read pose '" << pose_id
            << "': ok=" << result.ok
            << ", outputs=" << result.output_values.size()
            << ", error='" << result.error_message << "'"
            << std::endl;

  if (!result.ok) {
    return {MotionOutcome::Failed, "Failed to read saved pose: " + result.error_message};
  }

  // Il metodo Read deve restituire almeno un valore
  if (result.output_values.empty()) {
    return {MotionOutcome::Failed, "Saved pose read returned no values"};
  }

  // Il valore restituito deve essere un vector<double>
  if (!result.output_values[0].is<std::vector<double>>()) {
    return {MotionOutcome::Failed, "Saved pose has an unexpected data type"};
  }

  const std::vector<double> & values = result.output_values[0].as<std::vector<double>>();

  // Una posa Franka è una matrice 4x4 = 16 double
  if (values.size() != 16) {
    return {MotionOutcome::Failed, "Saved pose must contain 16 values"};
  }

  // Costruzione del KeyPosePair:
  // Key   = "target_pose"
  // Value = matrice 4x4 (16 double)

  if (!buildKeyPosePair("target_pose", values)) {
    return {MotionOutcome::Failed, "Failed to write target pose"};
  }

  // Avvia il task che utilizza target_pose
  std::cerr << "[MOVE] target_pose scritto. Avvio opcua_goto" << std::endl;

  bool task_result = executeNamedTask("opcua_goto");

  std::cerr << "[MOVE] StartTask opcua_goto: result="
            << task_result << std::endl;

  if (!task_result) {
    return {MotionOutcome::Failed, "Failed to start opcua_goto task"};
  }

  constexpr auto kPollingPeriod = std::chrono::milliseconds(100);
  const auto deadline = std::chrono::steady_clock::now() + execution_timeout_;
  bool task_started = false;

  while (std::chrono::steady_clock::now() < deadline) {
    const auto status = readStatus();

    if (!status) {
      return {MotionOutcome::Failed, "Failed to read robot execution status"};
    }
    if (status->has_error) {
      return {MotionOutcome::Failed, status->error_message};
    }
    if (should_abort && should_abort()) {
      return {MotionOutcome::Cancelled, "Motion cancelled by stop request"};
    }

    if (status->is_running || status->active_task_name == "opcua_goto") {
      task_started = true;
    } else if (task_started) {
      return {MotionOutcome::Completed, "Pose reached"};
    }

    std::this_thread::sleep_for(kPollingPeriod);
  }

  stop();
  return {MotionOutcome::Timeout, "Timed out while waiting for pose motion to finish"};
}

std::vector<double> FrankaRobot::fromPoseToVector(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Matrix4d matrix;

  if (!(std::isfinite(pose.position.x)) || !(std::isfinite(pose.position.y)) ||
    !(std::isfinite(pose.position.z))) {return {};}

  Eigen::Quaterniond quaternion(pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z
  );

  if (!(std::abs(quaternion.norm() - 1.0) < kTolerance)) {return {};}

  quaternion.normalize();

  const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  matrix.block<3, 3>(0, 0) = rotation;

  matrix(0, 3) = pose.position.x;
  matrix(1, 3) = pose.position.y;
  matrix(2, 3) = pose.position.z;

  matrix.block<1, 4>(3, 0) << 0.0, 0.0, 0.0, 1.0;

  std::vector<double> value(matrix.data(), matrix.data() + 16);
  return value;


}

bool FrankaRobot::buildKeyPosePair(const std::string & pose_id, const std::vector<double> & value)
{
  // Replace(KeyPosePair)
  std::vector<Value> replaceArgs;
  replaceArgs.push_back(Value(KeyPosePairValue{pose_id, value}));

  std::vector<std::string> pathKeyPose = kPoseMapPath;
  pathKeyPose.push_back("KeyPoseMap");

  CallResult result = client_->callMethod(pathKeyPose, "Replace", replaceArgs);

  return result.ok;
}


bool FrankaRobot::savePose(const std::string & pose_id)
{

  auto pose = tryReadCartesianPose();

  if (!pose) {return false;}

  return savePose(pose_id, *pose);
}

bool FrankaRobot::setIntegerVariable(const std::string & variable_id, int32_t value)
{

  std::vector<Value> replaceArgs;
  replaceArgs.push_back(Value(KeyIntPairValue{variable_id, value}));

  std::vector<std::string> pathKeyInt = kPoseMapPath;
  pathKeyInt.push_back("KeyIntMap");

  CallResult result = client_->callMethod(pathKeyInt, "Replace", replaceArgs);

  return result.ok;

}


bool FrankaRobot::savePose(const std::string & pose_id, const geometry_msgs::msg::Pose & pose)
{

  std::vector<double> poseVector = fromPoseToVector(pose);

  if (poseVector.size() != 16) {

    return false;
  }

  return buildKeyPosePair(pose_id, poseVector);

}

bool FrankaRobot::activateFCI()
{
  CallResult result = client_->callMethod(kExecutionControlPath, "ActivateFCI", {});

  return result.ok;
}

bool FrankaRobot::deactivateFCI()
{
  CallResult result = client_->callMethod(kExecutionControlPath, "DeactivateFCI", {});

  return result.ok;
}

std::optional<geometry_msgs::msg::Pose> FrankaRobot::tryReadCartesianPose()
{
  Value output_value;
  std::vector<std::string> path = kExecutionControlPath;
  std::string method = "CartesianPose";
  Eigen::Matrix4d matrix;
  geometry_msgs::msg::Pose pose;

  path.push_back(method);

  bool success = client_->readValue(path, output_value);

  if (success && output_value.is<std::vector<double>>() &&
    output_value.as<std::vector<double>>().size() == 16)
  {

    const std::vector<double> & values = output_value.as<std::vector<double>>();

    //16 double restituiti da Franka server in ordine column-major (quello che ci serve per Eigen)

    matrix = Eigen::Map<const Eigen::Matrix4d>(values.data());     //values.data(): puntatore al primo elemento dei double.

    //Estrazione della matrice di rotazione 3x3
    const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);

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

  return std::nullopt;

}


bool FrankaRobot::switchToExecution()
{

  CallResult result = client_->callMethod(kExecutionControlPath, "SwitchToExecution", {});

  return result.ok;
}


std::vector<std::string> FrankaRobot::showAvailableTasks()
{

  std::vector<std::string> path = kExecutionControlPath;
  std::string method = "Tasks";
  Value output_value;

  path.push_back(method);

  bool success = client_->readValue(path, output_value);

  if (success && output_value.is<std::vector<std::string>>()) {
    return output_value.as<std::vector<std::string>>();
  }


  return {};


}


std::optional<namespaceTree> FrankaRobot::showAddressSpaceNodes()
{

  return client_->exploreAddressSpace();

}


}
