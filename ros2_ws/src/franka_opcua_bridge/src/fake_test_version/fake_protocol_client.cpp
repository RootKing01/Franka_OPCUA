#include "franka_opcua_bridge/fake_test_version/fake_protocol_client.hpp"


namespace franka_opcua_bridge
{


bool FakeProtocolClient::isConnected() const {return connected_;}

bool FakeProtocolClient::connect(
  const std::string & endpoint,
  const std::string & username,
  const std::string & password)
{

  if (endpoint != "" &&
    username != "" &&
    password != "")
  {

    connected_ = true;
    return true;
  }

  return false;

}

void FakeProtocolClient::disconnect() {connected_ = false;}


CallResult FakeProtocolClient::callMethod(
  const std::vector<std::string> & object_browse_path,
  const std::string & method_name,
  const std::vector<Value> & args)
{

  CallResult result;

  std::vector<std::string> keyMapPath_ = {"Robot", "KeyValueMaps", "KeyPoseMap"};

  if (connected_ == false) {
    result.ok = false;
    result.error_message = "Errore: non  connesso";
    return result;
  }

  call_log_.push_back({object_browse_path, method_name, args});

  if (object_browse_path == keyMapPath_ && method_name == "Read" && !args.empty() &&
    args[0].is<std::string>())
  {

    std::vector<std::string> path = keyMapPath_;
    path.push_back(method_name);
    path.push_back(args[0].as<std::string>());

    Value found_value;
    if (this->readValue(path, found_value)) {

      result.ok = true;
      result.output_values = {found_value};
      return result;
    }

  }

  result.ok = true;
  result.output_values = args;
  return result;

}


bool FakeProtocolClient::readValue(
  const std::vector<std::string> & variable_browse_path,
  Value & out_value)
{


  if (connected_ == false) {return false;}

  std::string path_key = pathToKey(variable_browse_path);

  auto elem = variables_.find(path_key);

  if (elem != variables_.end()) {      // variables_.end() una "sentinella" che segna "oltre l'ultimo elemento", non un elemento vero

    //trovato elem->first è la chiave, elem->second è il value
    out_value = elem->second;
    return true;
  } else {

    //non trovato
    return false;
  }


}


bool FakeProtocolClient::writeValue(
  const std::vector<std::string> & variable_browse_path,
  const Value & value)
{

  if (connected_ == false) {return false;}

  std::string key = pathToKey(variable_browse_path);
  variables_[key] = value;         // crea se non esiste, sovrascrive se esiste già
  return true;

}


std::string FakeProtocolClient::pathToKey(const std::vector<std::string> & path)
{

  std::string key;
  for (const auto & p : path) {
    key += "/" + p;
  }
  return key;
}

}
