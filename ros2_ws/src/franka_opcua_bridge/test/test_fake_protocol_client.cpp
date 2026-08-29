#include <gtest/gtest.h>
#include "franka_opcua_bridge/fake_test_version/fake_protocol_client.hpp"

using franka_opcua_bridge::FakeProtocolClient;
using franka_opcua_bridge::Value;
using franka_opcua_bridge::CallResult;

//Prima di tentare connect(), isConnected() deve essere false
TEST(FakeProtocolClientTest, startsDisconnected)
{
  FakeProtocolClient client;

  EXPECT_FALSE(client.isConnected());

}

//Controllo funzionamento connect() con parametri corretti
TEST(FakeProtocolClientTest, checkConnection)
{
  FakeProtocolClient client;

  client.connect("ab", "cd", "ef");

  EXPECT_TRUE(client.isConnected());
}

//Test connect() con imput stringa vuota
TEST(FakeProtocolClientTest, checkEmptyString)
{
  FakeProtocolClient client;

  client.connect("", "", "");

  EXPECT_FALSE(client.isConnected());
}

//Test callMethod: se non sei connesso, deve restituire ok = false
TEST(FakeProtocolClientTest, checkConnectionBeforeCallMethod)
{
  FakeProtocolClient client;

  CallResult result = client.callMethod({"Robot", "ExecutionControl"}, "OpenBrakes", {});

  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error_message.empty());
}

//Test callMethod: se sei connesso, callMethod deve avere successo
TEST(FakeProtocolClientTest, checkCallMethodSuccess)
{
  FakeProtocolClient client;

  client.connect("ab", "cd", "ef");
  CallResult result = client.callMethod({"Robot", "ExecutionControl"}, "OpenBrakes", {});

  EXPECT_TRUE(result.ok);
  EXPECT_TRUE(result.error_message.empty());
}

//Test args di callMethod: verifichiamo che vengano correttamente passati in output_values
TEST(FakeProtocolClientTest, checkArgsCallMethodSuccess)
{
  FakeProtocolClient client;
  size_t vuoto = 0;
  std::vector<Value> args = {Value(std::string("nameOperation")), Value(int32_t(42))};

  client.connect("ab", "cd", "ef");
  CallResult result = client.callMethod({"Robot", "ExecutionControl"}, "OpenBrakes", args);


  EXPECT_TRUE(result.ok);
  EXPECT_TRUE(result.error_message.empty());
  EXPECT_TRUE(result.output_values.size() != vuoto);
}

//Test callLog(): verifica che una chiamata callMethod con un certo object_browse_path/method_name venga effettivamente registrata
TEST(FakeProtocolClientTest, verifyCallMethodWithCallLog)
{
  FakeProtocolClient client;
  CallResult result;

  client.connect("ab", "cd", "ef");
  std::vector<Value> args = {Value(std::string("nameOperation")), Value(int32_t(42))};
  result = client.callMethod({"Robot", "ExecutionControl"}, "OpenBrakes", args);

  ASSERT_TRUE(result.ok);
  ASSERT_TRUE(result.error_message.empty());

  const auto & log = client.callLog();
  ASSERT_EQ(log.size(), 1u);   //Esattamente una chiamata registrata

  //Usiamo log[0] per recuperare il primo callRecord (Che è anche l'unico)
  ASSERT_EQ(log[0].object_path, (std::vector<std::string>{"Robot", "ExecutionControl"}));
  ASSERT_EQ(log[0].method_name, "OpenBrakes");
  EXPECT_EQ(log[0].args.size(), 2u);
}

//Test readValue su un path mai scritto prima
TEST(FakeProtocolClientTest, checkNewValueInRead)
{
  FakeProtocolClient client;
  Value value;
  const std::vector<std::string> variable_browse_path = {"Robot", "ExecutionControl"};

  client.connect("ab", "cd", "ef");

  EXPECT_FALSE(client.readValue(variable_browse_path, value));

}

//Test readValue senza essere connessi
TEST(FakeProtocolClientTest, checkReadValue)
{
  FakeProtocolClient client;
  Value value;
  const std::vector<std::string> variable_browse_path = {"Robot", "ExecutionControl"};


  EXPECT_FALSE(client.readValue(variable_browse_path, value));

}

//Test write e read Value combinato
TEST(FakeProtocolClientTest, checkReadAndWrite)
{

  FakeProtocolClient client;
  const Value value(int32_t(7));
  Value output;
  const std::vector<std::string> variable_browse_path = {"Robot", "ExecutionControl"};

  client.connect("ab", "cd", "ef");

  ASSERT_TRUE(client.writeValue(variable_browse_path, value));
  ASSERT_TRUE(client.readValue(variable_browse_path, output));
  EXPECT_EQ(output.as<int32_t>(), 7);

}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
