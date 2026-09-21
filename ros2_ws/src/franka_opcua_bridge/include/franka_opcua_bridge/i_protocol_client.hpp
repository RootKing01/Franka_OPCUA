#ifndef FRANKA_OPCUA_BRIDGE__I_PROTOCOL_CLIENT_HPP_
#define FRANKA_OPCUA_BRIDGE__I_PROTOCOL_CLIENT_HPP_

namespace franka_opcua_bridge
{


// Interfaccia astratta per un client di un protocollo di comunicazione

class IProtocolClient
{

public:
  virtual ~IProtocolClient() = default;

  virtual bool connect() = 0;      

  virtual bool disconnect() = 0;

  virtual bool isConnected() const = 0;

protected:
  IProtocolClient() = default;

};


} //end namespace

#endif  // end FRANKA_OPCUA_BRIDGE__I_PROTOCOL_CLIENT_HPP_
