#include "networking.h"
#pragma once

Networking::Networking(painlessMesh *mesh): 
_mesh(mesh)
{
   
};

bool Networking::init(Tasker *tasker)
{
    if(!_mesh) return false ;
    if(!tasker) return false ;

    this->_mesh->setDebugMsgTypes(ERROR | MESH_STATUS | CONNECTION | GENERAL);
    this->_mesh->init(MESH_SSID, MESH_PASSWORD, &tasker->getSchedulerRef(), MESH_PORT, WIFI_STA, meshChannel);
    _nodeId = this->_mesh->getNodeId();
    return true; 
}

bool Networking::findRootNodeId()
{
   
    //   // Get the JSON topology
      String topology = this->_mesh->subConnectionJson();

      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, topology);

      if (error)
      {
        Serial.println("Error parsing topology");
      }

      if (doc.containsKey("subs"))
      {
        JsonArray subs = doc["subs"].as<JsonArray>();
        for (JsonObject node : subs)
        {
          if (node.containsKey("root") && node["root"] == true)
          {
            _rootNodeId = node["nodeId"];
            
            return true; 
          };
        };
      }
      return false;
};


uint32_t Networking::getRootNodeId()
{
    if(findRootNodeId() == true)
    {
        return _rootNodeId;
    }
    else {return uint32_t {1};};
};

bool Networking::setupMeshCallbacks(
    NewConnCallback newConn,
    ChangedConnCallback changedConn,
    TimeAdjCallback timeAdj,
    ReceiveCallback receive,
    DroppedConnCallback droppedConn)
{
    if (!_mesh)
        return false;

    // Assign callbacks if provided
    if (newConn)
        _mesh->onNewConnection(newConn);
    if (changedConn)
        _mesh->onChangedConnections(changedConn);
    if (timeAdj)
        _mesh->onNodeTimeAdjusted(timeAdj);
    if (receive)
        _mesh->onReceive(receive);
    if (droppedConn)
        _mesh->onDroppedConnection(droppedConn);

    return true;
};

bool Networking::setContainsRoot(bool flag){
    
    if (!_mesh) {return false;}
    this->_mesh->setContainsRoot(flag);
    return true;
}
void Networking::update()
{
    _mesh->update();
}

IPAddress Networking::getLocalIP()
{
    
    _localIP = _mesh->getStationIP();    
    return IPAddress(_localIP);
    
}
bool Networking::updateNodeList()
{
    _nodeList = _mesh->getNodeList();
    if (_nodeList.size() > 0) return true;
    else return false; 
};

std::list<uint32_t> Networking::getNodes()
{
    return _nodeList;
}
uint32_t Networking::getNodeId()
{
    return _nodeId;
};


bool Networking::sendMsg(uint32_t &dest, String& msg)
{
    for(auto node: _nodeList)
    {
        if(dest == node)
        {
            _mesh->sendSingle(dest, msg);
            return true;
        }
    }
    return false; 
}

void Networking::broadcastMsg(String& msg)
{
    _mesh->sendBroadcast(msg);
}
std::list<uint32_t> Networking::getNodeList()
{
    return _nodeList;
};
