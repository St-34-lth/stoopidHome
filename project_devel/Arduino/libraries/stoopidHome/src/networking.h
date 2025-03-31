#ifndef NETWORKING_H
#define NETWORKING_H
#pragma once

#include "def.h"
#include "globals.h"
#include <painlessMesh.h>
#include "tasker.h"
#include "setup.h"
class Tasker;

class Networking{
   

    private:
        painlessMesh *_mesh;
        uint32_t _rootNodeId;
        bool findRootNodeId();
        IPAddress _localIP;
        std::list<uint32_t> _nodeList;
        uint32_t _nodeId;
    public:
        Networking(painlessMesh *mesh);
        IPAddress getLocalIP();
        // Define callback types as function pointers
        using NewConnCallback = void (*)(uint32_t);
        using ChangedConnCallback = void (*)();
        using TimeAdjCallback = void (*)(int32_t);
        using ReceiveCallback = void (*)(uint32_t, String &);
        using DroppedConnCallback = void (*)(uint32_t);

        // Unified method to set up callbacks
        bool setupMeshCallbacks(
            NewConnCallback newConn = nullptr,
            ChangedConnCallback changedConn = nullptr,
            TimeAdjCallback timeAdj = nullptr,
            ReceiveCallback receive = nullptr,
            DroppedConnCallback droppedConn = nullptr);
        bool init(Tasker* tasker);
        uint32_t getRootNodeId();
        std::list<uint32_t> getNodeList();
        bool setContainsRoot(bool flag);
        bool setRoot(bool flag);
        void update();
        void broadcastMsg(String &msg);
        bool sendMsg(uint32_t &dest, String &msg);
        bool updateNodeList();
        std::list<uint32_t> getNodes();
        uint32_t getNodeId();
};




#endif