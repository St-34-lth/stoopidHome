#ifndef SOCKETIOCLIENTMOD_H
#define SOCKETIOCLIENTMOD_H

#pragma once 
#include <WebSocketsClient.h>
#include <SocketIOclient.h>

class SocketIOclientMod : public SocketIOclient
{
public:
    bool sendBIN(uint8_t *payload, size_t length, bool headerToPayload = false);
};

#endif 