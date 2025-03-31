#ifndef SETUP_H 
#define SETUP_H
#pragma once 

#include "globals.h"
#include "def.h"


#ifdef NODE_1

bool setupNode();
bool setupLed();
bool setupNodeCbs();

void receivedCallback( uint32_t from, String &msg ) ;
void newConnectionCallback(uint32_t nodeId) ;
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset) ;
void onRcvCb(uint32_t from, String &msg);
void onDroppedConnectionCb(size_t nodeId);

#endif 

#ifdef NODE_2
bool setupNode();

void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void onRcvCb(uint32_t from, String &msg);
void onDroppedConnectionCb(size_t nodeId);

#endif

#ifdef NODE_3

#endif

#ifdef ROBOT_1


#endif

#ifdef ROBOT_2
motorCmds resolveMotorCommand(char dir, char pwm);


void newConnectionCb(uint32_t nodeId);
void changedConnectionCb();
void nodeTimeAdjustedCb(long offset);
void onRcvCb(uint32_t from, String &msg);
void onDroppedConnectionCb(uint32_t nodeId);

#endif
#ifdef TESTING 

bool setupNode();
bool setupNodeCbs();

void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void onRcvCb(uint32_t from, String &msg);

#endif 

#endif 