#include "setup.h"

#pragma once 
#ifdef NODE_1

bool setupNode()
{

    if (setupLed() == 0)
    {
        Serial.println("Setup successful");
        return true;
    };
    return false;
};


void newConnectionCb(uint32_t nodeId)
{
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
    
}

void changedConnectionCb()
{
    Serial.printf("Changed connections\n");
    
}

void nodeTimeAdjustedCb(int32_t offset)
{
    // Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void onRcvCb(uint32_t from, String &msg)
{
    
};


void onDroppedConnectionCb(size_t nodeId)
{
    Serial.printf("Connection Dropped: %u\n", nodeId); 
};

#endif

#ifdef NODE_2

bool serveFood();
bool serveWater();
bool setupNode() ;


void newConnectionCb(uint32_t nodeId)
{
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCb()
{
    Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCb(int32_t offset)
{
    // Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void onRcvCb(uint32_t from, String &msg)
{
    Serial.printf("%s",msg);
    if (msg == "PUMP_ON")
    {
        
        if(serveWater() == 0)
        {
            Serial.println("Serving water");

        }
        else
        {
            Serial.println("Serving water failed");
        }
    }
    else if(msg=="STEPPER_ON")
    {
        
        if(serveFood()==true)

        {
            Serial.println("Serving food");
        }
        else
        {
            Serial.println("Serving food failed");
        }

    }
    else
    {
        Serial.println("nothing to do");
    };
};
void onDroppedConnectionCb(uint32_t nodeId)
{
    Serial.printf("Connection Dropped: %u\n", nodeId);
}
#endif

#ifdef NODE_3

bool setupNode()
{


};

void newConnectionCb(uint32_t nodeId)
{
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCb()
{
    Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCb(int32_t offset)
{
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void onRcvCb(uint32_t from, String &msg)
{
  
};
void onDroppedConnectionCb(size_t nodeId)
{
    Serial.printf("Connection Dropped: %u\n", nodeId);
}
#endif
#ifdef ROBOT_1

// void newConnectionCb(uint32_t nodeId)
// {
//     Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
// }

// void changedConnectionCb()
// {
//     Serial.printf("Changed connections\n");
// }

// void nodeTimeAdjustedCb(int32_t offset)
// {
  
// }

// void onRcvCb(uint32_t from, String &msg)
// {
//     if (msg == "MV_CMD")
//     {
//         Serial.println("mv_cmd rcvd");
//     }
//     else
//     {
//         Serial.println("nothing to do");
//     };
// };
// void onDroppedConnectionCb(size_t nodeId)
// {
//     Serial.printf("Connection Dropped: %u\n", nodeId);
// }
#endif
#ifdef ROBOT_2



void newConnectionCb(uint32_t nodeId)
{

 
};

void changedConnectionCb()
{
    Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCb(long offset)
{
    // Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
};

void onRcvCb(uint32_t from, String &msg)
{

   
};


void onDroppedConnectionCb(uint32_t nodeId)
{
    Serial.printf("Connection Dropped: %u\n", nodeId);
}
#endif 

#ifdef TESTING


void newConnectionCb(uint32_t nodeId)
{
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCb()
{
    Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCb(int32_t offset)
{
    // Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void onRcvCb(uint32_t from, String &msg) {

};

void onDroppedConnectionCb(size_t nodeId)
{
    Serial.printf("Connection Dropped: %u\n", nodeId);
};

#endif 