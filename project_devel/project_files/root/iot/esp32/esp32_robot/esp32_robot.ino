
#include <stoopidHome.h>

#ifdef ROBOT_2


float duration, distance;
UltraSonicIface sounder{triggerPin, echoPin} ;

painlessMesh meshConnection;
Scheduler scheduler;
Tasker tasker(scheduler);
Networking mesh(&meshConnection);

// auto nodeIP = IPAddress(0, 0, 0, 0);
// long a;
// IMUIface imu;


// (Configure pins in setup or pass them through iFaceState.)
MotorIface leftMotor(8 /*resolutionBits*/, 5000 /*freqHz*/);
MotorIface rightMotor(8 /*resolutionBits*/, 5000 /*freqHz*/);
motorCmds resolveMotorCommand(char dir, char pwm)
{
    // If pwm == '0' then STOP, ignoring dir
    if (pwm == '0')
    {
        return motorCmds::STOP;
    }

    // If pwm == '1' we look at dir to pick FORWARD or REVERSE
    if (dir == '0')
    {
        return motorCmds::FORWARD;
    }
    else
    {
        return motorCmds::REVERSE;
    }
}

void handleMotors(uint32_t from, String &msg)
{
    StaticJsonDocument<200> doc;
    DeserializationError err = deserializeJson(doc, msg);
    if (err)
    {
        Serial.println("JSON parse error; ignoring message");
        return;
    }

    const char *cmd = doc["cmd"];
    if (!cmd)
    {
        Serial.println("No cmd field; ignoring message");
        return;
    }

    if (strcmp(cmd, "MV_CMD") == 0)
    {
        Serial.println("mv_cmd received");

        // Grab the four fields controlling the motors
        const char *dirA = doc["dirA"]; // e.g. "0" or "1"
        const char *pwmA = doc["pwmA"]; // e.g. "0" or "1"
        const char *pwmB = doc["pwmB"]; // e.g. "0" or "1"
        const char *dirB = doc["dirB"]; // e.g. "0" or "1"

        if (!dirA || !pwmA || !pwmB || !dirB)
        {
            Serial.println("Incomplete motor command");
            return;
        }

        // Convert them to motorCmds
        motorCmds leftCommand = resolveMotorCommand(dirA[0], pwmA[0]);
        motorCmds rightCommand = resolveMotorCommand(dirB[0], pwmB[0]);

        // Now tell each MotorIface to move
        // e.g. FORWARD, REVERSE, or STOP
        leftMotor.setPresetControl(leftCommand);
        rightMotor.setPresetControl(rightCommand);

        // For debugging
        Serial.printf("LeftMotor: dir=%s,pwm=%s => %d  RightMotor: dir=%s,pwm=%s => %d\n",
                      dirA, pwmA, leftCommand, dirB, pwmB, rightCommand);
        
    }
    else
    {
        Serial.println("nothing to do");
    };
};
bool initMotors()
{
    static const uint8_t pins[] = {25U, 26U};
    static const uint8_t continuousStates[] = {0, 0};
    static const uint8_t discreteStates[] = {0, 0};
    auto leftState = iFaceState(pins, sizeof(pins) / sizeof(pins[0]),
                        discreteStates, sizeof(discreteStates) / sizeof(discreteStates[0]),
                        continuousStates, sizeof(continuousStates) / sizeof(continuousStates[0]));
    
    static const uint8_t pins2[] = {33U, 32U};
    auto rightState = iFaceState(pins2, sizeof(pins2) / sizeof(pins2[0]),
                            discreteStates, sizeof(discreteStates) / sizeof(discreteStates[0]),
                            continuousStates, sizeof(continuousStates) / sizeof(continuousStates[0]));

    bool okL = leftMotor.setup(leftState);
    bool okR = rightMotor.setup(rightState);

    return (okL && okR);
}

void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    mesh.init(&tasker);
    //can also take in lambdas if necessary
    mesh.setupMeshCallbacks([](uint32_t nodeId)
                            {
                                mesh.updateNodeList();
                                auto lst = mesh.getNodes();
                            },
                            &changedConnectionCb, 
                            &nodeTimeAdjustedCb, 
                            &handleMotors, 
                            &onDroppedConnectionCb);

    if(initMotors())
    {
        Serial.println("Motors ready");
    }
    
}

void loop()
{
  
 mesh.update();
  
}


#endif 

#ifdef TESTING

#include "../../include/SR04.cpp"
#include "../../include/def.h"
#include "../../include/globals.cpp"
#include "../../include/tests.h"

#define MPU6500_ADDR 0x68

const char *ssid = "st34lth_2.4G";
const char *password = "Anatomist7-Tint-Rockiness-Ditch";
// const int triggerPin = 27;
// /* select the pin for the blue LED */
// const int echoPin = 34;
// float duration, distance;
// SR04 sr04 = SR04(echoPin, triggerPin);
// long a;
// IMUIface imu;
void setup()
{
    Serial.begin(115200);
    // imu.begin();
    
   
}

void loop()
{
    // imu.read();
    // imu.getData(sensorData);
    // Serial.println(" \n");
    // serializeJson(sensorData, Serial);
    // delay(200);

    TestRunner::run();
}



// void loop()
// {

// }

#endif 