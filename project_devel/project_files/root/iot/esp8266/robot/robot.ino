
#include "motors.h"

// # root node id 3940086381
auto nodeIP = IPAddress(0, 0, 0, 0);
float duration, distance;
// UltraSonicIface sounder{triggerPin, echoPin} ;

painlessMesh meshConnection;
Scheduler scheduler;
Tasker tasker(scheduler);
Networking mesh(&meshConnection);

// IMUIface imu;
int resolveMotorCommand(esp8266motorCmd cmd)
{
    int status = -1;
    Serial.println("In drive motors");

    // Decide which movement to perform based on dir/motor bits
    if ((cmd.dirA == 0 && cmd.dirB == 0) && (cmd.motorA == 1 && cmd.motorB == 1))
    {
        // Turn left
        status = moveLeft();
        Serial.println("left");
    }
    else if ((cmd.dirA == 1 && cmd.dirB == 1) && (cmd.motorA == 1 && cmd.motorB == 1))
    {
        // Turn right
        status = moveRight();
        Serial.println("right");
    }
    else if ((cmd.dirA == 1 && cmd.dirB == 0) && (cmd.motorA == 1 && cmd.motorB == 1))
    {
        // Move backward
        status = moveBackward();
        Serial.println("bwd");
    }
    else if ((cmd.dirA == 0 && cmd.dirB == 1) && (cmd.motorA == 1 && cmd.motorB == 1))
    {
        // Move forward
        status = moveForward();
        Serial.println("fwd");
    }
    else if (cmd.motorA == 0 && cmd.motorB == 0)
    {
        // STOP
        status = killMotors();
        Serial.println("stop");
    }
    else
    {
        Serial.println("Invalid motor command, staying neutral");
        status = stayNeutral();
    }

    if (status != 0)
    {
        Serial.println("Error executing motor command");
    }
    return status;
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

        // Expect "dirA","dirB","pwmA","pwmB" in the JSON
        const char *dirA = doc["dirA"];
        const char *pwmA = doc["pwmA"];
        const char *pwmB = doc["pwmB"];
        const char *dirB = doc["dirB"];

        if (!dirA || !pwmA || !pwmB || !dirB)
        {
            Serial.println("Incomplete motor command");
            return;
        }

        // Fill esp8266motorCmd struct
        // (Assuming '0'/'1' are chars => convert to int 0 or 1)
        esp8266motorCmd motorCmd;
        motorCmd.dirA = (dirA[0] == '1') ? 1 : 0;
        motorCmd.dirB = (dirB[0] == '1') ? 1 : 0;
        motorCmd.motorA = (pwmA[0] == '1') ? 1 : 0;
        motorCmd.motorB = (pwmB[0] == '1') ? 1 : 0;

        // Call your function that uses moveLeft(), moveRight(), etc.
        int result = resolveMotorCommand(motorCmd);
        if (result != 0)
        {
            // Possibly handle the error or just log it
            Serial.println("Motor command execution failed");
        }
    }
    else
    {
        Serial.println("nothing to do");
    }
}

void setup()
{
    Serial.begin(9600);
    Serial.setDebugOutput(true);

    mesh.init(&tasker);
    mesh.setupMeshCallbacks(
        [](uint32_t nodeId)
        {
            mesh.updateNodeList();
            auto lst = mesh.getNodes();
            
        },
        &changedConnectionCb,
        &nodeTimeAdjustedCb,
        &handleMotors,
        &onDroppedConnectionCb);

    if (setupMotors())
    {
        Serial.println("Motors ready");
    }
}

void loop()
{
    mesh.update();
}