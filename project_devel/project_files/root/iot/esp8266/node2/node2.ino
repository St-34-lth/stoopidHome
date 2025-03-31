
#include <stoopidHome.h>

// # root node id 3940086381
painlessMesh meshConnection;
Scheduler scheduler;
Tasker tasker(scheduler);
Networking mesh(&meshConnection);
auto nodeIP = IPAddress(0, 0, 0, 0);
WaterSensor waterLevelSensor(waterSensorAnlgPin);
StepperIface stepper(STEPS_PER_REV, STEPPER_SPEED);
PumpInterface pump{};
bool setupStepper()
{
    // setup stepper motor configuration
    const uint8_t pins[] = {14U, 12U, 13U, 15U};
    const uint8_t continuousStates[] = {0, 0, 0, 0};
    const uint8_t discreteStates[] = {0, 0, 0, 0};
    iFaceState stepperState = iFaceState(pins, sizeof(pins) / sizeof(pins[0]),
                                         discreteStates, sizeof(discreteStates) / sizeof(discreteStates[0]),
                                         continuousStates, sizeof(continuousStates) / sizeof(continuousStates[0]));

    return (stepper.setup(stepperState));
}

bool setupPump()
{
    static const uint8_t pins[] = {D4, 0, 0, 0};
    static uint8_t discreteStates[] = {0, 0, 0, 0};
    static const uint8_t continuousStates[] = {0, 0, 0, 0};

    iFaceState pumpState = iFaceState(pins, sizeof(pins) / sizeof(pins[0]),
                                      discreteStates, sizeof(discreteStates) / sizeof(discreteStates[0]),
                                      continuousStates, sizeof(continuousStates) / sizeof(continuousStates[0]));

    return (pump.setup(pumpState));
};

bool setupNode()
{
    if (setupStepper() && setupPump())
    {
        return true;
    }
    return false;
};

// Function to check if water exists (using the water sensor)
bool waterExists()
{
    int waterLevel = analogRead(waterSensorAnlgPin);

    millis(); // Small delay for sensor stability

    return (waterLevel > 125 && waterLevel < 400); // Return true if water level is within the acceptable range
}

bool serveFood()
{
    auto close = stepperPos::CLOSE;
    auto open = stepperPos::OPEN;
    stepper.setStepperToPreset(open);
    int status = stepper.setStepperToPreset(close);

    if (status == 0)
    {
        return true;
    };

    return false;
}

bool serveWater()
{
    if (waterExists())
    {
        int status = pump.startPump();
        delay(500);
        status = pump.stopPump();
        if (status == 0)
        {
            return true;
        };
    }
    return false;
}

void gatherData()
{
    waterLevelSensor.read();
    waterLevelSensor.getData(sensorData);
};

void sendData()
{
    String msg;
    serializeJson(sensorData, msg);
    auto rootNodeId = mesh.getRootNodeId();
    mesh.sendMsg(rootNodeId, msg);
};

Task tGatherData(TASK_SECOND * 2, TASK_FOREVER, gatherData);
Task tSendData(TASK_SECOND * 3, TASK_FOREVER, sendData);

void setup()
{
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    mesh.init(&tasker);
    mesh.setupMeshCallbacks([](uint32_t nodeId)
                            {
                                mesh.updateNodeList();
                                auto lst = mesh.getNodes();
                                Serial.println("Node list changed");
                            },
                            &changedConnectionCb, &nodeTimeAdjustedCb, &onRcvCb, &onDroppedConnectionCb);

    if (setupNode())
    {

        Serial.println("Setup successful");
    }
    auto taskId1 = tasker.addTask(&tGatherData);
    auto taskId2 = tasker.addTask(&tSendData);

    tasker.enableTask(taskId1);
    tasker.enableTask(taskId2);
}

void loop()
{
    mesh.update();
    if (nodeIP != mesh.getLocalIP())
    {
        nodeIP = mesh.getLocalIP();
        Serial.println("My IP is " + nodeIP.toString());
    };
}