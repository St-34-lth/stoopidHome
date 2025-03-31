#include <stoopidHome.h>

DHTsensor sensor1(DHT_SENSOR_PIN);
FireSensor fireDet(fireSensorAnlgPin);
LedIface led = LedIface();

const uint8_t pins[] = {rPin, gPin, bPin};
const uint8_t continuousStates[] = {0, 0, 0};
const uint8_t discreteStates[] = {0, 0, 0};

iFaceState ledState = {
    .pins = pins,
    .pinsArrSize = sizeof(pins) / sizeof(pins[0]),
    .discreteStates = discreteStates,
    .discArrSize = sizeof(discreteStates) / sizeof(discreteStates[0]),
    .contStates = continuousStates,
    .contArrSize = sizeof(continuousStates) / sizeof(continuousStates[0])};

void gatherData()
{
    sensor1.read();
    sensor1.getData(sensorData);
    fireDet.read();
    fireDet.getData(sensorData);
};

void checkForFlame()
{

    if (!led.isOn())
    {
        if (sensorData[0]["fireValue"] < 800)
        {
            led.shiftColor(ledColor::RED);
            Serial.println("led is turning red");
            return;
        }
    }
    else
    {
        if (sensorData[0]["fireValue"] > 800)
        {
            Serial.println("led turning off");
            led.reset();
            return;
        };
    }
};

void sendData()
{
    String msg;
    serializeJson(sensorData, msg);
    auto rootNodeId = mesh.getRootNodeId();
    mesh.sendMsg(rootNodeId, msg);
};

Task tGatherData(TASK_SECOND * 2, TASK_FOREVER, gatherData);
Task tCheckForFlame(TASK_SECOND * 2, TASK_FOREVER, checkForFlame);
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
                                Serial.println("Node list changed"); },
                            &changedConnectionCb, &nodeTimeAdjustedCb, &onRcvCb, &onDroppedConnectionCb);

    led.setup(ledState);

    auto taskId1 = tasker.addTask(&tGatherData);
    auto taskId3 = tasker.addTask(&tCheckForFlame);
    auto taskId2 = tasker.addTask(&tSendData);

    tasker.enableTask(taskId1);
    tasker.enableTask(taskId3);
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