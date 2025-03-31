#include <WiFi.h>
#include <esp32cam.h>
#include <Arduino.h>
#include <SocketIOclientMod.hpp>

const char *ssid = "ssid";
const char *password = "pwd";

// Socket.IO server info
const char *socketServerIP = "<ip>";
const uint16_t socketPort = 8675;
const char *socketPath = "/socket.io/?EIO=4";

SocketIOclientMod socketIO;

void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case sIOtype_DISCONNECT:
        Serial.println("[IOc] Disconnected!");
        break;
    case sIOtype_CONNECT:
        Serial.printf("[IOc] Connected to URL: %s\n", payload);
        socketIO.send(sIOtype_CONNECT, "/");
        break;
    case sIOtype_EVENT:
        Serial.printf("[IOc] EVENT: %.*s\n", length, (char *)payload);
        break;
    default:
        Serial.printf("[IOc] Type %d: %.*s\n", type, length, (char *)payload);
        break;
    }
}

int setupSocketClient()
{
  socketIO.begin(socketServerIP, socketPort, socketPath);
  socketIO.onEvent(socketIOEvent);
  socketIO.loop();
  return socketIO.isConnected() ? 0 : -1;
}

void setup()
{
    Serial.begin(115200);
    Serial.println();
    esp32cam::setLogger(Serial);
    delay(1000);

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.printf("WiFi failure %d\n", WiFi.status());
        delay(5000);
        ESP.restart();
    }
    Serial.println("WiFi connected");
    delay(1000);
    
    {
        using namespace esp32cam;

        auto initialResolution = Resolution::find(1024, 768);

        Config cfg;
        cfg.setPins(pins::AiThinker);
        cfg.setResolution(initialResolution);
        cfg.setBufferCount(10);
        cfg.setJpeg(80);

        bool ok = Camera.begin(cfg);
        if (!ok)
        {
            Serial.println("camera initialize failure");
            delay(5000);
            ESP.restart();
        }
        Serial.println("camera initialize success");
    }
    setupSocketClient();
}

void loop()
{
   
    socketIO.loop();

     // Send a frame every x seconds
    static unsigned long lastFrameTime = 0;
    if (socketIO.isConnected() && millis() - lastFrameTime > 25)
    {
        lastFrameTime = millis();

        // 1) Capture a frame
        auto fb = esp32cam::capture();
        if (!fb)
        {
            Serial.println("Failed to capture image");
            return;
        }
        if(fb)
        {
            const char *eventNamePayload = "[\"chunk_start\"]";
            socketIO.sendEVENT(eventNamePayload);
        }
        // 2) We want to send an event called "frame_binary"
        
        
        // a) Send the EVENT name as text
        // b) Immediately send the binary data next

        // "sendEVENT" for the text, then sIOtype_BINARY for the data.

        // 2a) Send the EVENT name as JSON: ["frame_binary"]
        
        

        // 2b) Send the raw binary data
        // socketIO.send(sIOtype_BINARY_EVENT, fb->data(), fb->size());
        // uint8_t empty_data[1] = {0};
        // socketIO.send(sIOtype_BINARY_EVENT, empty_data, 0);
        socketIO.sendBIN(fb->data(), fb->size());

        // const char *eventNamePayload = "[\"chunk_end\"]";
        // socketIO.sendEVENT(eventNamePayload);
        // Release the frame buffer
        // fb->release();

        Serial.printf("[loop] Sent %d bytes of raw binary\n", fb->size());
    }
}