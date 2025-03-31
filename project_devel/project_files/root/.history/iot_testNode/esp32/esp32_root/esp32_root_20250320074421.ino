

// // Prototypes
// void receivedCallback(const uint32_t &from, const String &msg);
// void mqttCallback(char *topic, byte *payload, unsigned int length);
// IPAddress getlocalIP();
// IPAddress myIP(0, 0, 0, 0);
// IPAddress mqttBroker(192, 168, 1, 241);
// // painlessMesh mesh;
// // // WiFiClient wifiClient;
// // WiFiClientSecure wifiClient;
// // PubSubClient mqttClient(wifiClient);

// // void setup()
// // {
// //     Serial.begin(115200);
// //     //  Serial.println();
// //     // Serial.print("Connecting to ");
// //     // Serial.println(ssid);
  
// //     // wifiClient.setFingerprint(fingerprint);

// //     mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION | COMMUNICATION | MESH_STATUS);
// //     mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 4);
// //     mesh.onReceive(&receivedCallback);
// //     mesh.stationManual(ssid, password);
// //     mesh.setHostname(HOSTNAME);

// //     // Bridge node, should (in most cases) be a root node. See [the wiki](https://gitlab.com/painlessMesh/painlessMesh/wikis/Possible-challenges-in-mesh-formation) for some background
// //     mesh.setRoot(true);
// //     // This node and all other nodes should ideally know the mesh contains a root, so call this on all nodes
// //     mesh.setContainsRoot(true);
        
// //       wifiClient.setCACert(ca_cert);
// //     mqttClient.setServer(mqtt_server, 8883);    
// //     mqttClient.setCallback(mqttCallback);

// //     // mqttClient.connect(mqtt_server,mqtt_port,wifiClient);
// // }

// // void loop()
// // {
// //     mesh.update();
    
// //     if (myIP != getlocalIP())
// //     {
// //         myIP = getlocalIP();
// //         Serial.println("My IP is " + myIP.toString());
     
// //         // if (mqttClient.connect("painlessMeshClient", mqtt_username, mqtt_password))
// //         // if(mqttClient.connect(mqtt_server,"",""))
// //         // {
// //         //     mqttClient.publish("painlessMesh/from/gateway", "Ready!");
// //         //     mqttClient.subscribe("painlessMesh/to/#");
// //         // }
// //     }
// //     if(!mqttClient.connected())
// //     {
// //         mqtt_connect();
// //     }
// //     else 
// //     {
// //     mqttClient.loop();
    
// //     }
// // }

// // void receivedCallback(const uint32_t &from, const String &msg)
// // {
// //     Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
// //     String topic = "painlessMesh/from/" + String(from);
// //     mqttClient.publish(topic.c_str(), msg.c_str());
// // }

// // void mqttCallback(char *topic, uint8_t *payload, unsigned int length)
// // {
// //     char *cleanPayload = (char *)malloc(length + 1);
// //     memcpy(cleanPayload, payload, length);
// //     cleanPayload[length] = '\0';
// //     String msg = String(cleanPayload);
// //     free(cleanPayload);

// //     String targetStr = String(topic).substring(16);

// //     if (targetStr == "gateway")
// //     {
// //         if (msg == "getNodes")
// //         {
// //             auto nodes = mesh.getNodeList(true);
// //             String str;
// //             for (auto &&id : nodes)
// //                 str += String(id) + String(" ");
// //             mqttClient.publish("painlessMesh/from/gateway", str.c_str());
// //         }
// //     }
// //     else if (targetStr == "broadcast")
// //     {
// //         mesh.sendBroadcast(msg);
// //     }
// //     else
// //     {
// //         uint32_t target = strtoul(targetStr.c_str(), NULL, 10);
// //         if (mesh.isConnected(target))
// //         {
// //             mesh.sendSingle(target, msg);
// //         }
// //         else
// //         {
// //             mqttClient.publish("painlessMesh/from/gateway", "Client not connected!");
// //         }
// //     }
// // }

// // void mqtt_connect()
// // {

// //     Serial.print("MQTT connecting");
// //     while (!mqttClient.connect(mqtt_server, "", ""))
// //     {
// //         Serial.print(".");
// //         delay(1000);
// //     }
// //     Serial.println("connected!");
// //      mqttClient.publish("painlessMesh/from/gateway", "Ready!");
// //     mqttClient.subscribe("painlessMesh/to/#");

// // }

// IPAddress getlocalIP()
// {
//     return IPAddress(mesh.getStationIP());
// }
/************************************************************
 * Example: painlessMesh + PubSubClient (ESP32) with TLS
 *
 * This sketch:
 *   1) Sets up an ESP mesh network (painlessMesh).
 *   2) Forces this node to be the root and connect to your router (STA).
 *   3) Creates a secure TLS connection to an MQTT broker using WiFiClientSecure.
 *   4) Publishes/subscribes to a test topic.
 ************************************************************/


/********** Certificate(s) **********/
// This is a placeholder CA cert. Replace with your broker's CA certificate
// in PEM format (including the BEGIN/END lines). If you have multiple
// root/intermediate certificates, you can concatenate them.
// If you need mutual TLS (client certificates), see code comments below.


// MQTT credentials
// const char *mqtt_username = "user";
// const char *mqtt_password = "user";


/********** MQTT Broker Info **********/
// Use your broker's address and the secure (TLS) port, commonly 8883.
// For example, test.mosquitto.org can be used with TLS on port 8884,
// // or other providers use 8883, 443, etc. (Adjust accordingly.)




// /********** Global Objects **********/
// painlessMesh mesh;                  // The mesh instance
// WiFiClientSecure secureClient;       // For TLS
// PubSubClient mqttClient(secureClient);

// /********** Forward Declarations **********/
// void onMeshMessage(uint32_t from, String &msg);
// void onMeshConnectionsChanged();
// void connectToMqtt();
// void mqttCallback(char* topic, byte* payload, unsigned int length);

// // Optional: Give a unique name for the MQTT client ID
// // String nodeName = "RootNode";

// //********************************************************
// // SETUP
// //********************************************************

// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
// #include "esp_log.h"
// void setup() 
// {
//   Serial.begin(115200);
//   delay(2000);

//   // Initialize mesh
// esp_log_level_set("mbedtls", ESP_LOG_DEBUG);
// // or ESP_LOG_VERBOSE, if you want even more detail


//     mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION | COMMUNICATION | MESH_STATUS);
//     mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 4);
//     mesh.onReceive(&onMeshMessage);
//     mesh.onChangedConnections(&onMeshConnectionsChanged);

//   // Force this node to be root
//   mesh.setRoot(true);
//   mesh.setContainsRoot(true);
//     mesh.setHostname(HOSTNAME);
//   // Provide the router credentials (only the root node uses these in STA mode)
//   mesh.stationManual(ssid, password);

//   // Initialize the secure client with the CA certificate
//   secureClient.setCACert(mqtt_ca_cert);

//   // If using mutual TLS, also set your client cert & key:
//   secureClient.setCertificate(mqtt_client_cert);
//   secureClient.setPrivateKey(mqtt_client_key);

//   // Setup PubSubClient for MQTT
//   mqttClient.setServer(mqtt_server,mqtt_port);
//   mqttClient.setCallback(mqttCallback);

//   Serial.println("Setup complete. Waiting for mesh formation and STA connection...");
// }

// //********************************************************
// // MAIN LOOP
// //********************************************************
// void loop() 
// {
//   // Let mesh handle any events
//   mesh.update();

//   // If we are root, check if we have a valid station IP from the router
   
    
    
//       // We have a valid IP; ensure MQTT is connected
//       if (!mqttClient.connected()) {
//         connectToMqtt();
//       }
//       // Handle MQTT
//       mqttClient.loop();
   
//   }

#include <WiFi.h>             // ESP32 Wi-Fi
#include <AsyncTCP.h>         // Must be installed for ESP32 async
#include <WiFiClientSecure.h> // If you want secure client
#include <esp_wifi.h>         // Some Wi-Fi macros
#include <esp_system.h>       // esp_read_mac may be here on older cores
#include <esp_mac.h>          // Required on newer ESP-IDF for esp_read_mac type definitions

#include 
#include <painlessMesh.h>
const char client_cert[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDXjCCAkagAwIBAgIUEjeKYhNmN1Ene3FlLrsyBRXqx9AwDQYJKoZIhvcNAQEL\n"
    "BQAwRzELMAkGA1UEBhMCR1IxDzANBgNVBAgMBkF0dGljYTEPMA0GA1UEBwwGQXRo\n"
    "ZW5zMRYwFAYDVQQDDA0xOTIuMTY4LjEuMjQxMB4XDTI1MDMxNTEwMTkyNVoXDTMz\n"
    "MDczMTEwMTkyNVowRzELMAkGA1UEBhMCR1IxDzANBgNVBAgMBkF0dGljYTEPMA0G\n"
    "A1UEBwwGQXRoZW5zMRYwFAYDVQQDDA0xOTIuMTY4LjEuMjQxMIIBIjANBgkqhkiG\n"
    "9w0BAQEFAAOCAQ8AMIIBCgKCAQEAsOQJEUmsRWwdYd0JWzD/0nPDyeTzfDIa23ZG\n"
    "xts88NyyDGeLWyCH3z/UuXWGwDtTZg3/Q71wUzMIyhBMNdS4pb4iN3qlwK4LK33U\n"
    "pzPhy9fmqkXV8HyHlimxWJCILMMBfK+Zg/FOVNWqjoRQKzGvD0inW54v+6BNv0ao\n"
    "5PwbWIyPKpkxZlrkOiur0NySWPQaafJIWML50BOg3odFML5VYgU+6rrdycu68fes\n"
    "1BdqT65XArVP53h7Fo3Bw53N5/LoMTCrnP8qcqLOgyaNhCef/z9A/+PAwxCP2jB9\n"
    "/HUzCAJPUH/6HhZ6GqJKfHXEE5H0vDctfqcC88zFZ6RSxQHSgwIDAQABo0IwQDAd\n"
    "BgNVHQ4EFgQUZgK5MZTHpzt80KJYDMdFxk/I3BcwHwYDVR0jBBgwFoAUGwl8z1Pj\n"
    "6SkC9LF/oc2PlwUnoZEwDQYJKoZIhvcNAQELBQADggEBANUQZavB1CzZOST5Vysy\n"
    "xpxomdkzfjVLoOggOcj/iMMVLbK0ZXNFzxQzhBN0+7u8HzUreBRMiTc1NNArF2mj\n"
    "qusug+dpSRem4DuK06wYxrWcF+a4wfUTeGerrqL2OFRkBGJ92jVZRoHHKcUG1c7U\n"
    "92Ra1mxQE/Idf3gEcuKXrznqazjTWBntQDFvd5RSGlmzRAmTJGejT1ypMXoCOGWG\n"
    "sbN7352QK9OhS5FLuy000w5n7S3hNccDO00uqTSSVb5+WhGRubhAPfMB4DbZTmFR\n"
    "ceJrkIDxQx4HSgtOpMwTzmq8aM78pkqyY86+Ctthq81r3OVyyiGCJouHc4I/u7yZ\n"
    "7PU=\n"
    "-----END CERTIFICATE-----\n";

const char client_key[] =
    "-----BEGIN PRIVATE KEY-----\n"
    "MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQCw5AkRSaxFbB1h\n"
    "3QlbMP/Sc8PJ5PN8MhrbdkbG2zzw3LIMZ4tbIIffP9S5dYbAO1NmDf9DvXBTMwjK\n"
    "EEw11LilviI3eqXArgsrfdSnM+HL1+aqRdXwfIeWKbFYkIgswwF8r5mD8U5U1aqO\n"
    "hFArMa8PSKdbni/7oE2/Rqjk/BtYjI8qmTFmWuQ6K6vQ3JJY9Bpp8khYwvnQE6De\n"
    "h0UwvlViBT7qut3Jy7rx96zUF2pPrlcCtU/neHsWjcHDnc3n8ugxMKuc/ypyos6D\n"
    "Jo2EJ5//P0D/48DDEI/aMH38dTMIAk9Qf/oeFnoaokp8dcQTkfS8Ny1+pwLzzMVn\n"
    "pFLFAdKDAgMBAAECggEADkHQPJ1NrhS+M4Pku/2xiM1S06QelH0srqaUTXUghCCi\n"
    "6x6yBoi92sUNc1s2c/+ew67Pda7AQxHDHMIpL6eIWwzS5i9+SjmHkJxQ4PRst7Nx\n"
    "lc9mJGltENx7EkfSlJpXtYHTuOTglYmDzdPP6wNTTZiOAJ451ik/9Zw6LaaSL6tA\n"
    "ecrxaQnWpBZyzSN/JzR5c8qgW5Z2Jl0rKFEZqSYvgkD0+Zugc8f1DaobKclsi1wt\n"
    "pXdW6DUx3yrbpMhwzcKqb/PDP3ajdsPKuIPskDujHId5cyXiL5s1X1UutlZBgvwi\n"
    "ULl9ComFre7d6G7OgtWdiqD8arxiVz7iUFgyy72+kQKBgQD52U2H9qaCrrJfxRn4\n"
    "MGrWHdsfcgmkO7uEA/6AB7UzXmvjK4NEH9A6MsyP6Sm7NZzbmaDAHEBrrRS4Pe9M\n"
    "x4ZuzA1/C2Zj+eaw30eLSnQHATG7Nl/v1TUqw5PKJuW73ONucD7F3vxZLUIXJP5v\n"
    "p6owwHznVPi72clifogAGipH8QKBgQC1PugzsQ4+FpR/1aioxuOnhkCv37af+TuZ\n"
    "Hlcko+0QYkLH4qHO7GJBtD98E2R4wkNib5l7pXGVkddPigX0iUVkztJbbnUFDXpA\n"
    "qleeNdjRdD8itfWUW0OBuJZiOfL0iy/wj67iMa6pyVDL4PFX0Lbi9v4t6VhYfHV6\n"
    "Y4x2yvbVswKBgQCbKHrAc8kmIM0/Mk0VVyQRwkrA9XXNWxbxPfW/lonN09ZWJkCH\n"
    "fuY2LjPhLyAuVXYK3hX5F9JdzAew/y1r1gDDL8811WOJtLixoIdylhMGApTauULM\n"
    "0vAIlZpjENLcXqmSfPxOr1KxIq0HaH7l4g6L5g3rL1akefRZSVC2F4moUQKBgAbW\n"
    "1dpFfx0Wqwnb3R5pSuPibAR286Q7umT/wRgP2303x/9U7Pgq8VY7X/3lmW/5HKPV\n"
    "RDxnNM8JDs/gjHL6zxSLvWJX27CSHp/cDaxZqdHViiBcL2UmwHhfp8UTHLjfm7J/\n"
    "KYE9kLK1dwgKd/DxXkVYPqUq3fuQzroMkZtV81SDAoGBAKrgI/zZ5sFJkMPkqea0\n"
    "HQakY2djMtJTT+X2Wk8Rk6lasBgJKbr/gu0uTKOyaE2FexbimgABclT+Ih2CGSmF\n"
    "0yiPQNcZD0ftqFI2Kgry/sLtFTan85HdwjfK3cwjVATH5Wr4lk8FpoJJUBjTlGku\n"
    "HjNwXpHB2tGqhRVpBXHepnna\n"
    "-----END PRIVATE KEY-----\n";

const char ca_cert[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDbzCCAlegAwIBAgIUPuz0FuIDoSzWmZPVkewbv4cV3YIwDQYJKoZIhvcNAQEL\n"
    "BQAwRzELMAkGA1UEBhMCR1IxDzANBgNVBAgMBkF0dGljYTEPMA0GA1UEBwwGQXRo\n"
    "ZW5zMRYwFAYDVQQDDA0xOTIuMTY4LjEuMjQxMB4XDTI1MDMxNTEwMTgwN1oXDTMz\n"
    "MDczMTEwMTgwN1owRzELMAkGA1UEBhMCR1IxDzANBgNVBAgMBkF0dGljYTEPMA0G\n"
    "A1UEBwwGQXRoZW5zMRYwFAYDVQQDDA0xOTIuMTY4LjEuMjQxMIIBIjANBgkqhkiG\n"
    "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA5FoIyHNUfRTcVpmRCbaDeuNlG7uUMOX6qok9\n"
    "QjJEzY+fFabgwKs6OgLpVlIRbl4KAINr/G5H/4gq6zBCIXsbNSmW6YxoamxaOnzg\n"
    "7ke4k7XiFhtyKZmQ2G3E2HVnsaIx4dcRDV1xKSz70ukmzCU3Y+6myhiu7pfjIRiv\n"
    "nf2uwQOZYkAq0iieVmRYxIz8kG/h7c1gHl9hP85/Si/+q32lJ8x0yz3jzznSe3w7\n"
    "nWPt4USsYqEp6XMVlDeyYFC1m4qQUZrPxj4OJISjDKAjQgVqfBHhrmgmFYFZUyDG\n"
    "rjSRmG9z0YDb5mDrPYgeFyUnKRrfjWZVUakAHILt1L6RnxpMzwIDAQABo1MwUTAd\n"
    "BgNVHQ4EFgQUGwl8z1Pj6SkC9LF/oc2PlwUnoZEwHwYDVR0jBBgwFoAUGwl8z1Pj\n"
    "6SkC9LF/oc2PlwUnoZEwDwYDVR0TAQH/BAUwAwEB/zANBgkqhkiG9w0BAQsFAAOC\n"
    "AQEAwUSyCEvD1GLrtfGp7UiRALcLVC/W0j+xMCjWF57PvKAtLmoRXyyH7FuPdX1e\n"
    "UVKo4a5hNSE5Gv3wWBFB55wk9h/WXm0VrtbSk0RdrDHyatbqdxO2D3uoBy/Xkozf\n"
    "UCYy4ukqZNq0Q70hgNUoYLGxTxRZnylgoJ+qS/Yay1iCvQEpooRT0/wBZyHNtrWf\n"
    "BkcS4AmeszpMxbv7s33Cpq6xHQrMNZ0xpgM4/0WrSgxnUBsF/KnTF4R0YTfAWJwN\n"
    "L0BAVXA9jvwWXJk5sH5f6pipUUvoeWV+x5Geu/liDKVk4ft5cNELESGE6pagbi/O\n"
    "q6ER12p67oP895iCwYfq92mUpg==\n"
    "-----END CERTIFICATE-----\n";


#define MESH_PREFIX "stoopidMesh"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555
#define AP_CHANNEL 4 // should be the same as mesh channel according to painlessMesh
#define HOSTNAME "Bridge_node"
#define MESH_MODE WIFI_AP_STA

const char *ssid = "st34lth_2.4G";
const char *password = "Anatomist7-Tint-Rockiness-Ditch";
const char *mqtt_server = "192.168.1.241";
const int mqtt_port = 8883;

painlessMesh mesh;
/* create an instance of WiFiClientSecure */
WiFiClientSecure client;
PubSubClient mqtt_client(client);
// static const char *TAG = "WiFi_Debug";

// Forward declarations for mesh & MQTT callbacks
void onMeshMessage(uint32_t from, String &msg);
void onMeshConnectionsChanged();
void mqttCallback(char *topic, byte *payload, unsigned int length);

void setup()
{
    Serial.begin(115200);

    // Optional: Set verbose logging levels if you need debug output
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("wifi", ESP_LOG_VERBOSE);

    // 1) Initialize the mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION | COMMUNICATION | MESH_STATUS);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, MESH_MODE, AP_CHANNEL);
    mesh.stationManual(ssid, password);
    mesh.setHostname(HOSTNAME);

    // Force this node to be root
    mesh.setRoot(true);
    mesh.setContainsRoot(true);

    // Mesh callbacks
    mesh.onReceive(&onMeshMessage);
    mesh.onChangedConnections(&onMeshConnectionsChanged);

    // 2) Setup TLS with your certificates/keys
    client.setCACert(ca_cert);
    client.setCertificate(client_cert);
    client.setPrivateKey(client_key);

    // 3) Setup MQTT
    mqtt_client.setServer(mqtt_server, mqtt_port);
    // If you want to receive inbound messages:
    mqtt_client.setCallback(mqttCallback);
}

void loop()
{
    // Keep the mesh network running
    mesh.update();

    Serial.println("\nAttempting MQTT connection...");

    // Attempt to connect to MQTT broker
    if (mqtt_client.connect("ESP32_client"))
    {
        Serial.print("MQTT connected, client state: ");
        Serial.println(mqtt_client.state());
        // For example, subscribe to a topic once connected:
        // mqtt_client.subscribe("test/mesh");
    }
    else
    {
        Serial.print("MQTT connection failed, state: ");
        Serial.println(mqtt_client.state());
    }

    // Give it some time before re-trying
    delay(5000);
}

//================ MESH CALLBACKS ================

void onMeshMessage(uint32_t from, String &msg)
{
    Serial.printf("Mesh message from node %u: %s\n", from, msg.c_str());
    // If you want to forward mesh data up to MQTT:
    // if (mqtt_client.connected()) {
    //   mqtt_client.publish("mesh/incoming", msg.c_str());
    // }
}

void onMeshConnectionsChanged()
{
    Serial.println("Mesh connection changed:");
    SimpleList<uint32_t> nodes = mesh.getNodeList();
    Serial.printf("Now have %d nodes in the mesh.\n", nodes.size());
}

//================ MQTT CALLBACK ================

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("MQTT message [");
    Serial.print(topic);
    Serial.print("]: ");

    String message;
    for (unsigned int i = 0; i < length; i++)
    {
        message += (char)payload[i];
    }
    Serial.println(message);

    // If desired, broadcast to the mesh
    mesh.sendBroadcast("MQTT->Mesh: " + message);
}