#include <Arduino.h>
/**
 * 03. MQTT send and callback example
 *
 * Send MQTT messages to the IoT Guru cloud and register a callback function to handle incoming messages.
 *
 * You need:
 * - user short identifier (you can find it on the Account page)
 * - the device short identifier (you can find it on the Device page)
 * - the device key (you can find it on the Device page)
 *
 * Tutorial: https://iotguru.live/tutorials/devices
 *
 * Also you need:
 * - the node's key (you can find it on the Node page)
 * - the field name (you can find it ont the Field page)
 *
 * Tutorial: https://iotguru.live/tutorials/nodes
 * Tutorial: https://iotguru.live/tutorials/fields
 *
 * You can push MQTT messages to your device by using our API:
 *
 *     https://api.iotguru.live/mqtt/send/{nodeKey}/{fieldName}/{message}
 */
#include <IoTGuru.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
#endif

#ifdef ESP32
  #include <WiFi.h>
#endif
#include <SoftwareSerial.h> 
#include <SparkFunESP8266WiFi.h>
#include "main.h"
#include "secrets.h"

const char* ssid      = SECRET_SSID
const char* password  = SECRET_PASS;

WiFiClient client;

String userShortId    = USER_ID;
String deviceShortId  = DEVICE_ID;
String deviceKey      = DEVICE_KEY;
IoTGuru iotGuru = IoTGuru(userShortId, deviceShortId, deviceKey);

String nodeShortId    = node_ID[BottomFan];
String fieldName      = fieldName[BottomFan_tacho];

void setup() {
    Serial.begin(115200);
    delay(10);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(50);
        Serial.print(".");
    }
    Serial.println("");

    /**
     * Set the callback function.
     */
    iotGuru.setCallback(&callback);
    /**
     * Set the debug printer (optional).
     */
    iotGuru.setDebugPrinter(&Serial);
    /**
     * Set the network client.
     */
    iotGuru.setNetworkClient(&client);
}

volatile unsigned long nextSendUptime = 0;

void loop() {
    iotGuru.loop();

    if (nextSendUptime < millis()) {
        nextSendUptime = millis() + 60000;
        iotGuru.sendMqttValue(nodeShortId, fieldName, millis()/1000.0f);
    }
}

void callback(const char* nodeShortId, const char* fieldName, const char* message) {
    Serial.print(nodeShortId);Serial.print(" - ");Serial.print(fieldName);Serial.print(": ");Serial.println(message);
}
