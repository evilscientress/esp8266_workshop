/**
 * ESP_1OneWire.ino
 *
 *  Created on: 24.05.2015
 *
 */

#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <PubSubClient.h>

#define USE_SERIAL Serial
#define SSID "mqtt_workshop"
#define PASSWORD "VerySecurePassword"
#define MQTT_SERVER "mqtt.masterbase.at"
#define MQTT_USER "mrmcd"
#define MQTT_PASSWORD "VerySecurePassword"
#define LEDPIN 2

ESP8266WiFiMulti WiFiMulti;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

bool mqttReconnect() {
    // Loop until we're reconnected
    if (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect

        if (mqttClient.connect(WiFi.macAddress().c_str(), MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            //Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            //delay(5000);
            Serial.println();
        }
    }
    return mqttClient.connected();
}

void setup() {
    //setup LED GPIO
    digitalWrite(2, HIGH);
    pinMode(2, OUTPUT);
    
    //setup button GPIOs
    pinMode(D1, INPUT_PULLUP);
    pinMode(D2, INPUT_PULLUP);
    pinMode(D3, INPUT_PULLUP);

    USE_SERIAL.begin(115200);
   // USE_SERIAL.setDebugOutput(true);

    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

    /*for(uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }*/
    USE_SERIAL.print("[SETUP]\n");
    delay(10);
    USE_SERIAL.print("[WIFI] setup/connecting]\n");
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    
    mqttClient.setServer(MQTT_SERVER, 1883);

}

bool tempValid = false;
unsigned int tempTimestamp = micros();
String tempPayload;
unsigned int nextTemp = micros();

// waist memory but is faster
uint8_t button1_state = HIGH;
uint8_t button2_state = HIGH;
uint8_t button3_state = HIGH;

void loop() {
    if (mqttReconnect()) {
        if (button1_state == HIGH and digitalRead(D1) == LOW) {
            mqttClient.publish("mrmcd/button_demo/button", "1");
            USE_SERIAL.println(F("button 1 pushed"));
        }
        if (button2_state == HIGH and digitalRead(D2) == LOW) {
            mqttClient.publish("mrmcd/button_demo/button", "2");
            USE_SERIAL.println(F("button 2 pushed"));
        }
        if (button3_state == HIGH and digitalRead(D3) == LOW) {
            mqttClient.publish("mrmcd/button_demo/button", "3");
            USE_SERIAL.println(F("button 3 pushed"));
        }
        button1_state = digitalRead(D1);
        button2_state = digitalRead(D2);
        button3_state = digitalRead(D3);
    }
       
    mqttClient.loop();
    delay(100);
}

