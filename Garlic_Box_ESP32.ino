// Black Garlic Box Controller ESP32 //

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Wire.h>

#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <PubSubClient.h>
#include <WiFi.h>

#include "wifiCredentials.h"

//  setup variables

#define RELAY 12
#define ON HIGH
#define OFF LOW
#define LEDPIN 2

int REFRESH = 5000;

//Temp Probe Setup

#define ONE_WIRE_BUS 14

// set up thermometer bus, thermometer, and sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

float tempC;
float tempF;

int state;
int setpoint = 140;

// WiFi setup - details in wifiCredentials.h

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
const int mqttPort = MQTT_PORT;
const char* mqttServer = MQTT_SERVER;

String sensorType = "DS18B20";
const char* topic = "home/blackgarlic";

WiFiClient espClient;
PubSubClient client(espClient);


void setup() {

  Serial.begin(115200);
  Serial.println("************************");
  Serial.println("Black Garlic Temperature Controller");
  Serial.println("Greg and Elliot Hirson");
  Serial.println("2022, 2023");
  Serial.println("************************");

  pinMode(LEDPIN, OUTPUT);
  pinMode(RELAY, OUTPUT);

  // connect to Dallas Temp Sensor
  sensors.begin();
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");
  sensors.setResolution(insideThermometer, 11);

  sensors.requestTemperatures();

  tempC = sensors.getTempC(insideThermometer);
  tempF = DallasTemperature::toFahrenheit(tempC);

  //try to connect to WIFI
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);
  Serial.print("Connected at :");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);

}

void loop() {
  // put your main code here, to run repeatedly:

  // reconnect to MQTT
  if (!client.connected()) {
    reconnect();
  }


  sensors.requestTemperatures();
  tempC = sensors.getTempC(insideThermometer);
  tempF = DallasTemperature::toFahrenheit(tempC);

  // simple temperature controller loop
  if (tempF >= setpoint + 2) {
    digitalWrite(RELAY, OFF);
    state = 0;
  } else if (tempF <= (setpoint - 2)) {
    digitalWrite(RELAY, ON);
    state = 1;
  }

  Serial.print("Temp F");
  Serial.println(tempF);
  Serial.print("Relay State ");
  Serial.println(state);

  StaticJsonDocument<60> doc;
  char output[60];
  
  doc["sensor"] = sensorType;
  doc["temp"] = tempF;
  doc["state"] = state;

  serializeJson(doc, output);
  client.publish(topic, output);

  delay(REFRESH);

}


void reconnect() {
  // Loop until reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create Client
    String clientID = "ESP32Client-";
    clientID += String(random(0xffff), HEX);;

    if (client.connect(clientID.c_str())) {
      Serial.println("connected");
      digitalWrite(LEDPIN, HIGH);
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      digitalWrite(LEDPIN, LOW);
      delay(2000);
    }
  }
}
