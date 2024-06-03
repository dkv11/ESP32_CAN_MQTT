#include <mcp_can.h>
#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <ArduinoJson.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#define WIFI_SSID "Itsme"
#define WIFI_PASSWORD "Letsfly1!"
byte datasent[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];
MCP_CAN CAN0(5);
byte sndStat;
  String output;

//MQTT Topics
#define MQTT_PUB_BMS "esp/bms"

// BROKER
#define MQTT_HOST IPAddress(192,168,199,154)
#define MQTT_PORT 1883 
#define BROKER_USER "deepak"
#define BROKER_PASS "Deepak@2023"



AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time a message was published
const long interval = 1000;        // Interval at which to publish values

float voltageS, currentS, remainingSocS;
float temp[5] = { 0 };
float cellVoltage[19] = { 0 };
int cycleS;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Do whatever you want when you receive a message
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}




void setup() {
  Serial.begin(9600);
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(BROKER_USER, BROKER_PASS);
  connectToWifi();
}

void loop() {

  sndStat = CAN0.sendMsgBuf(0x100, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
  }
  //for 0X100
  sndStat = CAN0.sendMsgBuf(0x101, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 256) {
      float Total_Voltage = (rxBuf[0] << 8) | rxBuf[1];
      Serial.print("Total Voltage: ");
      Serial.print(Total_Voltage / 100);
      Serial.println(" Volt");
      voltageS = (Total_Voltage / 100);
      float Total_Current = (rxBuf[2] << 8) | rxBuf[3];
      Serial.print("Total Current: ");
      Serial.print(Total_Current / 100);
      Serial.println(" Amp");
      currentS = Total_Current / 100;
      float Remaining_Capacity = (rxBuf[4] << 8) | rxBuf[5];
      Serial.print("Remaining Capacity: ");
      Serial.print(Remaining_Capacity / 100);
      Serial.println(" Ah");
      remainingSocS = Remaining_Capacity / 100;
    }
  }


  //for 0X101
  sndStat = CAN0.sendMsgBuf(0x102, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 257) {
      float Full_capacity = (rxBuf[0] << 8) | rxBuf[1];
      Serial.print("Full_capacity: ");
      Serial.print(Full_capacity / 100);
      Serial.println(" Ah");
      int Discharge_Cycle = (rxBuf[2] << 8) | rxBuf[3];
      cycleS = Discharge_Cycle;
      Serial.print("Discharge_Cycle: ");
      Serial.println(Discharge_Cycle);
      float SOC = (rxBuf[4] << 8) | rxBuf[5];
      Serial.print("SOC: ");
      Serial.print(SOC);
      Serial.println("%");
    }
  }


  sndStat = CAN0.sendMsgBuf(0x103, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 258) {
      float Equilization_state_low_byte = (rxBuf[0] << 8) | rxBuf[1];
      Serial.print("Equilization_state_low_byte: ");
      Serial.println(Equilization_state_low_byte);
      float Equilization_state_high_byte = (rxBuf[2] << 8) | rxBuf[3];
      Serial.print("Equilization_state_high_byte: ");
      Serial.println(Equilization_state_high_byte);
      float Protection_status = (rxBuf[4] << 8) | rxBuf[5];
      Serial.print("Protection_status: ");
      Serial.println(Protection_status);
    }
  }

  //for 0X102

  //for 0X103
  sndStat = CAN0.sendMsgBuf(0x104, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 259) {
      float FET_Control_Status = (rxBuf[0] << 8) | rxBuf[1];
      Serial.print("FET_Control_Status: ");
      Serial.println(FET_Control_Status);
      float Production_date = (rxBuf[2] << 8) | rxBuf[3];
      Serial.print("Production_date: ");
      Serial.println(Production_date);
      float Software_Version = (rxBuf[4] << 8) | rxBuf[5];
      Serial.print("Software_Version: ");
      Serial.println(Software_Version);
    }
  }
  //for 0X104
  sndStat = CAN0.sendMsgBuf(0x105, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 260) {
      int No_Of_Battery_String = rxBuf[0];
      Serial.print("No_Of_Battery_String: ");
      Serial.println(No_Of_Battery_String);
      int NTC_No = rxBuf[1];
      Serial.print("NTC_No: ");
      Serial.println(NTC_No);
    }
  }


  //for 0X105
  sndStat = CAN0.sendMsgBuf(0x106, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 261) {
      float NTC_1 = (rxBuf[0] << 8) | rxBuf[1];
      temp[0] = (rxBuf[0] << 8 | rxBuf[1]) / 10.0 - 273.15;

      Serial.print("NTC_1: ");
      Serial.print((NTC_1 / 10) - 273.15);
      Serial.println(" °C");
      float NTC_2 = (rxBuf[2] << 8) | rxBuf[3];
      temp[1] = (rxBuf[2] << 8 | rxBuf[3]) / 10.0 - 273.15;

      Serial.print("NTC_2: ");
      Serial.print((NTC_2 / 10) - 273.15);
      Serial.println(" °C");
      float NTC_3 = (rxBuf[4] << 8) | rxBuf[5];
      temp[2] = (rxBuf[4] << 8 | rxBuf[5]) / 10.0 - 273.15;
      Serial.print("NTC_3: ");
      Serial.print((NTC_3 / 10) - 273.15);
      Serial.println(" °C");
    }
  }

  //for 0X106
  sndStat = CAN0.sendMsgBuf(0x107, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 262) {
      float NTC_4 = (rxBuf[0] << 8) | rxBuf[1];
      temp[3] = (rxBuf[0] << 8 | rxBuf[1]) / 10.0 - 273.15;

      Serial.print("NTC_4: ");
      Serial.print((NTC_4 / 10) - 273.15);
      Serial.println(" °C");
      float NTC_5 = (rxBuf[2] << 8) | rxBuf[3];
      temp[4] = (rxBuf[2] << 8 | rxBuf[3]) / 10.0 - 273.15;
      Serial.print("NTC_5: ");
      Serial.print((NTC_5 / 10) - 273.15);
      Serial.println(" °C");
    }
  }

  //for 0X107
  sndStat = CAN0.sendMsgBuf(0x108, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 263) {
      float Cell_1 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[0] = Cell_1 / 1000.0;
      Serial.print("Cell_1: ");
      Serial.print(Cell_1 / 1000);
      Serial.println(" Volt");
      float Cell_2 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[1] = Cell_2 / 1000.0;
      Serial.print("Cell_2: ");
      Serial.print(Cell_2 / 1000);
      Serial.println(" Volt");
      float Cell_3 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[2] = Cell_3 / 1000.0;
      Serial.print("Cell_3: ");
      Serial.print(Cell_3 / 1000);
      Serial.println(" Volt");
    }
  }

  //for 0X108
  sndStat = CAN0.sendMsgBuf(0x109, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 264) {
      float Cell_4 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[3] = Cell_4 / 1000.0;
      Serial.print("Cell_4: ");
      Serial.print(Cell_4 / 1000);
      Serial.println(" Volt");
      float Cell_5 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[4] = Cell_5 / 1000.0;
      Serial.print("Cell_5: ");
      Serial.print(Cell_5 / 1000);
      Serial.println(" Volt");
      float Cell_6 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[5] = Cell_6 / 1000.0;
      Serial.print("Cell_6: ");
      Serial.print(Cell_6 / 1000);
      Serial.println(" Volt");
    }
  }

  //for 0X109
  sndStat = CAN0.sendMsgBuf(0x10A, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 265) {
      float Cell_7 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[6] = Cell_7 / 1000.0;
      Serial.print("Cell_7: ");
      Serial.print(Cell_7 / 1000);
      Serial.println(" Volt");
      float Cell_8 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[7] = Cell_8 / 1000.0;
      Serial.print("Cell_8: ");
      Serial.print(Cell_8 / 1000);
      Serial.println(" Volt");
      float Cell_9 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[8] = Cell_9 / 1000.0;
      Serial.print("Cell_9: ");
      Serial.print(Cell_9 / 1000);
      Serial.println(" Volt");
    }
  }

  //for 0X10A
  sndStat = CAN0.sendMsgBuf(0x10B, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 266) {
      float Cell_10 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[9] = Cell_10 / 1000.0;
      Serial.print("Cell_10: ");
      Serial.print(Cell_10 / 1000);
      Serial.println(" Volt");
      float Cell_11 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[10] = Cell_11 / 1000.0;
      Serial.print("Cell_11: ");
      Serial.print(Cell_11 / 1000);
      Serial.println(" Volt");
      float Cell_12 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[11] = Cell_12 / 1000.0;
      Serial.print("Cell_12: ");
      Serial.print(Cell_12 / 1000);
      Serial.println(" Volt");
    }
  }

  //for 0X10B
  sndStat = CAN0.sendMsgBuf(0x10C, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 267) {
      float Cell_13 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[12] = Cell_13 / 1000.0;
      Serial.print("Cell_13: ");
      Serial.print(Cell_13 / 1000);
      Serial.println(" Volt");
      float Cell_14 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[13] = Cell_14 / 1000.0;
      Serial.print("Cell_14: ");
      Serial.print(Cell_14 / 1000);
      Serial.println(" Volt");
      float Cell_15 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[14] = Cell_15 / 1000.0;
      Serial.print("Cell_15: ");
      Serial.print(Cell_15 / 1000);
      Serial.println(" Volt");
    }
  }
  //for 0X10C
  sndStat = CAN0.sendMsgBuf(0x10D, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 268) {
      float Cell_16 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[15] = Cell_16 / 1000.0;
      Serial.print("Cell_16: ");
      Serial.print(Cell_16 / 1000);
      Serial.println(" Volt");
      float Cell_17 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[16] = Cell_17 / 1000.0;
      Serial.print("Cell_17: ");
      Serial.print(Cell_17 / 1000);
      Serial.println(" Volt");
      float Cell_18 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[17] = Cell_18 / 1000.0;
      Serial.print("Cell_18: ");
      Serial.print(Cell_18 / 1000);
      Serial.println(" Volt");
    }
  }

  //for 0X10D
  sndStat = CAN0.sendMsgBuf(0x10E, 8, datasent);
  if (sndStat == CAN_OK) {

    delay(1);
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (rxId == 269) {
      float Cell_19 = (rxBuf[0] << 8) | rxBuf[1];
      cellVoltage[18] = Cell_19 / 1000.0;
      Serial.print("Cell_19: ");
      Serial.print(Cell_19 / 1000);
      Serial.println(" Volt");
    }
  }

  

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {   
    previousMillis = currentMillis;
    
    StaticJsonDocument<5000> doc;
  doc["voltage"] = voltageS;
  doc["current"] = currentS;
  doc["RSOC"] = remainingSocS;
  doc["cycle"] = cycleS;
  JsonArray tempArray = doc.createNestedArray("temp");
  for (int i = 0; i < 5; i++) {  // Assuming 5 temperature readings
    tempArray.add(temp[i]);
  }

  JsonArray cellVoltageArray = doc.createNestedArray("cellVoltage");
  for (int i = 0; i < 19; i++) {  // Assuming 19 cell voltage readings
    cellVoltageArray.add(cellVoltage[i]);
  }

  serializeJson(doc, output);

  // Print BMS data to serial monitor
  Serial.println("Publishing BMS data to MQTT:");
  Serial.println(output);
    
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_BMS,1,true, String(output).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_BMS, packetIdPub1);
    Serial.printf("Message: %.2f \n", output);
}

Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
