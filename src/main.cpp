#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char* ssid = "BattleBot";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Store last control state
int leftMotor = 0;
int rightMotor = 0;
bool solenoidActive = false;

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, data);
    if (!error) {
      leftMotor = doc["left"];
      rightMotor = doc["right"];
      solenoidActive = doc["solenoid"];
      Serial.printf("L: %d | R: %d | Solenoid: %d\n", leftMotor, rightMotor, solenoidActive);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);  // ESP32 creates its own network
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  // You can drive motors / trigger solenoid here based on leftMotor, rightMotor, solenoidActive
  delay(10);
}