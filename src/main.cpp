#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "esp_task_wdt.h"

// ---------- Config ----------
const int pwmLeftPin = 12;   // GPIO 12
const int pwmRightPin = 10;  // GPIO 10

const int solenoidPinA = 32; // GPIO 32
const int solenoidPinB = 33; // GPIO 33

const char* ssid = "JPBattleBot-WiFi";
const char* password = "12345678";

// ---------- Globals ----------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Adafruit_MPU6050 mpu;

int leftMotor = 0;
int rightMotor = 0;
bool solenoidActive = false;

// ---------- Setup ----------
void setup() {
  // ✅ Always start USB Serial early
  Serial.begin(115200);
  delay(1000); // Allow USB to initialize
  Serial.println("Boot successful. Initializing...");

  // 🛡️ Optional software watchdog
  esp_task_wdt_init(10, true); // 10s timeout, panic on timeout
  esp_task_wdt_add(NULL);

  // Wi-Fi AP Mode
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // WebSocket
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, (char *)data) == DeserializationError::Ok) {
          leftMotor = doc["left"];
          rightMotor = doc["right"];
          solenoidActive = doc["solenoid"];
          Serial.printf("L: %d | R: %d | Solenoid: %d\n", leftMotor, rightMotor, solenoidActive);
        } else {
          Serial.println("Invalid control JSON.");
        }
      }
    }
  });
  server.addHandler(&ws);
  server.begin();

  // Motor PWM setup
  ledcSetup(0, 50, 16); // Channel 0, 50 Hz, 16-bit
  ledcAttachPin(pwmLeftPin, 0);

  ledcSetup(1, 50, 16); // Channel 1, 50 Hz, 16-bit
  ledcAttachPin(pwmRightPin, 1);

  // Solenoid pins
  pinMode(solenoidPinA, OUTPUT);
  pinMode(solenoidPinB, OUTPUT);
  digitalWrite(solenoidPinA, LOW);
  digitalWrite(solenoidPinB, LOW);

  // Gyro setup
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check wiring!");
    while (true) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("✅ MPU6050 connected");
}

// ---------- Main Loop ----------
void loop() {
  esp_task_wdt_reset();  // 🛡 Keep watchdog alive

  // Map motor values (-100 to 100) to PWM duty (1000-2000us)
  int leftDuty = map(leftMotor, -100, 100, 1000, 2000);
  int rightDuty = map(rightMotor, -100, 100, 1000, 2000);

  int leftPWM = (leftDuty * 65535L) / 20000;
  int rightPWM = (rightDuty * 65535L) / 20000;

  ledcWrite(0, leftPWM);
  ledcWrite(1, rightPWM);

  // Toggle double solenoid
  digitalWrite(solenoidPinA, solenoidActive ? HIGH : LOW);
  digitalWrite(solenoidPinB, solenoidActive ? LOW : HIGH);

  // Gyro readout
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.printf("Gyro X: %.2f Y: %.2f Z: %.2f\n", g.gyro.x, g.gyro.y, g.gyro.z);

  delay(10);
}