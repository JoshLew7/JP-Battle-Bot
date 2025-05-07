#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------- Config ----------
#define USE_PWM true  // Set to false to use CAN

// Motor PWM pins
const int pwmLeftPin = 12;   // GPIO 12
const int pwmRightPin = 10;  // GPIO 10

// Solenoid control pins
const int solenoidPinA = 32; // GPIO 32
const int solenoidPinB = 33; // GPIO 33

// Wi-Fi credentials
const char* ssid = "JPBattleBot-WiFi";
const char* password = "12345678";

// ---------- Globals ----------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Adafruit_MPU6050 mpu;

int leftMotor = 0;
int rightMotor = 0;
bool solenoidActive = false;

// ---------- Control Handler ----------
void applyControl(const String& jsonString) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  if (!error) {
    leftMotor = doc["left"];
    rightMotor = doc["right"];
    solenoidActive = doc["solenoid"];
    Serial.printf("L: %d | R: %d | Solenoid: %d\n", leftMotor, rightMotor, solenoidActive);
  } else {
    Serial.println("Invalid control JSON.");
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    String msg = String((char*)data);
    applyControl(msg);
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // Initialize Wi-Fi
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Initialize WebSocket
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();

  // Initialize PWM for motors
  if (USE_PWM) {
    ledcSetup(0, 50, 16); // Channel 0, 50 Hz, 16-bit resolution
    ledcAttachPin(pwmLeftPin, 0);

    ledcSetup(1, 50, 16); // Channel 1, 50 Hz, 16-bit resolution
    ledcAttachPin(pwmRightPin, 1);
  }

  // Initialize solenoid control pins
  pinMode(solenoidPinA, OUTPUT);
  pinMode(solenoidPinB, OUTPUT);
  digitalWrite(solenoidPinA, LOW);
  digitalWrite(solenoidPinB, LOW);

  // Initialize MPU6050
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check wiring!");
    while (1) delay(10);
  }
  Serial.println("✅ MPU6050 connected");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

// ---------- Loop ----------
void loop() {
  // Map motor values (-100 to 100) to PWM duty cycle (1000 to 2000 microseconds)
  int leftDuty = map(leftMotor, -100, 100, 1000, 2000);
  int rightDuty = map(rightMotor, -100, 100, 1000, 2000);

  // Convert microseconds to duty cycle for 16-bit resolution at 50Hz
  int leftPWM = (leftDuty * 65535) / 20000;
  int rightPWM = (rightDuty * 65535) / 20000;

  if (USE_PWM) {
    ledcWrite(0, leftPWM);
    ledcWrite(1, rightPWM);
  }

  // Control solenoid
  if (solenoidActive) {
    digitalWrite(solenoidPinA, HIGH);
    digitalWrite(solenoidPinB, LOW);
  } else {
    digitalWrite(solenoidPinA, LOW);
    digitalWrite(solenoidPinB, HIGH);
  }

  // Read MPU6050 data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Gyro (deg/s): X=");
  Serial.print(g.gyro.x);
  Serial.print(" Y=");
  Serial.print(g.gyro.y);
  Serial.print(" Z=");
  Serial.println(g.gyro.z);

  delay(10);
}