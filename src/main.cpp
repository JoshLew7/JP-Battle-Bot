#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "driver/twai.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <BleGamepad.h>

// ---------- Config ----------
#define USE_PWM true  // Set to false to use CAN

const int pwmLeftPin = 18;
const int pwmRightPin = 19;

const int solenoidPin = 23;

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

const char* ssid = "JPBattleBot-WiFi";
const char* password = "12345678";

Adafruit_MPU6050 mpu;
BleGamepad bleGamepad;

// ---------- Globals ----------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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

  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();

  bleGamepad.begin();
  Serial.println("BLE Gamepad started");

  if (USE_PWM) {
    ledcAttachPin(pwmLeftPin, 0);
    ledcSetup(0, 1000, 8);
    ledcAttachPin(pwmRightPin, 1);
    ledcSetup(1, 1000, 8);
  } else {
    twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = CAN_TX_PIN,
      .rx_io = CAN_RX_PIN,
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 5,
      .rx_queue_len = 5,
      .alerts_enabled = TWAI_ALERT_NONE,
      .clkout_divider = 0,
      .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    twai_timing_config_t t_config = {
      .brp = 4,
      .tseg_1 = 15,
      .tseg_2 = 4,
      .sjw = 3,
      .triple_sampling = false
    };

    twai_filter_config_t f_config = {
      .acceptance_code = 0,
      .acceptance_mask = 0xFFFFFFFF,
      .single_filter = true
    };

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
        twai_start() == ESP_OK) {
      Serial.println("CAN driver started");
    } else {
      Serial.println("Failed to start CAN driver");
    }
  }

  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);

  Wire.begin(21, 22);
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
  if (bleGamepad.isConnected()) {
    // Optionally: react to BLE gamepad inputs in future
  }

  int leftValue = constrain(map(leftMotor, -100, 100, 0, 255), 0, 255);
  int rightValue = constrain(map(rightMotor, -100, 100, 0, 255), 0, 255);

  if (USE_PWM) {
    ledcWrite(0, leftValue);
    ledcWrite(1, rightValue);
  } else {
    twai_message_t msgLeft = {
      .identifier = 0x01,
      .data_length_code = 1,
      .data = { (uint8_t)leftValue }
    };
    twai_message_t msgRight = {
      .identifier = 0x02,
      .data_length_code = 1,
      .data = { (uint8_t)rightValue }
    };
    twai_transmit(&msgLeft, pdMS_TO_TICKS(10));
    twai_transmit(&msgRight, pdMS_TO_TICKS(10));
  }

  digitalWrite(solenoidPin, solenoidActive ? HIGH : LOW);

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