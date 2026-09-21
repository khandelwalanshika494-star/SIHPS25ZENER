#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- Pin Definitions ---
#define VIB_PIN       26
#define CRACK_PIN     27
#define STRETCH_PIN   34
#define ONE_WIRE_PIN  4
#define LED_PIN       2

// --- Network & MQTT Settings ---
const char* ssid = "Wokwi-GUEST"; // Wokwi's built-in open Wi-Fi
const char* password = "";
const char* mqtt_server = "broker.emqx.io"; // Free public MQTT broker
const int   mqtt_port = 1883;
const char* mqtt_topic = "mines/subsidence/telemetry";

WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_MPU6050 mpu;
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature tempSensor(&oneWire);

// Thresholds
const float TILT_WATCH_DEG = 5.0;
const float TILT_ALERT_DEG = 15.0;
const int   STRETCH_ALERT_RAW = 3000;

enum NodeState { NORMAL, WATCH, ALERT };
NodeState state = NORMAL;

unsigned long lastReport = 0;
unsigned long reportInterval = 5000; // 5-second baseline for smooth live demo

void connectWiFi() {
  Serial.print("Connecting to Wokwi Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected successfully!");
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("[MQTT] Connecting to broker...");
    String clientId = "ESP32_Wokwi_Client_" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println(" connected!");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2 seconds...");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(VIB_PIN, INPUT_PULLUP);
  pinMode(CRACK_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);

  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
  tempSensor.requestTemperatures();

  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  sensors_event_t a, g, tempEvent;
  mpu.getEvent(&a, &g, &tempEvent);

  float tiltRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float tiltPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float totalTilt = sqrt(tiltRoll * tiltRoll + tiltPitch * tiltPitch);

  bool vib = (digitalRead(VIB_PIN) == LOW);
  bool crack = (digitalRead(CRACK_PIN) == LOW);
  int stretchRaw = analogRead(STRETCH_PIN);
  float crackDisp = (stretchRaw / 4095.0) * 50.0;

  if (crack || totalTilt > TILT_ALERT_DEG || stretchRaw > STRETCH_ALERT_RAW) {
    state = ALERT;
    reportInterval = 2000;
  } else if (vib || totalTilt > TILT_WATCH_DEG) {
    state = WATCH;
    reportInterval = 4000;
  } else {
    state = NORMAL;
    reportInterval = 6000;
  }

  digitalWrite(LED_PIN, state == ALERT ? HIGH : LOW);

  if (millis() - lastReport >= reportInterval) {
    lastReport = millis();

    float tempC = tempSensor.getTempCByIndex(0);
    tempSensor.requestTemperatures();

    // Prepare JSON payload
    char payload[256];
    snprintf(payload, sizeof(payload),
      "{\"node_id\":\"NODE_WOKWI_01\",\"state\":\"%s\",\"metrics\":{\"tilt_x\":%.2f,\"tilt_y\":%.2f,\"vibration\":%d,\"crack_switch\":%d,\"crack_disp_mm\":%.2f,\"temp_c\":%.1f}}",
      (state == ALERT) ? "CRITICAL" : (state == WATCH) ? "WARNING" : "NORMAL",
      tiltRoll, tiltPitch, vib ? 1 : 0, crack ? 1 : 0, crackDisp, tempC
    );

    // Publish directly to public broker
    client.publish(mqtt_topic, payload);
    Serial.print("[MQTT Published]: ");
    Serial.println(payload);
  }
}