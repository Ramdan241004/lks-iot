#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

#define DHT_PIN 15
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

Adafruit_MPU6050 mpu;

WiFiClientSecure net;
PubSubClient client(net);

void AdaFruitSetup() {
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
        case MPU6050_RANGE_2_G:  Serial.println("+-2G"); break;
        case MPU6050_RANGE_4_G:  Serial.println("+-4G"); break;
        case MPU6050_RANGE_8_G:  Serial.println("+-8G"); break;
        case MPU6050_RANGE_16_G: Serial.println("+-16G"); break;
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
        case MPU6050_RANGE_250_DEG:  Serial.println("+-250 deg/s"); break;
        case MPU6050_RANGE_500_DEG:  Serial.println("+-500 deg/s"); break;
        case MPU6050_RANGE_1000_DEG: Serial.println("+-1000 deg/s"); break;
        case MPU6050_RANGE_2000_DEG: Serial.println("+-2000 deg/s"); break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
        case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
        case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
        case MPU6050_BAND_94_HZ:  Serial.println("94 Hz");  break;
        case MPU6050_BAND_44_HZ:  Serial.println("44 Hz");  break;
        case MPU6050_BAND_21_HZ:  Serial.println("21 Hz");  break;
        case MPU6050_BAND_10_HZ:  Serial.println("10 Hz");  break;
        case MPU6050_BAND_5_HZ:   Serial.println("5 Hz");   break;
    }
}

void connectAWS() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi Connected!");

    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);

    Serial.print("Connecting to AWS IoT");
    while (!client.connected()) {
        if (client.connect(THINGNAME)) {
            Serial.println("\n✅ Connected to AWS IoT");
            if (client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC)) {
                Serial.print("✅ Subscribed to: ");
                Serial.println(AWS_IOT_SUBSCRIBE_TOPIC);
            } else {
                Serial.println("⚠️ Subscribe failed!");
            }
        } else {
            Serial.print(".");
            delay(1000);
        }
    }
}

void publishMessage() {
    StaticJsonDocument<200> doc;
    sensors_event_t a, g, temperature;
    mpu.getEvent(&a, &g, &temperature);

    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temp)) {
        Serial.println("⚠️ Temp NaN, set -999");
        temp = -999;
    }
    if (isnan(humidity)) {
        Serial.println("⚠️ Humidity NaN, set -1");
        humidity = -1;
    }

    doc["accel_x"] = a.acceleration.x;
    doc["accel_y"] = a.acceleration.y;
    doc["accel_z"] = a.acceleration.z;
    doc["gyro_x"]  = g.gyro.x;
    doc["gyro_y"]  = g.gyro.y;
    doc["gyro_z"]  = g.gyro.z;
    doc["humidity"]= humidity;
    doc["temp"]    = temp;

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);

    bool ok = client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
    Serial.print("Publishing: ");
    Serial.println(jsonBuffer);
    Serial.print("Result: ");
    Serial.println(ok ? "OK ✅" : "FAILED ❌");
}

void messageHandler(char *topic, byte *payload, unsigned int length) {
    Serial.print("📩 Incoming message on topic: ");
    Serial.println(topic);

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
        Serial.print("⚠️ JSON parse failed: ");
        Serial.println(error.c_str());
        return;
    }
    const char *message = doc["message"];
    Serial.print("Payload: ");
    Serial.println(message);
}

void setup() {
    Serial.begin(115200);
    AdaFruitSetup();
    dht.begin();
    delay(2000);
    connectAWS();
}

void loop() {
    if (!client.connected()) {
        Serial.println("⚠️ MQTT disconnected, reconnecting...");
        connectAWS();
    }
    client.loop();
    publishMessage();
    delay(1000);
}
