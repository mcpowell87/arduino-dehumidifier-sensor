#include "DHT.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

//#define DEBUG

#define WLAN_SSID SECRET_WLAN_SSID
#define WLAN_PASS SECRET_WLAN_PASS
#define MQTT_SERVER SECRET_MQTT_SERVER
#define MQTT_PORT SECRET_MQTT_PORT

#define MQTT_TOPIC_TEMP "home/sensor/dehumidifier/temp"
#define MQTT_TOPIC_BUCKET_STATUS "home/sensor/dehumidifier/bucket_status"
#define MQTT_TOPIC_AVAILABILITY "home/sensor/dehumidifier/availability"

WiFiClient client;
PubSubClient mqttClient;

#define DHTPIN 13
#define DHTTYPE DHT22
#define DHT_READ_INTERVAL 5000
#define LDR_READ_INTERVAL 100
#define CAPTURE_INTERVAL 5000 //4 seconds
#define LDRPIN A0

#define LED_ON_THRESHOLD 300
#define FLASH_RATE 1000

#define LED_ON 1
#define LED_OFF 0

int lastLdrReading;
int lastLdrReadTime;
int lastFlashTime;
int captureStartTime;
int flashCount;
bool flashing;
bool ldrCapturing;
float lastHumidity;
float lastTemperature;
int lastDhtReadTime;
int lastBucketMqttMessageTime;

DHT dht(DHTPIN, DHTTYPE, 15);

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  flashCount = 0;
  flashing = false;
  ldrCapturing = false;
  Serial.begin(9600);
  dht.begin(60);

  connectWifi();
  mqttClient.setClient(client);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  delay(1500);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  delay(LDR_READ_INTERVAL);

  readLdr();
  readDht();

  // Send an MQTT message every 10 seconds with the current status of the LED in case something falls through the cracks.
  if (millis() - lastBucketMqttMessageTime >= 10000) {
    publishBucketStatus(flashing);
  }
}

int readLdr() {
  int currentLdrRead = analogRead(LDRPIN);
  int currentTime = millis();

  if (ldrCapturing && currentTime - captureStartTime >= CAPTURE_INTERVAL) {
    ldrCapturing = false;
    // Capture period done, check if it was flashing
    if (flashCount >=4) {
      flashing = true;
      #ifdef DEBUG
      Serial.println("Possible flashing detected");
      #endif
    }
    else {
      flashing = false;
    }
    publishBucketStatus(flashing);
    flashCount = 0;
    
    #ifdef DEBUG
    Serial.println("Ending capture time.");
    #endif
  }
  // If there is a state change (ON -> OFF or OFF -> ON)
  else if (getLedState(currentLdrRead) != getLedState(lastLdrReading)) {
    if (!ldrCapturing) {
      ldrCapturing = true;
      captureStartTime = currentTime;

      #ifdef DEBUG
      Serial.println("Beginning capture period");
      #endif
    }
    if (currentTime - lastFlashTime >= 500) {
      flashCount += 1;
      lastFlashTime = currentTime;
      #ifdef DEBUG
      if (getLedState(currentLdrRead) == LED_ON) {
        Serial.print("LED On - Detected Reading: ");
      }
      else {
        Serial.print("LED Off - Detected Reading: ");
      }
      Serial.println(currentLdrRead);
      #endif
    }
  }
  else if (!ldrCapturing) {
    flashing = false;
  }

  lastLdrReadTime = currentTime;
  lastLdrReading = currentLdrRead;
}

int getLedState(int ldrReading) {
  if (ldrReading >= LED_ON_THRESHOLD) {
    return LED_ON;
  }
  return LED_OFF;
}

void readDht() {
  if(millis() - lastDhtReadTime >= DHT_READ_INTERVAL) {
    lastHumidity = dht.readHumidity();
    lastTemperature = dht.readTemperature(true);

    lastDhtReadTime = millis();

    if (isnan(lastHumidity) || isnan(lastTemperature)) {
      Serial.println(F("Failed to read from DHT sensor."));
      return;
    }
    publishTemp(lastTemperature, lastHumidity);
  }
}

void connectWifi() {
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void publishTemp(float temp, float humidity) {
  StaticJsonDocument<200> doc;
  doc["temperature"] = (String)temp;
  doc["humidity"] = (String)humidity;
  char data[200];
  Serial.print("Sending MQTT temp: ");
  serializeJson(doc, Serial);
  Serial.println();
  serializeJson(doc, data);
  mqttClient.publish(MQTT_TOPIC_TEMP, data, true);
}

void publishBucketStatus(bool isFlashing) {
  Serial.print("Sending MQTT Bucket Status with: ");
  Serial.println(isFlashing ? "full" : "empty");
  lastBucketMqttMessageTime = millis();
  mqttClient.publish(MQTT_TOPIC_BUCKET_STATUS, isFlashing ? "full" : "empty", true);
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266-dehumidifier";
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}