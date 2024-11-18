#include <Arduino.h>
#include <BLEDevice.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "PubSubClient.h"
#include "config.h"
#include "Sensor.h"
#include "esp_camera.h"     // 카메라 라이브러리 추가

#include "esp_system.h"
#include "esp_log.h"
#include "mbedtls/base64.h"  // Base64 관련 헤더

#include <HTTPClient.h>
const char* serverURL = HTTP_URL;
/**
   A BLE client for the Xiaomi Mi Plant Sensor, pushing measurements to an MQTT server.

   See https://github.com/nkolban/esp32-snippets/blob/master/Documentation/BLE%20C%2B%2B%20Guide.pdf
   on how bluetooth low energy and the library used are working.

   See https://github.com/ChrisScheffler/miflora/wiki/The-Basics for details on how the 
   protocol is working.
   
   MIT License

   Copyright (c) 2017 Sven Henkel
   Multiple units reading by Grega Lebar 2018

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

// device count
static const int deviceCount = sizeof FLORA_DEVICES / sizeof FLORA_DEVICES[0];

// the remote service we wish to connect to
static const BLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");

// the characteristic of the remote service we are interested in
static const BLEUUID uuid_version_battery("00001a02-0000-1000-8000-00805f9b34fb");
static const BLEUUID uuid_sensor_data("00001a01-0000-1000-8000-00805f9b34fb");
static const BLEUUID uuid_write_mode("00001a00-0000-1000-8000-00805f9b34fb");

// the device topic
static const String deviceBaseTopic = String(MQTT_BASE_TOPIC) + "/" + String(DEVICE_ID) + "/device";

// the base sensor topic
static const String sensorBaseTopic = String(MQTT_BASE_TOPIC) + "/" + String(DEVICE_ID) + "/sensor";

// the capacity of the device json document
static const size_t deviceCapacity = JSON_OBJECT_SIZE(5) + 80;

// the capacity of the sensor json document
static const size_t sensorCapacity = JSON_OBJECT_SIZE(16) + 230;

// boot count used to check if battery status should be read
RTC_DATA_ATTR int bootCount = 0;

TaskHandle_t hibernateTaskHandle = NULL;

WiFiClient espClient;
PubSubClient client(espClient);

void connectWifi(ArduinoJson::JsonDocument &jsonDocument)
{
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected\n");

  jsonDocument["id"] = DEVICE_ID;
  jsonDocument["ipAddress"] = WiFi.localIP().toString();
  jsonDocument["mac"] = WiFi.macAddress();
  jsonDocument["channel"] = WiFi.channel();
  jsonDocument["rssi"] = WiFi.RSSI();
  Serial.println("\nWiFi address\n");
  const char * temp = WiFi.localIP().toString().c_str();
  Serial.println(temp);
  
}

void disconnectWifi()
{
  WiFi.disconnect(true);
  Serial.println("WiFi disonnected");
}

void connectMqtt()
{
  Serial.println("Connecting to MQTT...");
  client.setServer(MQTT_HOST, MQTT_PORT);
  String lwtTopic = deviceBaseTopic + "/lwt";
  String deviceMac = WiFi.macAddress();
  String part = deviceMac.substring(9);
  part.replace(":", "");
  String mqttClientId = String(DEVICE_ID) + "_" + part;

  Serial.printf("- MQTT Client ID: %s\n", mqttClientId.c_str());

  while (!client.connected())
  {
    if (!client.connect(mqttClientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD, lwtTopic.c_str(), 1, true, "offline"))
    {
      Serial.print("MQTT connection failed:");
      Serial.print(client.state());
      Serial.println("Retrying...");
      delay(MQTT_RETRY_WAIT);
    }
    else
    {
      client.publish(lwtTopic.c_str(), "online", MQTT_RETAIN);
    }
  }
  Serial.println("MQTT connected");
  Serial.println("");
}

void disconnectMqtt()
{
  client.publish(String(deviceBaseTopic + "/lwt").c_str(), "offline", MQTT_RETAIN);
  client.disconnect();
  Serial.println("MQTT disconnected");
}

BLEClient *getFloraClient(BLEAddress floraAddress)
{
  BLEClient *floraClient = BLEDevice::createClient();

  if (!floraClient->connect(floraAddress))
  {
    Serial.println("- Connection failed, skipping");
    return nullptr;
  }
  Serial.println("- Connection successful");
  return floraClient;
}

BLERemoteService *getFloraService(BLEClient *floraClient)
{
  BLERemoteService *floraService = nullptr;

  try
  {
    floraService = floraClient->getService(serviceUUID);
  }
  catch (...)
  {
    // something went wrong
  }
  if (floraService == nullptr)
  {
    Serial.println("- Failed to find data service");
  }
  else
  {
    Serial.println("- Found data service");
  }

  return floraService;
}

bool forceFloraServiceDataMode(BLERemoteService *floraService)
{
  BLERemoteCharacteristic *floraCharacteristic;

  // get device mode characteristic, needs to be changed to read data
  Serial.println("- Force device in data mode");
  floraCharacteristic = nullptr;
  try
  {
    floraCharacteristic = floraService->getCharacteristic(uuid_write_mode);
  }
  catch (...)
  {
    // something went wrong
  }
  if (floraCharacteristic == nullptr)
  {
    Serial.println("-- Failed, skipping device");
    return false;
  }

  // write the magic data
  uint8_t buf[2] = {0xA0, 0x1F};
  floraCharacteristic->writeValue(buf, 2, true);

  delay(500);
  return true;
}

byte calculateMeasurementLevel(int current, int min, int max)
{
  if (current <= min)
  {
    return 0; // low
  }
  else if (current >= max)
  {
    return 2; // high
  }
  else
  {
    return 1; // medium
  }
}

bool readFloraDataCharacteristic(BLERemoteService *floraService, ArduinoJson::JsonDocument &jsonDocument, Sensor sensor)
{
  BLERemoteCharacteristic *floraCharacteristic = nullptr;

  // get the main device data characteristic
  Serial.println("- Access characteristic from device");
  try
  {
    floraCharacteristic = floraService->getCharacteristic(uuid_sensor_data);
  }
  catch (...)
  {
    // something went wrong
  }
  if (floraCharacteristic == nullptr)
  {
    Serial.println("-- Failed, skipping device");
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from characteristic");
  std::string value;
  try
  {
    value = floraCharacteristic->readValue();
  }
  catch (...)
  {
    // something went wrong
    Serial.println("-- Failed, skipping device");
    return false;
  }
  const char *val = value.c_str();

  Serial.print("Hex: ");
  for (int i = 0; i < 16; i++)
  {
    Serial.print((int)val[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

  int16_t *temp_raw = (int16_t *)val;
  float temperature = (*temp_raw) / ((float)10.0);
  Serial.printf("-- Temperature: %f\n", temperature);

  int moisture = val[7];
  Serial.printf("-- Moisture: %d\n", moisture);

  int light = val[3] + val[4] * 256;
  Serial.printf("-- Light: %d\n", light);

  int conductivity = val[8] + val[9] * 256;
  Serial.printf("-- Conductivity: %d\n", conductivity);

  if (temperature < -20 || temperature > 50)
  {
    Serial.println("-- Unreasonable values received for temperature, skip publish");
    return false;
  }

  if (moisture < 0 || moisture > 100)
  {
    Serial.println("-- Unreasonable values received for moisture, skip publish");
    return false;
  }

  if (light < 0)
  {
    Serial.println("-- Unreasonable values received for light, skip publish");
    return false;
  }

  if (conductivity < 0 || conductivity > 5000)
  {
    Serial.println("-- Unreasonable values received for conductivity, skip publish");
    return false;
  }

  // add all valid measurment values to the json
  jsonDocument["temperature"] = temperature;
  jsonDocument["temperatureLevel"] = calculateMeasurementLevel((int)temperature, sensor.getMinTemperature(), sensor.getMaxTemperature());
  jsonDocument["moisture"] = moisture;
  jsonDocument["moistureLevel"] = calculateMeasurementLevel(moisture, sensor.getMinMoisture(), sensor.getMaxMoisture());
  jsonDocument["light"] = light;
  jsonDocument["lightLevel"] = calculateMeasurementLevel(light, sensor.getMinLight(), sensor.getMaxLight());
  jsonDocument["conductivity"] = conductivity;
  jsonDocument["conductivityLevel"] = calculateMeasurementLevel(conductivity, sensor.getMinLight(), sensor.getMaxLight());

  return true;
}

bool readFloraBatteryCharacteristic(BLERemoteService *floraService, ArduinoJson::JsonDocument &jsonDocument)
{
  BLERemoteCharacteristic *floraCharacteristic = nullptr;

  // get the device battery characteristic
  Serial.println("- Access battery characteristic from device");
  try
  {
    floraCharacteristic = floraService->getCharacteristic(uuid_version_battery);
  }
  catch (...)
  {
    // something went wrong
  }
  if (floraCharacteristic == nullptr)
  {
    Serial.println("-- Failed, skipping battery level");
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from characteristic");
  std::string value;
  try
  {
    value = floraCharacteristic->readValue();
  }
  catch (...)
  {
    // something went wrong
    Serial.println("-- Failed, skipping battery level");
    return false;
  }
  const char *val2 = value.c_str();
  int battery = val2[0];

  jsonDocument["battery"] = battery;
  jsonDocument["batteryLow"] = battery <= BATTERY_THRESHOLD_LOW;
  jsonDocument["batteryLevel"] = calculateMeasurementLevel(battery, String(BATTERY_THRESHOLD_LOW).toInt(), String(BATTERY_THRESHOLD_MED).toInt());

  return true;
}

bool processFloraService(BLERemoteService *floraService, bool readBattery, ArduinoJson::JsonDocument &jsonDocument, Sensor sensor)
{
  // set device in data mode
  if (!forceFloraServiceDataMode(floraService))
  {
    return false;
  }

  bool dataSuccess = readFloraDataCharacteristic(floraService, jsonDocument, sensor);

  bool batterySuccess = true;
  if (readBattery)
  {
    batterySuccess = readFloraBatteryCharacteristic(floraService, jsonDocument);
  }

  return dataSuccess && batterySuccess;
}

bool processFloraDevice(BLEAddress floraAddress, bool getBattery, int tryCount, ArduinoJson::JsonDocument &jsonDocument, Sensor sensor)
{
  Serial.print("Processing Flora device at ");
  Serial.print(floraAddress.toString().c_str());
  Serial.print(" (try ");
  Serial.print(tryCount);
  Serial.println(")");

  // connect to flora ble server
  BLEClient *floraClient = getFloraClient(floraAddress);
  if (floraClient == nullptr)
  {
    return false;
  }

  // connect data service
  BLERemoteService *floraService = getFloraService(floraClient);
  if (floraService == nullptr)
  {
    floraClient->disconnect();
    return false;
  }

  Serial.printf("- %s has ble RSSI of %d\n", floraAddress.toString().c_str(), floraClient->getRssi());  
  jsonDocument["rssi"] = floraClient->getRssi();

  // process devices data
  bool success = processFloraService(floraService, getBattery, jsonDocument, sensor);

  // disconnect from device
  floraClient->disconnect();

  return success;
}

void hibernate()
{
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000ll);
  Serial.println("Going to sleep now.");
  delay(100);
  esp_deep_sleep_start();
}

void delayedHibernate(void *parameter)
{
  delay(EMERGENCY_HIBERNATE * 1000); // delay
  Serial.println("Something got stuck, entering emergency hibernate...");
  hibernate();
}

// 카메라 초기화 함수
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;  // 이미지 해상도 설정
  config.jpeg_quality = 12;            // 이미지 품질 설정
  config.fb_count = 1;

  // 카메라 초기화
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }
  Serial.println("Camera initialized successfully");
}

void captureAndSendImageViaMQTT() {
  camera_fb_t * fb = esp_camera_fb_get(); // 이미지 캡처
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  // HTTP 요청 시작
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "image/jpeg");

  // HTTP POST 요청으로 이미지 전송
  int httpResponseCode = http.POST(fb->buf, fb->len);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Response: " + response);
  } else {
    Serial.println("Error on sending POST: " + String(httpResponseCode));
  }

  // 버퍼 해제
  esp_camera_fb_return(fb);

  // HTTP 요청 종료
  http.end();
}

void setup()
{
  // all action is done when device is woken up
  Serial.begin(115200);
  delay(1000);

  // increase boot count
  bootCount++;

  // create a hibernate task in case something gets stuck
  xTaskCreate(delayedHibernate, "hibernate", 4096, NULL, 1, &hibernateTaskHandle);

  Serial.println("Initialize BLE client...");
  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P9);

 // WiFi 및 MQTT 연결
  DynamicJsonDocument deviceJson(deviceCapacity);
  connectWifi(deviceJson);
  connectMqtt();

  // 카메라 초기화 추가
  initCamera();  // **카메라 초기화**

  // publish device status
  char payload[deviceCapacity];
  serializeJson(deviceJson, payload);
  String deviceStatusTopic = deviceBaseTopic + "/status";

  Serial.printf("Device status: %s\n", payload);
  if (client.publish(deviceStatusTopic.c_str(), payload, MQTT_RETAIN))
  {
    Serial.printf("- Publishing device status -> %s\n", deviceStatusTopic.c_str());
  }
  else
  {
    Serial.println("- Publishing device status failed!");
  }

  // check if battery status should be read - based on boot count
  bool readBattery = ((bootCount % BATTERY_INTERVAL) == 0);

  // process devices
  for (int i = 0; i < deviceCount; i++)
  {
    Serial.println();
    int retryCount = 0;

    Sensor sensor;
    sensor.setMac(FLORA_DEVICES[i][0]);
    sensor.setLocation(FLORA_DEVICES[i][1]);
    sensor.setPlantId(FLORA_DEVICES[i][2]);
    sensor.setMinTemperature(FLORA_DEVICES[i][3].toInt());
    sensor.setMaxTemperature(FLORA_DEVICES[i][4].toInt());
    sensor.setMinMoisture(FLORA_DEVICES[i][5].toInt());
    sensor.setMaxMoisture(FLORA_DEVICES[i][6].toInt());
    sensor.setMinLight(FLORA_DEVICES[i][7].toInt());
    sensor.setMaxLight(FLORA_DEVICES[i][8].toInt());
    sensor.setMinConductivity(FLORA_DEVICES[i][9].toInt());
    sensor.setMaxConductivity(FLORA_DEVICES[i][10].toInt());

    // create sensor Json Document
    DynamicJsonDocument sensorJson(sensorCapacity);
    sensorJson["id"] = sensor.getPlantId();
    sensorJson["location"] = sensor.getLocation();
    sensorJson["mac"] = sensor.getMac();

    BLEAddress bleAddress(sensor.getMac().c_str());
    while (retryCount < SENSOR_RETRY)
    {
      retryCount++;
      sensorJson["retryCount"] = retryCount;

      // create sensor topic
      String sensorTopic = sensorBaseTopic + "/" + sensor.getLocation() + "/" + sensor.getPlantId();

      if (processFloraDevice(bleAddress, readBattery, retryCount, sensorJson, sensor))
      {
        char payload[sensorCapacity];
        serializeJson(sensorJson, payload);
        Serial.printf("- Sensor payload: %s\n", payload);
        if (client.publish(sensorTopic.c_str(), payload, MQTT_RETAIN))
        {
          Serial.printf("-- Publishing --> %s\n", sensorTopic.c_str());
        }
        else
        {
          Serial.println("-- Publishing failed!");
        }
        // **이미지 캡처 및 MQTT 전송 추가**
        captureAndSendImageViaMQTT();  // **이미지를 MQTT로 전송**
        break;
      }
      delay(3000); // wait for another try
    }
    delay(2000); // wait for next sensor
  }

  // disconnect mqtt and wifi
  disconnectMqtt();
  disconnectWifi();

  // delete emergency hibernate task
  vTaskDelete(hibernateTaskHandle);

  // go to sleep now
  hibernate();
}

void loop()
{
  /// we're not doing anything in the loop, only on device wakeup
  delay(10000);
}