/**
 * MIT License
 *
 * Copyright (c) 2020 Christopher Mumford
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <SparkFun_RHT03.h> // Temperature/Humidity sensor library.
#include <SparkFunCCS811.h> // Air quality sensor library.
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MQTT_FONA.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ESP8266_ESP12)
  #include <ESP8266WiFi.h>
#endif

#include "arduino_secrets.h"

namespace {

// Delay between readings in milliseconds.
const unsigned long kSensorReadingDelay = 60 * 1000;

// RHT03 data pin.
const int RHT03_DATA_PIN = 23;

// The I2C bus address for the air quality sensor.
// Default is 0x5B. Desolder the "ADDR" jumper to switch sensor to 0x5A.
const int CCS811_ADDR = 0x5B;

const String kSensorLocation = "hallway";
const char kSSID[] = SECRET_SSID;
const char kPASS[] = SECRET_PASS;
const char kMQTTBroker[] = "10.0.9.116";
const int  kMQTTPort = 1883;
const char kMQTTTopic[] = "sensors/inside_sensor_1";

/**
 * The data read from the various environmental sensors.
 */
struct EnvironmentalData {
  bool have_temp = false;    // True if the temp/humidity values are valid.
  bool have_quality = false; // True if the air quality values are valid.
  float humidity = 0.0f;     // The air relative humidity in % (0..100).
  float temperature = 0.0f;  // The air temperature in °F.
  int eCO2 = 0;              // Estimated carbon dioxide level (ppm).
  int TVOC = 0;              // Total volatile organic compounds (ppb).
};

// The global object to interract with the temperature/humidity sensor.
RHT03 g_temp_humidity_sensor;

// The global object to interract with the air quality sensor.
// Sensor is not accurate until after 20 minutes of runtime.
CCS811 g_air_quality_sensor(CCS811_ADDR);

// The main object connecting to the WiFi network.
WiFiClient g_WiFi_client;

// The MQTT Client.
Adafruit_MQTT_Client g_MQTT_client(&g_WiFi_client, kMQTTBroker, kMQTTPort, /*username=*/"", /*key=*/"");

// Object that publishes a specific topic to the MQTT broker.
Adafruit_MQTT_Publish g_MQTT_environ_publisher(&g_MQTT_client, kMQTTTopic, /*QOS=*/1);

/**
 * Retrieve the temperature and humidity from the sensor and write them to
 * |data|.
 * 
 * Return true if successful, false if not.
 */
bool getTempHumidity(EnvironmentalData* data) {
  if (g_temp_humidity_sensor.update() != 1) {
    // If the update failed, try delaying for RHT_READ_INTERVAL_MS ms before
    // trying again.
    delay(RHT_READ_INTERVAL_MS);
    return false;
  }

  data->temperature = g_temp_humidity_sensor.tempF();
  data->humidity = g_temp_humidity_sensor.humidity();
  data->have_temp = true;

  return true;
}

/**
 * Retrieve the air quality from the sensor and write them to |data|.
 * 
 * Return true if successful, false if not.
 */
bool getAirQuality(EnvironmentalData* data) {
  if (!g_air_quality_sensor.dataAvailable())
    return false;
  // Set sensor humidity/temp to improve accuracy.
  if (data->have_temp) {
    g_air_quality_sensor.setEnvironmentalData(data->humidity,
                                             (data->temperature-32.0f) / 1.8f);
  }
  g_air_quality_sensor.readAlgorithmResults();
  data->eCO2 = g_air_quality_sensor.getCO2();
  data->TVOC = g_air_quality_sensor.getTVOC();
  data->have_quality = true;
  return true;
}

/**
 * Determine if the device is currently connected to the WiFi network.
 */
bool WiFiConnected() {
  switch (WiFi.status()) {
    case WL_CONNECTED:
    case WL_IDLE_STATUS:
      return true;
    case WL_NO_SHIELD:
    case WL_NO_SSID_AVAIL:
    case WL_SCAN_COMPLETED:
    case WL_CONNECT_FAILED:
    case WL_CONNECTION_LOST:
    case WL_DISCONNECTED:
    default:
      return false;
  }
}

/**
 * Get the environmental data from all sensors and write to |data|.
 * 
 * Return true if successful, false if not.
 */
bool getData(EnvironmentalData* data) {
  return getTempHumidity(data) || getAirQuality(data);
}

/**
 * Connect to the WiFi network.
 * 
 * The radio takes a little while to connect, so the first call will
 * likely fail.
 * 
 * Return true if connected, false if not.
 */
bool ConnectToWiFi() {
  const int kMaxAttempts = 3;

  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(kSSID);
  int status = WiFi.begin(kSSID, kPASS);
  for (int attempt = 0; status != WL_CONNECTED && attempt < kMaxAttempts; attempt++) {
    // wait 10 seconds for the connection.
    delay(10 * 1000);
    status = WiFi.status();
  }

  return status == WL_CONNECTED;
}

/**
 * Log the enrivonmental data to the MQTT server.
 * 
 * Return true if successful, false if not.
 */
bool writeData(const EnvironmentalData& data) {
  if (!WiFiConnected()) {
    if (g_MQTT_client.connected())
      g_MQTT_client.disconnect();

    if (!ConnectToWiFi()) {
      Serial.println("Unable to connect to WiFi network.");
      return false;
    }
    Serial.println("WiFi is connected.");
  }

  int ret = -1;
  if ((ret = g_MQTT_client.connect()) != 0) {
    Serial.println(g_MQTT_client.connectErrorString(ret));
    return false;
  }

  Serial.println("MQTT client connected.");
  
  // Format the message payload in InfluxDB format.
  String msg = String("environment,location=") + kSensorLocation;

  if (data.have_temp) {
    msg +=
      String(" temperature=") + String(data.temperature, 2) +
      String(",humidity=") + String(data.humidity, 2);
  }
  if (data.have_quality) {
    if (data.have_temp)
      msg += ",";
    else
      msg += " ";
    msg +=
      String("CO2=") + String(data.eCO2) +
      String(",TVOC=") + String(data.TVOC);
  }

  Serial.println(msg);

  if (!g_MQTT_environ_publisher.publish(msg.c_str())) {
    Serial.println("Failed to publish.");
    return false;
  }
  
  return true;
}

}  // anonymous namespace

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only.
  }

  // Inialize I2C Hardware
  Wire.begin();
  
  // Initialize temperature/humidity sensor.
  g_temp_humidity_sensor.begin(RHT03_DATA_PIN);

  // Initialize air quality sensor.
  g_air_quality_sensor.begin();
}

void loop() {
  EnvironmentalData data;
  if (getData(&data)) {
    if (writeData(data)) {
      Serial.println("Successfully logged environmental data.");
    } else {
      Serial.println("ERROR writing environmental data.");
    }
  } else {
    Serial.println("ERROR getting environmental data.");
  }

  delay(kSensorReadingDelay);
}
