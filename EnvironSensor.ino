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
Adafruit_MQTT_Client g_MQTT_client(&g_WiFi_client, kMQTTBroker,
                                   kMQTTPort, /*username=*/"", /*key=*/"");

// Object that publishes a specific topic to the MQTT broker.
Adafruit_MQTT_Publish g_MQTT_environ_publisher(&g_MQTT_client, kMQTTTopic,
                                               /*QOS=*/1);

// The number of sensor readings attempted (but maybe not logged).
int g_num_readings = 0;

// The number of times we have attempted to connected to the WiFi network.
int g_num_net_connects = 0;

// The number of MQTT publish errors since connecting to WiFi.
int g_num_publish_errors = 0;

/**
 * Return a debug string (or empty) providing additional debug info.
 */
String GetDebugInfo() {
  return "[reading:" + String(g_num_readings) +
         ", connection:" + String(g_num_net_connects) +
         ", pub_err:" + String(g_num_publish_errors) +
         ']';
}

// Convert Fahrenheit to Celsius.
template <class T>
T FtoC(T F) {
  return (F-32.0f) / 1.8f;
}

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
                                             FtoC(data->temperature));
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
  return WiFi.status() == WL_CONNECTED;
}

/**
 * Returns a WiFi status string for debugging purposes.
 */
const __FlashStringHelper* GetWiFiStatusString(int status) {
  switch (status) {
    case WL_CONNECTED:
      return F("WL_CONNECTED");       // assigned when connected to a WiFi network
    case WL_NO_SHIELD:
      return F("WL_NO_SHIELD");       // assigned when no WiFi shield is present;
    case WL_IDLE_STATUS:
      return F("WL_IDLE_STATUS");     // it is a temporary status assigned when WiFi.begin() is
                                      // called and remains active until the number of attempts
                                      // expires (resulting in WL_CONNECT_FAILED) or a connection
                                      // is established (resulting in WL_CONNECTED);
    case WL_NO_SSID_AVAIL:
      return F("WL_NO_SSID_AVAIL");   // assigned when no SSID are available;
    case WL_SCAN_COMPLETED:
      return F("WL_SCAN_COMPLETED");  // assigned when the scan networks is completed;
    case WL_CONNECT_FAILED:
      return F("WL_CONNECT_FAILED");  // assigned when the connection fails for all the attempts;
    case WL_CONNECTION_LOST:
      return F("WL_CONNECTION_LOST"); // assigned when the connection is lost;
    case WL_DISCONNECTED:
      return F("WL_DISCONNECTED");    // assigned when disconnected from a network;
  }
  return F("Unknown");
}

/**
 * Connect to the WiFi network.
 *
 * The radio takes a little while to connect, so the first call will
 * likely fail.
 *
 * Return the current connection status (WL_CONNECTED means connected).
 */
int ConnectToWiFi() {
  const int kMaxAttempts = 3;

  g_num_net_connects++;

  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(kSSID);
  int status = WiFi.begin(kSSID, kPASS);
  for (int attempt = 0; status != WL_CONNECTED && attempt < kMaxAttempts; attempt++) {
    // wait 10 seconds for the connection.
    delay(10 * 1000);
    status = WiFi.status();
  }

  return status;
}

/**
 * Get the environmental data from all sensors and write to |data|.
 * 
 * Return true if successful, false if not.
 */
bool getData(EnvironmentalData* data) {
  g_num_readings++;
  return getTempHumidity(data) || getAirQuality(data);
}

/**
 * Returns a MQTT message payload in influxdb format.
 */
String formatMQTTMessage(const EnvironmentalData& data) {
  // Format the message payload in InfluxDB format.
  String mqtt_message = String("environment,location=") + kSensorLocation;

  if (data.have_temp) {
    mqtt_message +=
      String(" temperature=") + String(data.temperature, 2) +
      String(",humidity=") + String(data.humidity, 1);
  }
  if (data.have_quality) {
    if (data.have_temp)
      mqtt_message += ",";
    else
      mqtt_message += " ";
    mqtt_message +=
      String("CO2=") + String(data.eCO2) +
      String(",TVOC=") + String(data.TVOC);
  }

  return mqtt_message;
}

/**
 * Log the enrivonmental data to the MQTT server.
 * 
 * Return true if successful, false if not.
 */
bool writeData(const EnvironmentalData& data) {
  if (!WiFiConnected()) {
    g_num_publish_errors = 0;
    if (g_MQTT_client.connected() && !g_MQTT_client.disconnect()) {
      // Disconnecting involves sending a disconnect packet, which can't be done,
      // but it also resets the client to a disconnected state.
      Serial.println("Error disconnecting from MQTT server" + GetDebugInfo() + '.');
    }

    int wifi_status = ConnectToWiFi();
    if (wifi_status != WL_CONNECTED) {
      Serial.println("Unable to connect to WiFi network" + GetDebugInfo() +
                     ": " + GetWiFiStatusString(wifi_status) + '.');
      return false;
    }
    Serial.println("WiFi is connected.");
  }

  if (!g_MQTT_client.connected()) {
    int mqtt_client_status = g_MQTT_client.connect();
    if (mqtt_client_status != 0) {
      Serial.print("MQTT client connect error" + GetDebugInfo() + ": ");
      Serial.println(g_MQTT_client.connectErrorString(mqtt_client_status));
      return false;
    }
  }

  const String mqtt_message = formatMQTTMessage(data);

  if (!g_MQTT_environ_publisher.publish(mqtt_message.c_str())) {
    g_num_publish_errors++;
    Serial.println("Failed to publish" + GetDebugInfo() + '.');
    return false;
  }

  Serial.print("MQTT client logged" + GetDebugInfo() + ": ");
  Serial.println(mqtt_message);

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
      //Serial.println("Successfully logged environmental data.");
    } else {
      //Serial.println("ERROR writing environmental data.");
    }
  } else {
    Serial.println("ERROR getting environmental data.");
  }

  delay(kSensorReadingDelay);
}
