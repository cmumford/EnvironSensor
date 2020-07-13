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
#include <Wire.h>
#include <Adafruit_MQTT_FONA.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ESP8266_ESP12) || defined(ARDUINO_ESP8266_NODEMCU)
  #include <ESP8266WiFi.h>
#else
  #include <WiFi.h>
#endif

#include "arduino_secrets.h"

namespace {

// Delay between readings in milliseconds.
const unsigned long kSensorReadingDelay = 60 * 1000;

#define SENSOR_RHT03   0  // A Temp/humidity sensor.
#define SENSOR_BME280  1  // A Temp/humidity/pressure sensor.
#define SENSOR_CCS811  0  // An air quality sensor (CO2/VOC).


#if SENSOR_RHT03

// RHT03 data pin.
const int RHT03_DATA_PIN = 23;

#endif //  SENSOR_RHT03

#if SENSOR_BME280

const float kSeaLevelPressureHPA = 1013.25f;

// Can be changed with a solder jumper.
const int BME280_ADDR = 0x76;

#endif  // SENSOR_BME280


#if SENSOR_CCS811

// The I2C bus address for the air quality sensor.
// Default is 0x5B. Desolder the "ADDR" jumper to switch sensor to 0x5A.
const int CCS811_ADDR = 0x5B;

#endif  // SENSOR_CCS811

const String kSensorLocation = "office";
const char* kSSID = SECRET_SSID;
const char* kPASS = SECRET_PASS;
const char kMQTTBroker[] = "10.0.9.116";
const int  kMQTTPort = 1883;
const char kMQTTTopic[] = "sensors/inside";

/**
 * The data read from the various environmental sensors.
 */
struct EnvironmentalData {
  bool have_temp = false;    // True if the temp/humidity values are valid.
  bool have_quality = false; // True if the air quality values are valid.
  bool have_pressures = false; // True if air pressures are valud.
  float humidity = 0.0f;     // The air relative humidity in % (0..100).
  float temperature = 0.0f;  // The air temperature in °F.
  int eCO2 = 0;              // Estimated carbon dioxide level (ppm).
  int TVOC = 0;              // Total volatile organic compounds (ppb).
  float altitude = 0.0f;     // Altitude above sea level (ft.).
  float pressure = 0.0f;     // Air pressure (hPA).
};

#if SENSOR_RHT03

// The global object to interract with the temperature/humidity sensor.
RHT03 g_temp_humidity_sensor;

#endif  // SENSOR_RHT03

#if SENSOR_BME280

Adafruit_BME280 g_BME280;

#endif  // SENSOR_BME280

#if SENSOR_CCS811

// The global object to interract with the air quality sensor.
// Sensor is not accurate until after 20 minutes of runtime.
CCS811 g_air_quality_sensor(CCS811_ADDR);

#endif  // SENSOR_CCS811

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

// The number of times the WiFi network is not connected when
// MQTT logging is attempted.
int g_num_wifi_errors = 0;

// The number of MQTT publish errors since connecting to WiFi.
int g_num_publish_errors = 0;


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
 * Return a debug string (or empty) providing additional debug info.
 */
String GetDebugInfo() {
  return "[reading:" + String(g_num_readings) +
         ", wifi_err:" + String(g_num_wifi_errors) +
         ", pub_err:" + String(g_num_publish_errors) +
         ']';
}

// Convert Fahrenheit to Celsius.
float FtoC(float F) {
  return (F-32.0f) / 1.8f;
}

// Convert Celsius to Fahrenheit.
float CtoF(float C) {
  return C * 9.0f / 5.0f + 32.0f;
}

// Convert meters to feet.
float MtoF(float M) {
  return M * 3.28084f;
}

/**
 * Retrieve the temperature and humidity from the sensor and write them to
 * |data|.
 *
 * Return true if successful, false if not.
 */
bool getTempHumidity(EnvironmentalData* data) {
  data->have_temp = false;

#if SENSOR_RHT03
  if (g_temp_humidity_sensor.update() != 1) {
    // If the update failed, try delaying for RHT_READ_INTERVAL_MS ms before
    // trying again.
    delay(RHT_READ_INTERVAL_MS);
    return false;
  }
  data->temperature = g_temp_humidity_sensor.tempF();
  data->humidity = g_temp_humidity_sensor.humidity();
  data->have_temp = true;
#elif SENSOR_BME280
  data->temperature =  CtoF(g_BME280.readTemperature());
  data->humidity = g_BME280.readHumidity();
  data->have_temp = true;
#endif

  return data->have_temp;
}

/**
 * Retrieve the air quality from the sensor and write them to |data|.
 *
 * Return true if successful, false if not.
 */
bool getAirQuality(EnvironmentalData* data) {
  data->have_quality = false;

#if SENSOR_CCS811
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
#endif  // #if SENSOR_CCS811

  return data->have_quality;
}

/**
 * Read atmospheric pressures.
 */
bool getPressures(EnvironmentalData* data) {
  data->have_pressures = false;

#if SENSOR_BME280
  data->altitude =  MtoF(g_BME280.readAltitude(kSeaLevelPressureHPA));
  data->pressure = g_BME280.readPressure() / 100.0f;
  data->have_pressures = true;
#endif

  return data->have_pressures;
}

/**
 * Get the environmental data from all sensors and write to |data|.
 *
 * Return true if successful, false if not.
 */
bool getData(EnvironmentalData* data) {
  g_num_readings++;
  int count = 0;

  if (getTempHumidity(data))
    count++;
  if (getAirQuality(data))
    count++;
  if (getPressures(data))
    count++;

  return count != 0;
}

/**
 * Returns a MQTT message payload in influxdb format.
 */
String createMQTTMessage(const EnvironmentalData& data) {
  // Format the message payload in InfluxDB format.
  String mqtt_message = String("environment,location=") + kSensorLocation;

  bool have_value = false;
  if (data.have_temp) {
    mqtt_message +=
      String(" temperature=") + String(data.temperature, 2) +
      String(",humidity=") + String(data.humidity, 1);
    have_value = true;
  }
  if (data.have_quality) {
    if (have_value)
      mqtt_message += ",";
    else
      mqtt_message += " ";
    mqtt_message +=
      String("CO2=") + String(data.eCO2) +
      String(",TVOC=") + String(data.TVOC);
    have_value = true;
  }

  if (data.have_pressures) {
    if (have_value)
      mqtt_message += ",";
    else
      mqtt_message += " ";
    mqtt_message +=
      String("altitude=") + String(data.altitude) +
      String(",pressure=") + String(data.pressure);
    have_value = true;
  }

  return mqtt_message;
}

/**
 * Log the enrivonmental data to the MQTT server.
 *
 * Return true if successful, false if not.
 */
bool writeData(const EnvironmentalData& data) {
  const String mqtt_message = createMQTTMessage(data);

  int wifi_status = WiFi.status();
  if (wifi_status != WL_CONNECTED) {
    Serial.print("Not connected to WiFi");
    Serial.print(GetDebugInfo());
    Serial.print(": ");
    Serial.print(GetWiFiStatusString(wifi_status));
    Serial.println(": " + mqtt_message + '.');
    g_num_wifi_errors++;
    return false;
  }

  if (!g_time_is_set) {
    getNetworkTime();
    if (!g_time_is_set)
      Serial.println("Online, but error setting time.");
  }

  if (!g_MQTT_client.connected()) {
    int mqtt_client_status = g_MQTT_client.connect();
    if (mqtt_client_status != 0) {
      Serial.print("MQTT client connect error" + GetDebugInfo() + ": " + mqtt_message);
      Serial.println(g_MQTT_client.connectErrorString(mqtt_client_status));
      return false;
    }
  }

  bool published = g_MQTT_environ_publisher.publish(mqtt_message.c_str());
  if (!published) {
    g_num_publish_errors++;
    Serial.println("Failed to publish" + GetDebugInfo() + ": " + mqtt_message + '.');
    // There is currently a bug (in either Mosquitto or on our  end) where
    // the MQTT server (Mosquitto) retransmits the ACK packet. This means
    // our end receives the same ACK packet twice and the packet ID's are
    // forever out of sync. Disconnecting here will force a reconnect and
    // get these ID's back in sync (until the next time).
    g_MQTT_client.disconnect();
    return false;
  }

  Serial.print("MQTT client logged");
  Serial.print(GetDebugInfo());
  Serial.print(": ");
  Serial.println(mqtt_message);

  return true;
}

}  // anonymous namespace

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only.
  }

  for (int i = 0; i < 20; i++) {
    Serial.println();
    delay(10);
  }
  Serial.print("Starting environmental sensor ");
  Serial.print(kMQTTTopic);
  Serial.print(", location: ");
  Serial.print(kSensorLocation);
  Serial.println(".");

  // Inialize I2C Hardware
  Wire.begin();

#if SENSOR_RHT03
  // Initialize temperature/humidity sensor.
  g_temp_humidity_sensor.begin(RHT03_DATA_PIN);
#endif

#if SENSOR_CCS811
  // Initialize air quality sensor.
  g_air_quality_sensor.begin();
#endif  // SENSOR_CCS811

#if SENSOR_BME280
  if (!g_BME280.begin(BME280_ADDR)) {
    Serial.println("Could not find a BME280 sensor.");
  }
#endif  // SENSOR_BME280

#if defined(ARDUINO_ESP8266_ESP12) || defined(ARDUINO_ESP8266_NODEMCU)
  WiFi.mode(WIFI_STA);
#endif

  // Initiate the connection to WiFi.
  WiFi.begin(const_cast<char*>(kSSID), const_cast<char*>(kPASS));

  Serial.println("Setup complete");
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
