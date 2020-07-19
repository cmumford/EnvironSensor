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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>

#include <limits>
#include <string>

#include <BME280.h>
#include <mgos.h>
#include <mgos_mqtt.h>

namespace {

constexpr double kInvalidValue = std::numeric_limits<double>::quiet_NaN();

/**
 * The data read from the various environmental sensors.
 */
struct EnvironmentalData {
  /**
   * Serialize this instance into a string in InfluxDB format as field values.
   *
   * Invalid values are omitted.
   *
   * https://docs.influxdata.com/influxdb/v1.8/introduction/get-started/#writing-and-exploring-data
   */
  std::string ToInfluxDBString() const;

  double humidity = kInvalidValue;     // The relative humidity in % (0..100).
  double temperature = kInvalidValue;  // The temperature in °F.
  int eCO2 = -1;                       // Estimated carbon dioxide level (ppm).
  int TVOC = -1;                    // Total volatile organic compounds (ppb).
  double pressure = kInvalidValue;  // Air pressure (hPA).
};

BME280* g_BME280 = nullptr;
bool g_got_first_conn_ack = false;

#define UNUSED(expr) \
  do {               \
    (void)(expr);    \
  } while (0)

std::string EnvironmentalData::ToInfluxDBString() const {
  const int kMessageLen = 120;
  char mqtt_message[kMessageLen];
  snprintf(mqtt_message, kMessageLen,
           "temperature=%.1f,humidity=%.1f,pressure=%.1f", temperature,
           humidity, pressure);
  mqtt_message[kMessageLen - 1] = '\0';

  return mqtt_message;
}

/**
 * Convert Celsius to Fahrenheit.
 */
double CtoF(double C) {
  return C * 9.0 / 5.0 + 32.0;
}

/**
 * Read the BME280 sensor values into |data|.
 */
bool ReadBME280(EnvironmentalData* data) {
  if (!g_BME280 || !g_BME280->isBME280()) {
    LOG(LL_ERROR, ("uptime: %.2lf, NO BME280 SENSOR.", mgos_uptime()));
    return false;
  }

  struct mgos_bme280_data bme_data;
  int8_t result = g_BME280->read(bme_data);
  if (result != 0) {
    LOG(LL_ERROR,
        ("uptime: %.2lf Error %d reading BME280.", mgos_uptime(), result));
    return false;
  }

  data->temperature = CtoF(bme_data.temp);
  data->humidity = bme_data.humid;
  data->pressure = bme_data.press / 100.0;

  return true;
}

/**
 * Publish sensor data to the MQTT broker.
 */
bool PublishSensorData(const EnvironmentalData& data) {
  mgos_wifi_status wifi_status = mgos_wifi_get_status();
  if (wifi_status != MGOS_WIFI_IP_ACQUIRED) {
    LOG(LL_WARN,
        ("uptime: %.2lf, WiFi not connected: %d.", mgos_uptime(), wifi_status));
    return false;
  }

  std::string mqtt_message = "environment,location=";
  mqtt_message += mgos_sys_config_get_app_sensor_location();
  mqtt_message += ' ';
  mqtt_message += data.ToInfluxDBString();

  uint16_t packet_id =
      mgos_mqtt_pub(mgos_sys_config_get_app_mqtt_topic(), mqtt_message.c_str(),
                    mqtt_message.size(), /*qos=*/1, /*retain=*/false);
  if (!packet_id) {
    LOG(LL_ERROR,
        ("uptime: %.2lf, Error publishing to MQTT broker.", mgos_uptime()));
    return false;
  }

  LOG(LL_INFO, ("Successfully published %s", mqtt_message.c_str()));
  return true;
}

/**
 * Read, and publish via MQTT, all sensor values.
 */
void ReadSensorsCb(void* arg) {
  UNUSED(arg);

  EnvironmentalData data;
  if (!ReadBME280(&data))
    return;

  PublishSensorData(data);
}

/**
 * A global handler for MQTT messages.
 */
void MQTTGlobalHandler(struct mg_connection* nc,
                       int ev,
                       void* ev_data MG_UD_ARG(void* user_data)) {
  UNUSED(nc);
  UNUSED(ev_data);
  UNUSED(user_data);

  if (g_got_first_conn_ack)
    return;

  if (ev == MG_EV_MQTT_CONNACK) {
    LOG(LL_INFO,
        ("uptime: %.2lf, Just connected to MQTT broker.", mgos_uptime()));
    g_got_first_conn_ack = true;
    ReadSensorsCb(nullptr);
  }
}

}  // namespace

enum mgos_app_init_result mgos_app_init(void) {
  g_BME280 = new BME280(mgos_sys_config_get_app_bme280_addr());

  mgos_mqtt_add_global_handler(MQTTGlobalHandler, nullptr);

  mgos_set_timer(mgos_sys_config_get_app_read_interval_ms(), MGOS_TIMER_REPEAT,
                 ReadSensorsCb, nullptr);

  return MGOS_APP_INIT_SUCCESS;
}
