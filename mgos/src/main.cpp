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

#include <math.h>

#include <limits>
#include <sstream>
#include <string>

#include <BME280.h>
#include <mgos.h>
#include <mgos_ccs811.h>
#include <mgos_dht.h>
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
  double eCO2 = kInvalidValue;         // Estimated carbon dioxide level (ppm).
  double TVOC = kInvalidValue;      // Total volatile organic compounds (ppb).
  double pressure = kInvalidValue;  // Air pressure (hPA).
};

BME280* g_BME280 = nullptr;
mgos_dht* g_DHT22 = nullptr;
mgos_ccs811* g_CCS811 = nullptr;
bool g_got_first_conn_ack = false;

#define UNUSED(expr) \
  do {               \
    (void)(expr);    \
  } while (0)

std::string EnvironmentalData::ToInfluxDBString() const {
  std::stringstream ss;
  bool has_field = false;

  if (!isnan(temperature)) {
    if (has_field)
      ss << ',';
    ss << "temperature=" << temperature;
    has_field = true;
  }
  if (!isnan(humidity)) {
    if (has_field)
      ss << ',';
    ss << "humidity=" << humidity;
    has_field = true;
  }
  if (!isnan(pressure)) {
    if (has_field)
      ss << ',';
    ss << "pressure=" << pressure;
    has_field = true;
  }
  if (!isnan(eCO2)) {
    if (has_field)
      ss << ',';
    ss << "CO2=" << eCO2;
    has_field = true;
  }
  if (!isnan(TVOC)) {
    if (has_field)
      ss << ',';
    ss << "TVOC=" << TVOC;
    has_field = true;
  }

  return ss.str();
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
    LOG(LL_ERROR, ("No BME280 SENSOR."));
    return false;
  }

  mgos_bme280_data bme_data;
  int8_t result = g_BME280->read(bme_data);
  if (result != 0) {
    LOG(LL_ERROR, ("Error %d reading BME280.", result));
    return false;
  }

  data->temperature = CtoF(bme_data.temp);
  data->humidity = bme_data.humid;
  data->pressure = bme_data.press / 100.0;

  return true;
}

bool ReadDHT22(EnvironmentalData* data) {
  if (!g_DHT22) {
    LOG(LL_ERROR, ("No DHT22 SENSOR."));
    return false;
  }

  double temp = mgos_dht_get_temp(g_DHT22);
  if (isnan(temp)) {
    LOG(LL_ERROR, ("Error reading temp from DHT22 sensor."));
    return false;
  }
  data->temperature = CtoF(temp);
  data->humidity = mgos_dht_get_humidity(g_DHT22);
  if (isnan(data->humidity)) {
    LOG(LL_ERROR, ("Error reading humidity from DHT22 sensor."));
    return false;
  }

  return true;
}

bool ReadCCS811(EnvironmentalData* data) {
  if (!g_CCS811) {
    LOG(LL_ERROR, ("No CCS811 sensor."));
    return false;
  }

  data->eCO2 = mgos_ccs811_get_eco2(g_CCS811);
  if (isnan(data->eCO2)) {
    LOG(LL_ERROR, ("Error reading eCO2 from CCS811 sensor."));
    return false;
  }
  data->TVOC = mgos_ccs811_get_tvoc(g_CCS811);
  if (isnan(data->TVOC)) {
    LOG(LL_ERROR, ("Error reading TVOC from CCS811 sensor."));
    return false;
  }

  return true;
}

/**
 * Publish sensor data to the MQTT broker.
 */
bool PublishSensorData(const EnvironmentalData& data) {
  mgos_wifi_status wifi_status = mgos_wifi_get_status();
  if (wifi_status != MGOS_WIFI_IP_ACQUIRED) {
    LOG(LL_WARN, ("WiFi not connected: %d.", wifi_status));
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
    LOG(LL_ERROR, ("Error publishing to MQTT broker."));
    return false;
  }

  LOG(LL_INFO, ("Published %s", mqtt_message.c_str()));
  return true;
}

/**
 * Read, and publish via MQTT, all sensor values.
 */
void ReadSensorsCb(void* arg) {
  UNUSED(arg);

  EnvironmentalData data;
  bool have_data = false;

  if (g_BME280)
    have_data = ReadBME280(&data);

  if (!have_data && g_DHT22)
    have_data = ReadDHT22(&data);

  if (g_CCS811) {
    bool success = ReadCCS811(&data);
    if (!have_data)
      have_data = success;
  }

  if (!have_data) {
    LOG(LL_WARN, ("No sensor data to publish."));
    return;
  }

  PublishSensorData(data);
}

/**
 * A global handler for MQTT messages.
 */
void MQTTGlobalHandler(mg_connection* nc,
                       int ev,
                       void* ev_data MG_UD_ARG(void* user_data)) {
  UNUSED(nc);
  UNUSED(ev_data);
  UNUSED(user_data);

  if (g_got_first_conn_ack)
    return;

  if (ev == MG_EV_MQTT_CONNACK) {
    LOG(LL_INFO, ("Just connected to MQTT broker."));
    g_got_first_conn_ack = true;
    ReadSensorsCb(nullptr);
  }
}

void InitSensors() {
  if (mgos_sys_config_get_app_bme280_addr()) {
    LOG(LL_INFO, ("Connecting to BME280 at 0x%x.",
                  mgos_sys_config_get_app_bme280_addr()));
    g_BME280 = new BME280(mgos_sys_config_get_app_bme280_addr());
  }

  if (mgos_sys_config_get_app_dht_pin()) {
    LOG(LL_INFO, ("Connecting to DHT-22 on GPIO %d.",
                  mgos_sys_config_get_app_dht_pin()));
    g_DHT22 = mgos_dht_create(mgos_sys_config_get_app_dht_pin(), DHT22);
  }

  if (mgos_sys_config_get_app_ccs811_addr()) {
    LOG(LL_INFO, ("Connecting to CCS811 at 0x%x.",
                  mgos_sys_config_get_app_ccs811_addr()));
    mgos_i2c* i2c = mgos_i2c_get_global();
    if (i2c)
      g_CCS811 = mgos_ccs811_create(i2c, mgos_sys_config_get_app_ccs811_addr());
    else
      LOG(LL_INFO, ("No I2C addr"));
  }
}

}  // namespace

enum mgos_app_init_result mgos_app_init(void) {
  InitSensors();

  mgos_mqtt_add_global_handler(MQTTGlobalHandler, nullptr);

  mgos_set_timer(mgos_sys_config_get_app_read_interval_ms(), MGOS_TIMER_REPEAT,
                 ReadSensorsCb, nullptr);

  return MGOS_APP_INIT_SUCCESS;
}
