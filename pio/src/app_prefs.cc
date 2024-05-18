#include "app_prefs.h"

#include <cctype>
#include <cstdlib>
#include <expected>
#include <string_view>

#include <esp_log.h>
#include <nvs.h>

namespace {
constexpr std::string_view kNamespace = "prefs";
constexpr std::string_view kMqttUriKey = "mqtt-uri";
constexpr std::string_view kMqttUsernameKey = "mqtt-username";
constexpr std::string_view kMqttPasswordKey = "mqtt-password";
constexpr std::string_view kWifiSsidKey = "wifi-ssid";
constexpr std::string_view kWifiPasswordKey = "wifi-password";
constexpr std::string_view kSensorNameKey = "sensor-name";
constexpr std::string_view kSensorLocationKey = "sensor-location";
constexpr std::string_view kSensorTypeKey = "sensor-type";

constexpr char TAG[] = "prefs";

static_assert(kNamespace.size() < NVS_KEY_NAME_MAX_SIZE);
static_assert(kMqttUriKey.size() < NVS_KEY_NAME_MAX_SIZE);
static_assert(kMqttUsernameKey.size() < NVS_KEY_NAME_MAX_SIZE);
static_assert(kMqttPasswordKey.size() < NVS_KEY_NAME_MAX_SIZE);
static_assert(kWifiSsidKey.size() < NVS_KEY_NAME_MAX_SIZE);
static_assert(kWifiPasswordKey.size() < NVS_KEY_NAME_MAX_SIZE);
static_assert(kSensorNameKey.size() < NVS_KEY_NAME_MAX_SIZE);
static_assert(kSensorLocationKey.size() < NVS_KEY_NAME_MAX_SIZE);
static_assert(kSensorTypeKey.size() < NVS_KEY_NAME_MAX_SIZE);

class NvsCloser {
 public:
  NvsCloser(nvs_handle_t nvs) : nvs_(nvs) {}
  ~NvsCloser() { nvs_close(nvs_); }

 private:
  nvs_handle_t nvs_;
};

bool IsWhitespace(char ch) {
  return ch == '\0' || std::isspace(ch);
}

void ChompString(std::string* str) {
  while (!str->empty() && IsWhitespace(str->at(str->size() - 1)))
    str->resize(str->size() - 1);
}

std::expected<std::string, esp_err_t> ReadString(nvs_handle_t nvs,
                                                 std::string_view key) {
  size_t len;
  esp_err_t err = nvs_get_str(nvs, key.data(), nullptr, &len);
  if (err != ESP_OK) {
    return std::unexpected(err);
  }
  char* val = static_cast<char*>(std::malloc(len));
  if (!val) {
    return std::unexpected(ESP_ERR_NO_MEM);
  }
  if ((err = nvs_get_str(nvs, key.data(), val, &len)) != ESP_OK) {
    std::free(val);
    return std::unexpected(err);
  }
  std::string sval(val, len);
  std::free(val);
  ChompString(&sval);
  return sval;
}

}  // namespace

AppPrefs::AppPrefs() = default;

esp_err_t AppPrefs::Load() {
  nvs_handle_t nvs;
  esp_err_t err;

  ESP_LOGI(TAG, "Loading prefs");
  if ((err = nvs_open(kNamespace.data(), NVS_READONLY, &nvs)) != ESP_OK) {
    return err;
  }
  NvsCloser closer(nvs);
  std::expected<std::string, esp_err_t> res;
  if ((res = ReadString(nvs, kMqttUriKey)); res.has_value())
    mqtt_uri_ = std::move(res.value());
  else
    return res.error();
  if ((res = ReadString(nvs, kMqttUsernameKey)); res.has_value())
    mqtt_username_ = std::move(res.value());
  else
    return res.error();
  if ((res = ReadString(nvs, kMqttPasswordKey)); res.has_value())
    mqtt_password_ = std::move(res.value());
  else
    return res.error();
  if ((res = ReadString(nvs, kWifiSsidKey)); res.has_value())
    wifi_ssid_ = std::move(res.value());
  else
    return res.error();
  if ((res = ReadString(nvs, kWifiPasswordKey)); res.has_value())
    wifi_password_ = std::move(res.value());
  else
    return res.error();
  if ((res = ReadString(nvs, kSensorNameKey)); res.has_value())
    sensor_name_ = std::move(res.value());
  else {
    return res.error();
  }
  if ((res = ReadString(nvs, kSensorLocationKey)); res.has_value())
    sensor_location_ = std::move(res.value());
  else
    return res.error();
  if ((res = ReadString(nvs, kSensorTypeKey)); res.has_value())
    if (res.value() == "BME280")
      sensor_type_ = SensorType::BME280;
    else if (res.value() == "BME680")
      sensor_type_ = SensorType::BME680;
    else
      ESP_LOGE(TAG, "Unknown sensor type: \"%s\"", res.value().c_str());
  else
    return res.error();
  return ESP_OK;
}

esp_err_t AppPrefs::Save() {
  nvs_handle_t nvs;
  esp_err_t err;

  if ((err = nvs_open(kNamespace.data(), NVS_READWRITE, &nvs)) != ESP_OK)
    return err;
  NvsCloser closer(nvs);
  if ((err = nvs_set_str(nvs, kMqttUriKey.data(), mqtt_uri_.c_str())) != ESP_OK)
    return err;
  if ((err = nvs_set_str(nvs, kMqttUsernameKey.data(),
                         mqtt_username_.c_str())) != ESP_OK)
    return err;
  if ((err = nvs_set_str(nvs, kMqttPasswordKey.data(),
                         mqtt_password_.c_str())) != ESP_OK)
    return err;
  if ((err = nvs_set_str(nvs, kWifiSsidKey.data(), wifi_ssid_.c_str())) !=
      ESP_OK)
    return err;
  if ((err = nvs_set_str(nvs, kWifiPasswordKey.data(),
                         wifi_password_.c_str())) != ESP_OK)
    return err;
  if ((err = nvs_set_str(nvs, kSensorNameKey.data(), sensor_name_.c_str())) !=
      ESP_OK)
    return err;
  if ((err = nvs_set_str(nvs, kSensorLocationKey.data(),
                         sensor_location_.c_str())) != ESP_OK)
    return err;
  {
    std::string val;
    switch (sensor_type_) {
      case SensorType::BME280:
        val = "BME280";
        break;
      case SensorType::BME680:
        val = "BME680";
        break;
      case SensorType::Unknown:
        // Will skip writing.
        break;
    }
    if (!val.empty()) {
      if ((err = nvs_set_str(nvs, kSensorTypeKey.data(), val.c_str())) !=
          ESP_OK)
        return err;
    }
  }
  return ESP_OK;
}