#include "app.h"

#include <cstring>
#include <memory>
#include <string_view>
#include <utility>

#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <i2clib/bus.h>

#include <i2clib/status.h>
#include <nvs_flash.h>

namespace {
constexpr char TAG[] = "App";
constexpr uint16_t kMaxReconnectCount = 5;
}  // namespace

// static
void App::WifiEventHandler(void* event_handler_arg,
                           esp_event_base_t event_base,
                           int32_t event_id,
                           void* event_data) {
  App* app = static_cast<App*>(event_handler_arg);
  switch (event_id) {
    case WIFI_EVENT_STA_START:
      ESP_LOGD(TAG, "WiFi connecting...");
      break;
    case WIFI_EVENT_STA_CONNECTED:
      ESP_LOGI(TAG, "WiFi connected...");
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_LOGI(TAG, "WiFi disconnected");
      if (app->entering_sleep_)  // WiFi disabled when entering sleep.
        return;
      if (app->wifi_disconnect_count_ < kMaxReconnectCount) {
        esp_wifi_connect();
        app->wifi_disconnect_count_++;
        ESP_LOGI(TAG, "Retrying WiFi connection...");
      } else {
        // Too many connection attempts. Go to sleep and hope WiFi
        // connection works next time.
        app->EnterDeepSleep();
      }
      break;
    case IP_EVENT_STA_GOT_IP:
      ESP_LOGI(TAG, "Wifi got IP.");
      if (!app->logger_.connected()) {
        app->StartLogger();
      }
      break;
    default:
      break;
  }
}

App::App() : i2c_master_() {}

esp_err_t App::ConnectToWifi() {
  esp_err_t err;

  err = esp_netif_init();
  if (err != ESP_OK)
    return err;
  err = esp_event_loop_create_default();
  if (err != ESP_OK)
    return err;
  /*auto netif =*/esp_netif_create_default_wifi_sta();
  wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&wifi_initiation);
  if (err != ESP_OK)
    return err;
  err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                   WifiEventHandler, this);
  if (err != ESP_OK)
    return err;
  err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                   WifiEventHandler, this);
  if (err != ESP_OK)
    return err;
  wifi_config_t wifi_configuration;
  std::memset(&wifi_configuration, 0, sizeof(wifi_configuration));
  std::strncpy(reinterpret_cast<char*>(wifi_configuration.sta.ssid),
               prefs_.wifi_ssid().c_str(), sizeof(wifi_configuration.sta.ssid));
  std::strncpy(reinterpret_cast<char*>(wifi_configuration.sta.password),
               prefs_.wifi_password().c_str(),
               sizeof(wifi_configuration.sta.password));
  ESP_LOGI(TAG, "Calling esp_wifi_set_config.");
  err = esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration);
#if 0
  if (err != ESP_OK)
    return err;
#endif
  ESP_LOGI(TAG, "Calling esp_wifi_start.");
  err = esp_wifi_start();
  if (err != ESP_OK)
    return err;
  ESP_LOGI(TAG, "Calling esp_wifi_set_mode.");
  err = esp_wifi_set_mode(WIFI_MODE_STA);
  if (err != ESP_OK)
    return err;
  ESP_LOGI(TAG, "Calling esp_wifi_connect.");
  err = esp_wifi_connect();
  if (err != ESP_OK)
    return err;

  ESP_LOGI(TAG, "WiFi started");
  return ESP_OK;
}

esp_err_t App::InitI2C(uint32_t bus_speed) {
  const i2c::Bus::InitParams params = {
      .i2c_bus = I2C_NUM_0,
      .sda_gpio = 21,
      .scl_gpio = 22,
      .clk_speed = bus_speed,
      .sda_pullup_enable = true,
      .scl_pullup_enable = true,
  };
  i2c::Status s = i2c::Bus::Initialize(params);
  return s.ok() ? ESP_OK : ESP_FAIL;
}

void App::EnterDeepSleep() {
  esp_err_t err;
  ESP_LOGI(TAG, "Sleeping for %u seconds", prefs_.sleep_duration_secs());
  entering_sleep_ = true;

  // Sleep the sensor. If it is kept on 100% of the time then that could warm
  // the sensor and affect sensor readings. The BME280 handling mounting and
  // soldering instructions says:
  //
  //   "The sensor should be not more than 10% in the active state to avoid self
  //   heating".
  err = sensor_->EnterSleep();
  if (err == ESP_OK)
    ESP_LOGI(TAG, "Sensor is asleep");
  else
    ESP_LOGE(TAG, "Error 0x%x putting sensor to sleep", err);

  if (err = esp_wifi_stop(); err != ESP_OK)
    ESP_LOGE(TAG, "Error 0x%x stopping WiFi", err);

  uint64_t sleep_us =
      static_cast<uint64_t>(prefs_.sleep_duration_secs()) * 1'000'000;
  if (err = esp_sleep_enable_timer_wakeup(sleep_us); err != ESP_OK)
    ESP_LOGE(TAG, "Error enabling sleep timer wakeup: 0x%x", err);

  esp_deep_sleep_start();
}

void App::ConnectedCallback(bool connected) {
  ESP_LOGI(TAG, "Logger connected callback, connected:%c",
           connected ? 'Y' : 'N');
  if (connected) {
    esp_err_t err = LogSensor();
    if (err != ESP_OK)
      EnterDeepSleep();
  } else {
    EnterDeepSleep();
  }
}

void App::PublishCallback(int msg_id) {
  ESP_LOGI(TAG, "Logger publish callback, msg id: %d", msg_id);
  EnterDeepSleep();
}

void App::ErrorCallback() {
  ESP_LOGE(TAG, "Logger MQTT error");
  EnterDeepSleep();
}

esp_err_t App::StartLogger() {
  esp_err_t err = logger_.StartClient(
      prefs_, std::bind(&App::ConnectedCallback, this, std::placeholders::_1),
      std::bind(&App::PublishCallback, this, std::placeholders::_1),
      std::bind(&App::ErrorCallback, this));
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Successfully started MQTT logger");
  } else {
    ESP_LOGE(TAG, "Failure starting MQTT client: 0x%x", err);
  }
  return err;
}

uint32_t App::GetI2CBusSpeed() const {
  switch (prefs_.sensor_type()) {
    case SensorType::BME280:
      return BME280::MaxI2CClockSpeed();
      break;
    case SensorType::BME680:
      return BME68X::MaxI2CClockSpeed();
      break;
    case SensorType::Unknown:
      std::unreachable();
      break;
  }
  return 100'000;  // Standard speed
}

esp_err_t App::Init() {
  esp_err_t err;

  nvs_flash_init();

  auto [e, prefs_migrated] = prefs_.Load();
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "Error loading prefs: 0x%x", e);
    return e;
  }
  if (prefs_migrated) {
    ESP_LOGI(TAG, "Prefs were migrated, saving");
    err = prefs_.Save();
    if (err = prefs_.Save(); err != ESP_OK)
      ESP_LOGE(TAG, "Error saving prefs: 0x%x", err);
  }
  ESP_LOGI(TAG, "Initializing device %s/%s", prefs_.sensor_name().c_str(),
           prefs_.sensor_location().c_str());
  err = InitI2C(GetI2CBusSpeed());
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C init failure: 0x%x", err);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "I2C initialized.");
  switch (prefs_.sensor_type()) {
    case SensorType::BME280:
      sensor_ = std::make_unique<BME280>(i2c_master_);
      ESP_LOGI(TAG, "Sensor is a BME280");
      break;
    case SensorType::BME680:
      sensor_ = std::make_unique<BME68X>(i2c_master_);
      ESP_LOGI(TAG, "Sensor is a BME680");
      break;
    case SensorType::Unknown:
      std::unreachable();
      break;
  }
  if (err = sensor_->Init(); err != ESP_OK) {
    ESP_LOGE(TAG, "Sensor init failure: 0x%x.", err);
    return err;
  }

  err = ConnectToWifi();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failure starting WIFI: 0x%x", err);
    return err;
  }
  initialized_ = true;
  return ESP_OK;
}

esp_err_t App::LogSensor() {
  auto data = sensor_->ReadData(kBME280All);
  if (data.has_value()) {
    return logger_.LogSensorData(prefs_, data.value());
  }
  ESP_LOGE(TAG, "Failure getting sensor data: %d.", data.error());
  return ESP_FAIL;
}

esp_err_t App::Run() {
  if (!initialized_)
    return ESP_FAIL;
  while (true) {
    vTaskDelay(1'000 / portTICK_PERIOD_MS);
  }
}
