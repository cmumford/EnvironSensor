#include "app.h"

#include <cstring>
#include <string_view>

#include <esp_log.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <i2clib/bus.h>

#include <i2clib/status.h>
#include <nvs_flash.h>

namespace {
constexpr char TAG[] = "App";
}

int s_WifiConnectCount = 0;

// static
void App::WifiEventHandler(void* event_handler_arg,
                           esp_event_base_t event_base,
                           int32_t event_id,
                           void* event_data) {
  App* app = static_cast<App*>(event_handler_arg);
  switch (event_id) {
    case WIFI_EVENT_STA_START:
      ESP_LOGI(TAG, "WIFI CONNECTING...");
      break;
    case WIFI_EVENT_STA_CONNECTED:
      ESP_LOGI(TAG, "WIFI CONNECTED...");
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_LOGI(TAG, "WIFI Connection lost");
      if (s_WifiConnectCount < 5) {
        esp_wifi_connect();
        s_WifiConnectCount++;
        ESP_LOGI(TAG, "Retrying to Connect...");
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

App::App()
    : i2c_master_(),
#if defined(DEVICE_BME280)
      bme280_(i2c_master_)
#elif defined(DEVICE_BME68X)
      bme68x_(i2c_master_)
#else
#error "Unknown device";
#endif
{
}

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

esp_err_t App::InitI2C() {
  constexpr i2c::Bus::InitParams params = {
      .i2c_bus = I2C_NUM_0,
      .sda_gpio = 21,
      .scl_gpio = 22,
#if defined(DEVICE_BME68X)
      .clk_speed = 400'000,  // BME68X supports standard/fast (100Kbps/400Kbps).
#elif defined(DEVICE_BME280)
      .clk_speed = 1'000'000,  // Max BME280 I2C bus speed is 3.4 MHz.
#else
#error Unknown device
#endif
      .sda_pullup_enable = true,
      .scl_pullup_enable = true,
  };
  i2c::Status s = i2c::Bus::Initialize(params);
  return s.ok() ? ESP_OK : ESP_FAIL;
}

esp_err_t App::StartLogger() {
  esp_err_t err = logger_.StartClient(prefs_);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Successfully started MQTT logger");
  } else {
    ESP_LOGE(TAG, "Failure starting MQTT client: 0x%x", err);
  }
  return err;
}

esp_err_t App::Init() {
  esp_err_t err;

  nvs_flash_init();

  err = prefs_.Load();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error loading prefs: 0x%x", err);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Initializing device %s/%s", prefs_.sensor_name().c_str(),
           prefs_.sensor_location().c_str());
  err = InitI2C();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C init failure: 0x%x", err);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "I2C initialized.");
#if defined(DEVICE_BME280)
  if (!bme280_.Init()) {
    ESP_LOGE(TAG, "BME280 init failure.");
    return ESP_FAIL;
  }
#elif defined(DEVICE_BME68X)
  if (!bme68x_.Init()) {
    ESP_LOGE(TAG, "BME68X init failure.");
    return ESP_FAIL;
  }
#else
#error "Unknown device";
#endif

  err = ConnectToWifi();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failure starting WIFI: 0x%x", err);
    return err;
  }
  initialized_ = true;
  return ESP_OK;
}

#if defined(DEVICE_BME280)
esp_err_t App::LogBME280() {
  auto data = bme280_.ReadData(kBME280All);
  if (data.has_value()) {
    if (data->temperature.has_value())
      ESP_LOGI(TAG, "Temperature: %.1f C", data->temperature.value());
    if (data->humidity.has_value())
      ESP_LOGI(TAG, "Humidity: %.1f %%", data->humidity.value());
    if (data->pressure.has_value())
      ESP_LOGI(TAG, "Pressure: %.1f hPa", data->pressure.value() / 100);
    esp_err_t err = logger_.LogSensorData(prefs_, data.value());
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failure logging sensor data: 0x%x", err);
    }
    return err;
  }
  ESP_LOGE(TAG, "Failure getting sensor data: %d.", data.error());
  return ESP_FAIL;
}
#endif  // defined(DEVICE_BME280)

#if defined(DEVICE_BME68X)
esp_err_t App::LogBME68X() {
  auto data = bme68x_.ReadData();
  if (data.has_value()) {
    return logger_.LogSensorData(prefs_, data.value());
  }
  ESP_LOGE(TAG, "Failure getting sensor data: %d.", data.error());
  return ESP_FAIL;
}
#endif  // defined(DEVICE_BME68X)

esp_err_t App::Run() {
  esp_err_t err;

  if (!initialized_)
    return ESP_FAIL;
  while (true) {
    if (logger_.connected()) {
#if defined(DEVICE_BME280)
      err = LogBME280();
#elif defined(DEVICE_BME68X)
      err = LogBME68X();
#else
#error "Unknown device";
#endif
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failure logging sensor data: 0x%x", err);
      }
    }
    vTaskDelay(15'000 / portTICK_PERIOD_MS);
  }
}
