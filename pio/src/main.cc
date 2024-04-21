#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <i2clib/bus.h>
#include <i2clib/master.h>
#include <i2clib/status.h>

#include "mybme280.h"

constexpr char TAG[] = "App";

namespace {
esp_err_t InitI2C() {
  constexpr i2c::Bus::InitParams params = {
      .i2c_bus = I2C_NUM_0,
      .sda_gpio = 21,
      .scl_gpio = 22,
      .clk_speed = 1'000'000,  // Max BME280 I2C bus speed is 3.4 MHz.
      .sda_pullup_enable = true,
      .scl_pullup_enable = true,
  };
  i2c::Status s = i2c::Bus::Initialize(params);
  return s.ok() ? ESP_OK : ESP_FAIL;
}
}  // namespace
extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting app");
  esp_err_t err = InitI2C();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C init failure: %d", err);
    return;
  }
  ESP_LOGI(TAG, "I2C initialized.");
  i2c::Master i2c_master;
  BME280 bme280(i2c_master);

  if (!bme280.Init()) {
    ESP_LOGE(TAG, "BME280 init failure.");
    return;
  }

  while (true) {
    auto data = bme280.GetData(kBME280All);
    if (data.has_value()) {
      ESP_LOGI(TAG, "Temperature: %.1f C", data->temperature.value());
      ESP_LOGI(TAG, "Humidity: %.1f %%", data->humidity.value());
      ESP_LOGI(TAG, "Pressure: %.1f hPa", data->pressure.value() / 100);
    } else {
      ESP_LOGE(TAG, "Failure getting sensor data: %u.", data.error());
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}