#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c.h"
#include "mybme280.h"

constexpr char TAG[] = "App";

extern "C" void app_main(void) {
  I2CMaster i2c_master;
  BME280 bme280(i2c_master);

  ESP_LOGI(TAG, "Starting app");

  esp_err_t err = i2c_master.Init();

  if (!bme280.Init()) {
    ESP_LOGE(TAG, "BME280 init failure.");
    return;
  }

  while (true) {
    auto temp = bme280.GetTemperature();
    if (temp.has_value()) {
      ESP_LOGI(TAG, "Temp is %g.", temp.value());
    } else {
      ESP_LOGE(TAG, "Failure getting temp: %u.", temp.error());
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}