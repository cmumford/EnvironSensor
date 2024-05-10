#include <esp_log.h>

#include "app.h"

namespace {
constexpr char TAG[] = "main";
}  // namespace

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting app");
  App app;
  esp_err_t err = app.Init();
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "Failed to initialize app");
    return;
  }

  err = app.Run();
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "Failed to run app");
    return;
  }
}