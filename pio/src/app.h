#pragma once

#include <esp_err.h>
#include <esp_event_base.h>
#include <i2clib/master.h>

#include <string_view>

#include "app_prefs.h"
#include "logger.h"
#include "mybme280.h"

class App {
 public:
  App();

  esp_err_t Init();
  esp_err_t Run();

 private:
  static void WifiEventHandler(void* event_handler_arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void* event_data);

  esp_err_t ConnectToWifi();
  esp_err_t InitI2C();
  esp_err_t StartLogger();

  i2c::Master i2c_master_;
  BME280 bme280_;
  bool initialized_ = false;
  AppPrefs prefs_;
  Logger logger_;
};