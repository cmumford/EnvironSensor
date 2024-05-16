#pragma once

#include <esp_err.h>
#include <esp_event_base.h>
#include <i2clib/master.h>

#include <string_view>

#undef DEVICE_BME280
#define DEVICE_BME680

#include "app_prefs.h"
#include "logger.h"

#if defined(DEVICE_BME280)
#include "mybme280.h"
#elif defined(DEVICE_BME680)
#include "mybme680.h"
#else
#error "Unknown device";
#endif

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
#if defined(DEVICE_BME280)
  esp_err_t LogBME280();
#elif defined(DEVICE_BME680)
  esp_err_t LogBME680();
#else
#error "Unknown device";
#endif

  i2c::Master i2c_master_;
#if defined(DEVICE_BME280)
  BME280 bme280_;
#elif defined(DEVICE_BME680)
  BME680 bme680_;
#else
#error "Unknown device";
#endif
  bool initialized_ = false;
  AppPrefs prefs_;
  Logger logger_;
};