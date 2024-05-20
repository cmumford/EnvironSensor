#pragma once

#include <esp_err.h>
#include <esp_event_base.h>
#include <i2clib/master.h>

#include <cstdint>
#include <memory>
#include <string_view>

#include "app_prefs.h"
#include "logger.h"

#include "mybme280.h"
#include "mybme68x.h"

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
  esp_err_t InitI2C(uint32_t bus_speed);
  esp_err_t StartLogger();
  esp_err_t LogSensor();
  uint32_t GetI2CBusSpeed() const;
  void ConnectedCallback(bool connected);
  void PublishCallback(int msg_id);
  void ErrorCallback();
  void EnterDeepSleep();

  i2c::Master i2c_master_;
  std::unique_ptr<Sensor> sensor_;
  bool initialized_ = false;
  AppPrefs prefs_;
  Logger logger_;
  bool entering_sleep_ = false;
  uint16_t wifi_disconnect_count_ = 0;
};