#pragma once

#include <functional>

#include "mybme280.h"
#include "mybme68x.h"

#include <esp_event_base.h>
#include <mqtt_client.h>

#include "app_prefs.h"
#include "sensor_data.h"

class Logger {
 public:
  using ConnectCallback = std::function<void(bool connected)>;
  using PublishedCallback = std::function<void(int msg_id)>;
  using MQTTErrorCallback = std::function<void()>;

  // `prefs` is guaranteed to live longer than this instance.
  Logger();

  esp_err_t StartClient(const AppPrefs& prefs,
                        ConnectCallback connect_callback,
                        PublishedCallback publish_callback,
                        MQTTErrorCallback error_callback);
  esp_err_t LogSensorData(const AppPrefs& prefs, const SensorData& data);
  bool connected() const { return connected_; }

 private:
  static void MQTTEventHandler(void* handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void* event_data);

  bool connected_ = false;
  esp_mqtt_client_handle_t client_ = nullptr;
  ConnectCallback connect_callback_;
  PublishedCallback publish_callback_;
  MQTTErrorCallback error_callback_;
};