#include "logger.h"

#include <cstdio>
#include <cstring>

#include <esp_err.h>
#include <esp_event.h>
#include <esp_log.h>
#include <mqtt_client.h>

namespace {
constexpr char TAG[] = "Logger";
}  // namespace

Logger::Logger() = default;

void Logger::MQTTEventHandler(void* handler_args,
                              esp_event_base_t base,
                              int32_t event_id,
                              void* event_data) {
  const esp_mqtt_event_handle_t event =
      static_cast<esp_mqtt_event_handle_t>(event_data);
  Logger* logger = static_cast<Logger*>(handler_args);
  switch (event->event_id) {
    case MQTT_EVENT_ERROR:
      ESP_LOGI(TAG, "MQTT error");
      break;
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT connected");
      logger->connected_ = true;
      break;
    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "MQTT disconnected");
      logger->connected_ = false;
      break;
    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "MQTT subscribed, msg_id=%d", event->msg_id);
      break;
    case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(TAG, "MQTT unsubscribed, msg_id=%d", event->msg_id);
      break;
    case MQTT_EVENT_PUBLISHED:
      ESP_LOGI(TAG, "MQTT message published, msg_id=%d", event->msg_id);
      break;
    case MQTT_EVENT_DATA:
      ESP_LOGI(TAG, "MQTT data received, topic=%.*s, data=%.*s",
               event->topic_len, event->topic, event->data_len, event->data);
      break;
    case MQTT_EVENT_BEFORE_CONNECT:
      ESP_LOGI(TAG, "MQTT before connect");
      break;
    case MQTT_EVENT_DELETED:
      ESP_LOGI(TAG, "MQTT deleted");
      break;
    case MQTT_USER_EVENT:
      ESP_LOGI(TAG, "MQTT user event");
      break;
    case MQTT_EVENT_ANY:
      ESP_LOGI(TAG, "MQTT any event");
      break;
      break;
  }
}

esp_err_t Logger::StartClient(const AppPrefs& prefs) {
  if (connected_)
    return ESP_ERR_INVALID_STATE;
  esp_mqtt_client_config_t mqtt_cfg;
  std::memset(&mqtt_cfg, 0, sizeof(mqtt_cfg));
  mqtt_cfg.broker.address.uri = prefs.mqtt_uri().c_str();
  mqtt_cfg.credentials.username = prefs.mqtt_username().c_str();
  mqtt_cfg.credentials.authentication.password = prefs.mqtt_password().c_str();
  if ((client_ = esp_mqtt_client_init(&mqtt_cfg)); client_ == nullptr)
    return ESP_FAIL;
  esp_err_t err = esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY,
                                                 MQTTEventHandler, this);
  if (err != ESP_OK)
    return err;
  return esp_mqtt_client_start(client_);
}

esp_err_t Logger::LogSensorData(const AppPrefs& prefs,
                                const BME280::Data& data) {
  if (!connected_)
    return ESP_ERR_INVALID_STATE;
  char buff[120];
  int len;

  if (data.temperature.has_value()) {
    len =
        std::snprintf(buff, sizeof(buff),
                      "environment,location=%s "
                      "temperature=%g,humidity=%g,pressure=%g",
                      prefs.sensor_location().c_str(), data.temperature.value(),
                      data.humidity.value(), data.pressure.value());
    char topic[80];
    std::snprintf(topic, sizeof(topic), "sensors/%s",
                  prefs.sensor_name().c_str());
    esp_mqtt_client_publish(client_, topic, buff, len, /*qos=*/1,
                            /*retain=*/0);
  }

  return ESP_OK;
}
