#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include <esp_err.h>

enum class SensorType {
  Unknown,
  BME280,
  BME680,
};

class AppPrefs {
 public:
  AppPrefs();

  // Load prefs from NVS and return error code and boolean to indicate that
  // the preferences were safely migrated safely (but should probably be saved).
  // Prefs will be migrated when new values are added to the app, but are not
  // present in NVS and can be given a default value.
  std::pair<esp_err_t, bool> Load();
  // Save the preferences to NVS.
  esp_err_t Save();

  const std::string& mqtt_uri() const { return mqtt_uri_; }
  const std::string& mqtt_username() const { return mqtt_username_; }
  const std::string& mqtt_password() const { return mqtt_password_; }
  const std::string& wifi_ssid() const { return wifi_ssid_; }
  const std::string& wifi_password() const { return wifi_password_; }
  const std::string& sensor_name() const { return sensor_name_; }
  const std::string& sensor_location() const { return sensor_location_; }
  uint16_t sleep_duration_secs() const { return sleep_duration_secs_; }
  SensorType sensor_type() const { return sensor_type_; }

 private:
  std::string mqtt_uri_;
  std::string mqtt_username_;
  std::string mqtt_password_;
  std::string wifi_ssid_;
  std::string wifi_password_;
  std::string sensor_name_;
  std::string sensor_location_;
  uint16_t sleep_duration_secs_ = 0;  // zero = don't sleep.
  SensorType sensor_type_ = SensorType::Unknown;
};