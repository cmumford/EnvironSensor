#pragma once

#include <string>

#include <esp_err.h>

class AppPrefs {
 public:
  AppPrefs();

  esp_err_t Load();
  esp_err_t Save();

  const std::string& mqtt_uri() const { return mqtt_uri_; }
  const std::string& mqtt_username() const { return mqtt_username_; }
  const std::string& mqtt_password() const { return mqtt_password_; }
  const std::string& wifi_ssid() const { return wifi_ssid_; }
  const std::string& wifi_password() const { return wifi_password_; }
  const std::string& sensor_name() const { return sensor_name_; }
  const std::string& sensor_location() const { return sensor_location_; }

 private:
  std::string mqtt_uri_;
  std::string mqtt_username_;
  std::string mqtt_password_;
  std::string wifi_ssid_;
  std::string wifi_password_;
  std::string sensor_name_;
  std::string sensor_location_;
};