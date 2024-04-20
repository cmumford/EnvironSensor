#include <expected>

#include <bme280.h>

#pragma once

class BME280 {
 public:
  BME280();

  bool Init();
  std::expected<double, int8_t> GetTemperature();

 private:
  bool SetSettings();
  bool ReadSettings();
  bool SetSensorMode();
  bool CalcMeasurementDelay();

  bme280_dev dev_;
  bme280_settings settings_;
  uint32_t period_ = 0;
};