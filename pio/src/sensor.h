#pragma once

#include <cstdint>
#include <expected>

#include "sensor_data.h"

// Bitmask values for GetData.
constexpr uint8_t kBME280Pressure = 1;
constexpr uint8_t kBME280Temperature = 1 << 1;
constexpr uint8_t kBME280Humidity = 1 << 2;
constexpr uint8_t kBME280All = 0x07;

class Sensor {
 public:
  virtual ~Sensor() = default;

  virtual bool Init() = 0;

  // Retrieve the sensor data.
  // |values| is a bitmask identifying the desired values to read from the
  // sensor - See kBME280All above. If this function succeeds then all
  // requested values will be set.
  virtual std::expected<SensorData, esp_err_t> ReadData(uint8_t values) = 0;

  // Put the device in sleep mode.
  virtual esp_err_t EnterSleep() = 0;

 protected:
  Sensor() = default;
};