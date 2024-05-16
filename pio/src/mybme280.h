#include <expected>
#include <optional>

#include <bme280.h>
#include <i2clib/master.h>

#include "sensor_data.h"

#pragma once

// Bitmask values for GetData.
constexpr uint8_t kBME280Pressure = 1;
constexpr uint8_t kBME280Temperature = 1 << 1;
constexpr uint8_t kBME280Humidity = 1 << 2;
constexpr uint8_t kBME280All = 0x07;

class BME280 {
 public:
  BME280(i2c::Master& i2c_master);

  bool Init();
  // Retrieve the sensor data.
  // |values| is a bitmask identifying the desired values to read from the
  // sensor - See kBME280All above. If this function succeeds then all
  // requested values will be set.
  std::expected<SensorData, int8_t> ReadData(uint8_t values);

 private:
  // A read callback function used by the BME280 library.
  static BME280_INTF_RET_TYPE ReadFunc(uint8_t reg_addr,
                                       uint8_t* reg_data,
                                       uint32_t len,
                                       void* intf_ptr);
  // A write callback function used by the BME280 library.
  static BME280_INTF_RET_TYPE WriteFunc(uint8_t reg_addr,
                                        const uint8_t* reg_data,
                                        uint32_t len,
                                        void* intf_ptr);
  // A delay callback function used by the BME280 library.
  static void DelayUsFunc(uint32_t period, void* intf_ptr);

  bool WriteSettings();
  bool ReadSettings();
  bool SetSensorMode();
  bool CalcMeasurementDelay();

  bme280_dev dev_;
  bme280_settings settings_;
  uint32_t period_ = 0;
  i2c::Master& i2c_master_;
};