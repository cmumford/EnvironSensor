#include <expected>
#include <optional>

#include <bme280.h>
#include <i2clib/master.h>

#pragma once

constexpr uint8_t kBME280Pressure = 1;
constexpr uint8_t kBME280Temperature = 1 << 1;
constexpr uint8_t kBME280Humidity = 1 << 2;
constexpr uint8_t kBME280All = 0x07;

class BME280 {
 public:
  struct Data {
    std::optional<double> pressure;
    std::optional<double> temperature;
    std::optional<double> humidity;
  };

  BME280(i2c::Master& i2c_master);

  bool Init();
  // Retrieve the sensor data.
  // values is a bitmask.
  std::expected<Data, int8_t> GetData(uint8_t values);

 private:
  static BME280_INTF_RET_TYPE ReadFunc(uint8_t reg_addr,
                                       uint8_t* reg_data,
                                       uint32_t len,
                                       void* intf_ptr);
  static BME280_INTF_RET_TYPE WriteFunc(uint8_t reg_addr,
                                        const uint8_t* reg_data,
                                        uint32_t len,
                                        void* intf_ptr);
  static void DelayUsFunc(uint32_t period, void* intf_ptr);

  bool SetSettings();
  bool ReadSettings();
  bool SetSensorMode();
  bool CalcMeasurementDelay();

  bme280_dev dev_;
  bme280_settings settings_;
  uint32_t period_ = 0;
  i2c::Master& i2c_master_;
};