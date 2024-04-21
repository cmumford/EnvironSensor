#include <expected>

#include <bme280.h>
#include <i2clib/master.h>

#pragma once

class BME280 {
 public:
  BME280(i2c::Master& i2c_master);

  bool Init();
  std::expected<double, int8_t> GetTemperature();

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