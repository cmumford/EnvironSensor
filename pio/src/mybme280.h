#include <expected>
#include <optional>

#include <bme280.h>
#include <i2clib/master.h>

#include "sensor.h"

#pragma once

class BME280 : public Sensor {
 public:
  static uint32_t MaxI2CClockSpeed();

  BME280(i2c::Master& i2c_master);

  // Sensor:
  bool Init() override;
  std::expected<SensorData, esp_err_t> ReadData(uint8_t values) override;
  esp_err_t EnterSleep() override;

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