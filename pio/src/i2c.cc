#include "i2c.h"

#include <driver/i2c.h>

#include "sdkconfig.h"

namespace {
constexpr i2c_port_t kI2CPort = I2C_NUM_0;
}

I2CMaster::I2CMaster() = default;

esp_err_t I2CMaster::Init() {
  constexpr int I2C_MASTER_SCL_IO = 22;
  constexpr int I2C_MASTER_SDA_IO = 21;
  constexpr int I2C_MASTER_FREQ_HZ = 400000;

  constexpr size_t kTransmitBufSize = 0;  // Master doesn't need buffer.
  constexpr size_t kReceiveBufSize = 0;   // Master doesn't need buffer.

  constexpr i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master =
          {
              .clk_speed = I2C_MASTER_FREQ_HZ,
          },
      .clk_flags = 0,
  };

  esp_err_t err = i2c_param_config(kI2CPort, &conf);
  if (err != ESP_OK)
    return err;

  return i2c_driver_install(kI2CPort, conf.mode, kReceiveBufSize,
                            kTransmitBufSize, 0);
}

esp_err_t I2CMaster::Read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len) {
  constexpr TickType_t kReadWaitTicks = pdMS_TO_TICKS(1000);
  return i2c_master_read_from_device(kI2CPort, reg_addr, reg_data, len,
                                     kReadWaitTicks);
}

esp_err_t I2CMaster::Write(uint8_t reg_addr,
                           const uint8_t* reg_data,
                           uint32_t len) {
  constexpr TickType_t kWriteWaitTicks = pdMS_TO_TICKS(1000);
  return i2c_master_write_to_device(kI2CPort, reg_addr, reg_data, len,
                                    kWriteWaitTicks);
}