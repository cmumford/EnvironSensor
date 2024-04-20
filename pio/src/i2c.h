#pragma once

#include <esp_err.h>

class I2CMaster {
 public:
  I2CMaster();

  esp_err_t Init();
  esp_err_t Read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len);
  esp_err_t Write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len);
};