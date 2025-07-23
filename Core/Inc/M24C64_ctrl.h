#ifndef M24C64_CTRL_H
#define M24C64_CTRL_H

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef M24C64_ReadData(uint8_t* dataBuffer, uint16_t page, uint16_t size) {
  if (size > 32) {
    size = 32;
  }
  HAL_StatusTypeDef M24C64_status = HAL_I2C_Mem_Read(&hi2c1, 0XA0, (page<<6), 2, dataBuffer, size, HAL_MAX_DELAY);
  return M24C64_status;
};

HAL_StatusTypeDef M24C64_WriteData(uint8_t* dataBuffer, uint16_t page, uint16_t size) {
  if (size > 32) {
    size = 32;
  }
  HAL_StatusTypeDef M24C64_status = HAL_I2C_Mem_Write(&hi2c1, 0XA0, (page<<6), 2, dataBuffer, size, HAL_MAX_DELAY);
  HAL_Delay(10);
  return M24C64_status;
};

#endif