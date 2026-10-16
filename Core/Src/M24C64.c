#include <string.h>
#include "M24C64.h"

HAL_StatusTypeDef M24C64_Read(uint16_t addr, uint8_t *buf, uint16_t size)
{
  return HAL_I2C_Mem_Read(&hi2c1, M24C64_I2C_ADDR, addr, I2C_MEMADD_SIZE_16BIT, buf, size, M24C64_TIMEOUT_MS);
}

HAL_StatusTypeDef M24C64_Write(uint16_t addr, const uint8_t *buf, uint16_t size)
{
  // 一次寫入不能跨過晶片的 32 byte 頁邊界
  if (size == 0 || (addr & 31) + size > 32) {
    return HAL_ERROR;
  }
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, M24C64_I2C_ADDR, addr, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buf, size, M24C64_TIMEOUT_MS);
  HAL_Delay(10);  // 寫入週期 (規格書 tW 最長 5 ms)
  return st;
}

HAL_StatusTypeDef M24C64_ReadData(uint8_t *dataBuffer, uint16_t page, uint16_t size)
{
  if (size > 32) {
    size = 32;
  }
  return M24C64_Read(page << 6, dataBuffer, size);
}

HAL_StatusTypeDef M24C64_WriteData(uint8_t *dataBuffer, uint16_t page, uint16_t size)
{
  if (size > 32) {
    size = 32;
  }
  return M24C64_Write(page << 6, dataBuffer, size);
}

HAL_StatusTypeDef WritePreF(const float PreFArray[PRE_F_LEN])
{
  const uint16_t numPages = PRE_F_LEN * 4 / 32;
  for (int pageChose = 0; pageChose < numPages; pageChose++) {
    uint8_t pageBuffer[32];
    memcpy(pageBuffer, ((const uint8_t*)PreFArray) + (pageChose * 32), 32);
    HAL_StatusTypeDef st = M24C64_WriteData(pageBuffer, PAGE__PRE_F + pageChose, 32);
    if (st != HAL_OK) {
      return st;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef ReadPreF(float PreFArray[PRE_F_LEN])
{
  const uint16_t numPages = PRE_F_LEN * 4 / 32;
  for (int pageChose = 0; pageChose < numPages; pageChose++) {
    HAL_StatusTypeDef st = M24C64_ReadData(((uint8_t*)PreFArray) + (pageChose * 32), PAGE__PRE_F + pageChose, 32);
    if (st != HAL_OK) {
      return st;
    }
  }
  return HAL_OK;
}
