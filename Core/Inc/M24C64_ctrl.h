#ifndef M24C64_CTRL_H
#define M24C64_CTRL_H

#include "main.h"

#define PAGE__ANG_SHIFT 0
#define PAGE__PRE_F 1
#define PRE_F_LEN 360

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

float ReadAngShift() {
  uint8_t M24C64_AngShift[4];
  float angShift_float;
  M24C64_ReadData(M24C64_AngShift, PAGE__ANG_SHIFT, 4);
  memcpy(&angShift_float, M24C64_AngShift, 4);
  return angShift_float;
}

void WriteAngShift(float AngShift) {
  uint8_t M24C64_AngShift[4];
  memcpy(M24C64_AngShift, &AngShift, 4);
  M24C64_WriteData(M24C64_AngShift, PAGE__ANG_SHIFT, 4);
}

void WritePreF(float PreFArray[360]) {
  const uint16_t numPages = 360*4/32;
  for (int pageChose=0;pageChose<numPages;pageChose++){
    uint8_t pageBuffer[32];
    memcpy(pageBuffer, ((uint8_t*)PreFArray) + (pageChose * 32), 32);
    M24C64_WriteData(pageBuffer, PAGE__PRE_F+pageChose, 32);
  }
}

void ReadPreF(float PreFArray[360]) {
  const uint16_t numPages = 360*4/32;
  for (int pageChose=0;pageChose<numPages;pageChose++){
    M24C64_ReadData(((uint8_t*)PreFArray) + (pageChose * 32),PAGE__PRE_F+pageChose, 32);
  }
}


#endif