#ifndef FOC_SENSOR_H
#define FOC_SENSOR_H

#include <math.h>
#include "main.h"
#include "qfplib-m3.h"

// 由 main.c 提供
extern I2C_HandleTypeDef hi2c2;
HAL_StatusTypeDef status;
uint8_t accel_data[2];
extern uint16_t adc_dma_buffer[];
/**
 * 讀取機械角度
 */
float readAng(float shift) {

  status = HAL_I2C_Mem_Read(&hi2c2, 0x36 << 1, 0x0E, I2C_MEMADD_SIZE_8BIT, accel_data, 2, HAL_MAX_DELAY);
  if (status == HAL_OK) {
    uint16_t combined_data = (accel_data[0] << 8) | (accel_data[1] & 0xFF);
    float ang_ = ((float)combined_data/4096.0f*360.0f) - shift;
    if (ang_ < 0.f) {
      ang_ += 360.f;
    }
    return ang_;
  }
  return -1.f;
  // float ang_ = qfp_fsub(qfp_fmul((float)adc_dma_buffer[2], 0.08791208791208792) ,shift);
  // // float ang_ = ((float)(adc_dma_buffer[2])*0.08791208791208792) - shift; // 設置錯誤碼
  // if (ang_ < 0.f) {
  //   ang_ = qfp_fadd(ang_, 360.f);
  //   // ang_ += 360;
  // }
  // return ang_;
}

#endif