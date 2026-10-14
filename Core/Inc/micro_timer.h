#ifndef MICRO_TIMER_H
#define MICRO_TIMER_H

#include <stdint.h>
#include "main.h"

// TIM2：PSC = 71，1 MHz、16 bit。溢位次數由 TIM2 更新中斷 (overflow_handler) 累加，
// 組成 64 bit 的微秒計數。TIM2 中斷的優先權要最高，否則在其他中斷內呼叫 micros() 會少算一次溢位
extern TIM_HandleTypeDef htim2;

void init_micros_timer(void);
uint64_t micros(void);
void overflow_handler(void);

// micros() 的低 16 bit，等於 TIM2 計數值，擷取的 t_us 欄位用
static inline uint16_t micros16(void)
{
  return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

#endif
