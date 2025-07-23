#ifndef MICRO_TIMER_H
#define MICRO_TIMER_H

#include "main.h"

extern TIM_HandleTypeDef htim2;

volatile uint32_t uwTick_us_overflow = 0; // 微秒級溢出計數器

void init_micros_timer(void)
{
  // 啟動你配置的計時器 (例如 TIM2)
  HAL_TIM_Base_Start(&htim2);

  // 如果使用16位元計時器並需要長時間計數，啟用更新中斷
  // (這會讓每次計時器溢出時觸發中斷)
  HAL_TIM_Base_Start_IT(&htim2);
}

uint64_t micros(void)
{
  uint32_t current_count = __HAL_TIM_GET_COUNTER(&htim2);
  uint32_t overflow_count = uwTick_us_overflow;

  // 處理潛在的競態條件：如果在讀取計數器後發生溢出
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) && (current_count < 100)) // 檢查是否剛溢出 (計數器又從0開始)
  {
    overflow_count++;
  }

  return (uint64_t)overflow_count * (65535UL + 1) + current_count;
}

// 實作 Timer 更新中斷回調函數
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) // 替換為你使用的計時器實例
  {
    uwTick_us_overflow++;
  }
}

#endif