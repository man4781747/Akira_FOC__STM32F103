#include "micro_timer.h"

static volatile uint32_t uwTick_us_overflow = 0; // 微秒級溢出計數器

void init_micros_timer(void)
{
  // 16 bit 計時器，每 65.536 ms 溢位一次，由更新中斷累計溢位次數
  HAL_TIM_Base_Start_IT(&htim2);
}

uint64_t micros(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t current_count = __HAL_TIM_GET_COUNTER(&htim2);
  uint32_t overflow_count = uwTick_us_overflow;
  // 讀計數之前剛溢位、更新中斷還沒執行 (關中斷期間，或呼叫者的優先權比 TIM2 高)：旗標還在，自己補上
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) {
    current_count = __HAL_TIM_GET_COUNTER(&htim2);
    overflow_count++;
  }
  __set_PRIMASK(primask);
  return (uint64_t)overflow_count * (65535UL + 1) + current_count;
}

void overflow_handler(void)
{
  uwTick_us_overflow++;
}
