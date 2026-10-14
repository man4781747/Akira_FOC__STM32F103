#ifndef PI_FIX_H
#define PI_FIX_H

#include <stdint.h>

// 定點數 PI (這一代的電流環用)。F103 沒有 FPU，電流環以浮點計算放不進 20 kHz 的 FOC 中斷。
// 算法同 PIDController_run() 的 P + I 部分：積分以 Tustin 累加、積分與輸出都限幅在 ±limit。
// kp、ki 由 PIDController 的 P、I (浮點) 換算 (main.c 的 Ctrl_Sync())，SET / GET 的單位不變
typedef struct {
  int32_t kp;       // Q16：輸出單位 / 輸入單位
  int32_t ki;       // Q16：I × Ts / 2，換成輸出單位 / 輸入單位
  int32_t limit;    // 輸出單位
  int64_t integ;    // 積分 (Q16 輸出單位)
  int32_t e_prev;
} PiFix;

static inline int32_t PiFix_Run(PiFix *pi, int32_t e)
{
  int64_t lim = (int64_t)pi->limit << 16;
  int64_t integ = pi->integ + (int64_t)pi->ki * (e + pi->e_prev);
  if (integ > lim) integ = lim;
  else if (integ < -lim) integ = -lim;
  int64_t out = (int64_t)pi->kp * e + integ;
  if (out > lim) out = lim;
  else if (out < -lim) out = -lim;
  pi->integ = integ;
  pi->e_prev = e;
  return (int32_t)(out >> 16);
}

static inline void PiFix_Reset(PiFix *pi)
{
  pi->integ = 0;
  pi->e_prev = 0;
}

#endif
