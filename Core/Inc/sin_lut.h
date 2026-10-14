#ifndef SIN_LUT_H
#define SIN_LUT_H

#include <stdint.h>

// 一圈 256 格的正弦表 (Q15) 加線性內插。F103 沒有 FPU，qfp_fsin + qfp_fcos 要約 1000 個週期，查表約 100 個。
// 線性內插的最大誤差約 (2π/256)² / 8 ≈ 7.5e-5
#define SIN_LUT_N 256
extern const int16_t sin_lut[SIN_LUT_N + 1];

// angle：0 ~ 65535 對應 0 ~ 2π。回傳 sin、cos (-1 ~ 1)
static inline void Sin_Cos(uint16_t angle, float *s, float *c)
{
  uint32_t i = angle >> 8;
  int32_t f = angle & 0xFF;
  int32_t s0 = sin_lut[i];
  int32_t sv = s0 + (((sin_lut[i + 1] - s0) * f) >> 8);
  uint32_t j = (i + SIN_LUT_N / 4) & (SIN_LUT_N - 1);  // cos = sin(θ + π/2)
  int32_t c0 = sin_lut[j];
  int32_t cv = c0 + (((sin_lut[j + 1] - c0) * f) >> 8);
  *s = (float)sv * (1.0f / 32767.0f);
  *c = (float)cv * (1.0f / 32767.0f);
}

#endif
