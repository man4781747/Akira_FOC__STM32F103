#ifndef FOC_DRIVER_H
#define FOC_DRIVER_H

#include <math.h>
#include <stdint.h>
#include "main.h"
#include "setting.h"

// 與後代 (STM32F446RE_TEST) 的 driver.h 相同，計時器換成 TIM1，函式都是 static inline，Svpwm() 改成以整數計算。
// 比較值 = duty × ARR，duty 用到完整的 0 ~ 100% (2026-10-14 以前只乘 900，duty 上限 50%)

// 由 main.c 提供
extern TIM_HandleTypeDef htim1;

// SetAng() 開環向量的強度 (duty 比例)，同後代
static const float fullTime = 0.2f;

static inline void SetAng(float ang)
{
  float ang_ = fmodf(ang, 360.0f);
  const float period = (float)htim1.Init.Period;
  //第一扇區
  if (ang_ < 60.) {
    //? 100 -> 110 -> 111 -> 110 -> 100
    //? U永不關、V調整、P負責輸出強度
    //? U4執行佔比多少，U6的佔空筆就設定多少
    //? U4+U6執行佔比多少，U7的佔空筆就設定多少

    double time_100 = sin( ( 60 - ang_) * M_PI / 180.0) * fullTime;
    double time_110 = sin( ang_ * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_100-time_110;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_100+time_110+time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_110+time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_000_111/2)*period);
  }
  else if (ang_ >= 60. && ang_ < 120.) {
    //? 110 -> 010 -> 000 -> 010 -> 110
    //? U調整、V負責輸出強度、P永不開

    double time_110 = sin( ( 60 - (ang_-60.)) * M_PI / 180.0) * fullTime;
    double time_010 = sin( (ang_-60.) * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_010-time_110;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_110+time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_110+time_010+time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_000_111/2)*period);
  }
  else if (ang_ >= 120. && ang_ < 180.) {
    //? 010 -> 011 -> 111 -> 011 -> 010
    //? U負責輸出強度、V永不關、P調整

    double time_010 = sin( ( 60 - (ang_-120.)) * M_PI / 180.0)  * fullTime;
    double time_011 = sin( (ang_-120.) * M_PI / 180.0)  * fullTime;
    double time_000_111 = 1-time_010-time_011;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_010+time_011+time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_011+time_000_111/2)*period);
  }
  else if (ang_ >= 180. && ang_ < 240.) {
    //? 011 -> 001 -> 000 -> 001 -> 011
    //? U永不開、V調整、P負責輸出強度

    double time_011 = sin( ( 60 - (ang_-180.)) * M_PI / 180.0) * fullTime;
    double time_001 = sin( (ang_-180.) * M_PI / 180.0)  * fullTime;
    double time_000_111 = 1-time_011-time_001;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_011+time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_011+time_001+time_000_111/2)*period);
  }
  else if (ang_ >= 240. && ang_ < 300.) {
    //? 001 -> 101 -> 111 -> 101 -> 001
    //? U調整、V負責輸出強度、P永不關

    double time_001 = sin( ( 60 - (ang_-240.)) * M_PI / 180.0) * fullTime;
    double time_101 = sin( (ang_-240.) * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_101-time_001;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_101+time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_001+time_101+time_000_111/2)*period);
  }
  else if (ang_ >= 300. && ang_ < 360.) {
    //? 101 -> 100 -> 000 -> 100 -> 101
    //? U負責輸出強度、V永不開、P調整

    double time_101 = sin( ( 60 - (ang_-300.)) * M_PI / 180.0) * fullTime;
    double time_100 = sin( (ang_-300.) * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_101-time_100;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_101+time_100+time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_000_111/2)*period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_101+time_000_111/2)*period);
  }
}

static inline void runPWM(float Ta, float Tb, float Tc) {
  if (Ta>1) {Ta=1.;}
  else if (Ta<0) {Ta=0.;}
  if (Tb>1) {Tb=1.;}
  else if (Tb<0) {Tb=0.;}
  if (Tc>1) {Tc=1.;}
  else if (Tc<0) {Tc=0.;}

  const float period = (float)htim1.Init.Period;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(Ta*period));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(Tb*period));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(Tc*period));
}

// α/β 電壓以比較值為單位 (a = uα、b = uβ × √3/2)：三相、min-max 零序注入置中、限幅後寫入 TIM1
static inline void Svpwm_Counts(int32_t a, int32_t b){
  const int32_t period = (int32_t)htim1.Init.Period;
  int32_t U1 = a;
  int32_t U2 = -a / 2 + b;
  int32_t U3 = -a / 2 - b;

  int32_t Umin = U1;
  if (U2 < Umin) Umin = U2;
  if (U3 < Umin) Umin = U3;
  int32_t Umax = U1;
  if (U2 > Umax) Umax = U2;
  if (U3 > Umax) Umax = U3;

  int32_t center = period / 2 - (Umax + Umin) / 2;
  U1 += center;
  U2 += center;
  U3 += center;
  if (U1 < 0) U1 = 0; else if (U1 > period) U1 = period;
  if (U2 < 0) U2 = 0; else if (U2 > period) U2 = period;
  if (U3 < 0) U3 = 0; else if (U3 > period) U3 = period;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)U1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)U2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)U3);
}

// 輸入是 α/β 電壓命令，以 10 V 為基準 (±5 對應 duty 0 ~ 100%)，min-max 零序注入置中。
// 和後代的算式相同：duty = (U + 5 − (Umax + Umin)/2) × 0.1，比較值 = duty × ARR。
// F103 沒有 FPU，換成比較值 (整數) 之後才做三相、min-max 與限幅 (Svpwm_Counts)
static inline void Svpwm(float uAlpha, float uBeta){
  const float k = (float)htim1.Init.Period * 0.1f;  // 每單位電壓命令對應的比較值
  Svpwm_Counts((int32_t)(uAlpha * k), (int32_t)(uBeta * (k * _SQRT3_2)));
}

#endif
