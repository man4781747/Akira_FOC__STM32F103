#ifndef FOC_DRIVER_H
#define FOC_DRIVER_H

#include <math.h>
#include "main.h"
#include "qfplib-m3.h"


#define _SQRT3_2 0.86602540378f
#define _SQRT3 1.73205080757f

// 由 main.c 提供
extern TIM_HandleTypeDef htim1;
// extern uint16_t pwmPluse_1;
// extern uint16_t pwmPluse_2;
// extern uint16_t pwmPluse_3;
int period_ticksSet = 1800;
float pwmNum = 900.f;
float fullTime = 0.5;

static inline void SetAng(float ang)
{
    float ang_ = fmodf(ang, 360.0f);
  //第一扇區
  if (ang_ < 60.) {
    //? 100 -> 110 -> 111 -> 110 -> 100
    //? U永不關、V調整、P負責輸出強度
    //? U4執行佔比多少，U6的佔空筆就設定多少
    //? U4+U6執行佔比多少，U7的佔空筆就設定多少

    double time_100 = sin( ( 60 - ang_) * M_PI / 180.0) * fullTime;
    double time_110 = sin( ang_ * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_100-time_110;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_100+time_110+time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_110+time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_000_111/2)*period_ticksSet/2);
  }
  else if (ang_ >= 60. && ang_ < 120.) {
    //? 110 -> 010 -> 000 -> 010 -> 110
    //? U調整、V負責輸出強度、P永不開

    double time_110 = sin( ( 60 - (ang_-60.)) * M_PI / 180.0) * fullTime;
    double time_010 = sin( (ang_-60.) * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_010-time_110;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_110+time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_110+time_010+time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_000_111/2)*period_ticksSet/2);
  }
  else if (ang_ >= 120. && ang_ < 180.) {
    //? 010 -> 011 -> 111 -> 011 -> 010
    //? U負責輸出強度、V永不關、P調整

    double time_010 = sin( ( 60 - (ang_-120.)) * M_PI / 180.0)  * fullTime;
    double time_011 = sin( (ang_-120.) * M_PI / 180.0)  * fullTime;
    double time_000_111 = 1-time_010-time_011;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_010+time_011+time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_011+time_000_111/2)*period_ticksSet/2);
  }
  else if (ang_ >= 180. && ang_ < 240.) {
    //? 011 -> 001 -> 000 -> 001 -> 011
    //? U永不開、V調整、P負責輸出強度

    double time_011 = sin( ( 60 - (ang_-180.)) * M_PI / 180.0) * fullTime;
    double time_001 = sin( (ang_-180.) * M_PI / 180.0)  * fullTime;
    double time_000_111 = 1-time_011-time_001;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_011+time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_011+time_001+time_000_111/2)*period_ticksSet/2);
  }
  else if (ang_ >= 240. && ang_ < 300.) {
    //? 001 -> 101 -> 111 -> 101 -> 001
    //? U調整、V負責輸出強度、P永不關

    double time_001 = sin( ( 60 - (ang_-240.)) * M_PI / 180.0) * fullTime;
    double time_101 = sin( (ang_-240.) * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_101-time_001;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_101+time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_001+time_101+time_000_111/2)*period_ticksSet/2); 
  }
  else if (ang_ >= 300. && ang_ < 360.) {
    //? 101 -> 100 -> 000 -> 100 -> 101
    //? U負責輸出強度、V永不開、P調整

    double time_101 = sin( ( 60 - (ang_-300.)) * M_PI / 180.0) * fullTime;
    double time_100 = sin( (ang_-300.) * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_101-time_100;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_101+time_100+time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_000_111/2)*period_ticksSet/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_101+time_000_111/2)*period_ticksSet/2);
  }
}

void runPWM(float Ta, float Tb, float Tc) {
  if (Ta>1) {Ta=1.;}
  else if (Ta<0) {Ta=0.;}
  if (Tb>1) {Tb=1.;}
  else if (Tb<0) {Tb=0.;}
  if (Tc>1) {Tc=1.;}
  else if (Tc<0) {Tc=0.;}
  
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, qfp_fmul(Ta, pwmNum));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, qfp_fmul(Tb, pwmNum));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, qfp_fmul(Tc, pwmNum));
}

void Svpwm(float uAlpha, float uBeta){
	float U1, U2, U3;


  float center = 5.f;
  U1 = uAlpha;
  U2 = qfp_fsub(qfp_fmul(_SQRT3_2, uBeta),qfp_fmul(uAlpha, 0.5f));
  // U2 = -0.5f * uAlpha + _SQRT3_2 * uBeta;
  U3 = qfp_fsub(qfp_fmul(-
    _SQRT3_2, uBeta),qfp_fmul(uAlpha, 0.5f));
  // U3 = -0.5f * uAlpha - _SQRT3_2 * uBeta;
  float Umin = fmin(U1, fmin(U2, U3));
  float Umax = fmax(U1, fmax(U2, U3));
  center = qfp_fsub(
    center,
    qfp_fmul(
      qfp_fadd(Umax, Umin), 0.5f
    )
  );
  // center -= (Umax+Umin) / 2;

  U1 = qfp_fadd(U1, center);
  // U1 += center;
  U2 = qfp_fadd(U2, center);
  // U2 += center;
  U3 = qfp_fadd(U3, center);
  // U3 += center;
  U1 = qfp_fdiv(U1, 10);
  // U1 /= 10;
  U2 = qfp_fdiv(U2, 10);
  // U2 /= 10;
  U3 = qfp_fdiv(U3, 10);
  // U3 /= 10;
  runPWM(U1, U2, U3);

  
	// float T1, T2, T3, T4, T5, T6, T7, Ts = 1.0f;
  // float Ta, Tb, Tc;
  // int Sector;
	// U1 =  uBeta;
  // U2 = qfp_fsub(qfp_fmul(uAlpha, _SQRT3_2), qfp_fmul(uBeta, 0.5f));
	// // U2 =  uAlpha * _SQRT3_2 - uBeta / 2;
  // U3 = qfp_fsub(qfp_fmul(uAlpha, -_SQRT3_2), qfp_fmul(uBeta, 0.5f));
	// // U3 = -uAlpha * _SQRT3_2 - uBeta / 2;
	// uint8_t A = 0, B = 0, C = 0;
	// if (U1 > 0) A = 1;
	// if (U2 > 0) B = 1;
	// if (U3 > 0) C = 1;

	// uint8_t N = 4*C + 2*B + A;
	// switch (N) {
	// 	case 3: Sector = 1; break;
	// 	case 1: Sector = 2; break;
	// 	case 5: Sector = 3; break;
	// 	case 4: Sector = 4; break;
	// 	case 6: Sector = 5; break;
	// 	case 2: Sector = 6; break;
	// }

  // float Ts_ = _SQRT3 * Ts / 10.;
  // switch (Sector) {
  //   case 1:
  //     T4 = Ts_ * U2;
  //     T6 = Ts_ * U1;
  //     T7 = (Ts - T4 - T6) / 2;
  //     Ta = T4 + T6 + T7;
  //     Tb = T6 + T7;
  //     Tc = T7;
  //     break;
  //   case 2:
  //     T2 = -Ts_ * U2;
  //     T6 = -Ts_ * U3;
  //     T7 = (Ts - T2 - T6) / 2;
  //     Ta = T6 + T7;
  //     Tb = T2 + T6 + T7;
  //     Tc = T7;
  //     break;
  //   case 3:
  //     T2 = Ts_ * U1;
  //     T3 = Ts_ * U3;
  //     T7 = (Ts - T2 - T3) / 2;
  //     Ta = T7;
  //     Tb = T2 + T3 + T7;
  //     Tc = T3 + T7;
  //     break;
  //   case 4:
  //     T1 = -Ts_ * U1;
  //     T3 = -Ts_ * U2;
  //     T7 = (Ts - T1 - T3) / 2;
  //     Ta = T7;
  //     Tb = T3 + T7;
  //     Tc = T1 + T3 + T7;
  //     break;
  //   case 5:
  //     T1 = Ts_ * U3;
  //     T5 = Ts_ * U2;
  //     T7 = (Ts - T1 - T5) / 2;
  //     Ta = T5 + T7;
  //     Tb = T7;
  //     Tc = T1 + T5 + T7;
  //     break;
  //   case 6:
  //     T4 = -Ts_ * U3;
  //     T5 = -Ts_ * U1;
  //     T7 = (Ts - T4 - T5) / 2;
  //     Ta = T4 + T5 + T7;
  //     Tb = T7;
  //     Tc = T5 + T7;
  //     break;
  //   default:
  //     Ta = 0;
  //     Tb = 0;
  //     Tc = 0;
  //     break;
  // }
  // runPWM(Ta, Tb, Tc);
}


#endif