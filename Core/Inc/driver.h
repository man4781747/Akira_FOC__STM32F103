#ifndef FOC_DRIVER_H
#define FOC_DRIVER_H

#include <math.h>
#include <stdint.h>
#include "main.h"
#include "setting.h"
#include "micro_timer.h"
#include "qfplib-m3.h"
#include "lowpass_filter.h"
#include "PID_Ctrl.h"
#include "sensor.h"


// 由 main.c 提供
extern TIM_HandleTypeDef htim1;
extern uint64_t old_ang_time, new_ang_time;
extern float ang_pre, d_ang, ang_speed, ang_temp, angShift, angToRad;
extern LowPassFilter filter_U, filter_V, filter_W, filter_Speed;
extern float current_U, current_V, current_W;
extern uint16_t adc_bios_W, adc_bios_U;
extern float u_alpha, u_beta;
extern float I_alpha, I_beta;
extern float cosRad, sinRad;
extern float I_d, I_q;
extern float uq;
extern float ud;
extern float ang_temp, angToRad;
extern float U1, U2, U3;
extern float positon_Target,speed__Target, Iq__Target;
extern enum DeviceMode deviceMode;
extern PIDController PID__current_Id, PID__current_Iq, PID__velocity, PID__position;
extern float exf;
extern float exf_map[360];
// extern float cogging_calibration_map[360];


int logCount = 0;
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

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_100+time_110+time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_110+time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_000_111/2)*htim1.Init.Period/2);
  }
  else if (ang_ >= 60. && ang_ < 120.) {
    //? 110 -> 010 -> 000 -> 010 -> 110
    //? U調整、V負責輸出強度、P永不開

    double time_110 = sin( ( 60 - (ang_-60.)) * M_PI / 180.0) * fullTime;
    double time_010 = sin( (ang_-60.) * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_010-time_110;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_110+time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_110+time_010+time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_000_111/2)*htim1.Init.Period/2);
  }
  else if (ang_ >= 120. && ang_ < 180.) {
    //? 010 -> 011 -> 111 -> 011 -> 010
    //? U負責輸出強度、V永不關、P調整

    double time_010 = sin( ( 60 - (ang_-120.)) * M_PI / 180.0)  * fullTime;
    double time_011 = sin( (ang_-120.) * M_PI / 180.0)  * fullTime;
    double time_000_111 = 1-time_010-time_011;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_010+time_011+time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_011+time_000_111/2)*htim1.Init.Period/2);
  }
  else if (ang_ >= 180. && ang_ < 240.) {
    //? 011 -> 001 -> 000 -> 001 -> 011
    //? U永不開、V調整、P負責輸出強度

    double time_011 = sin( ( 60 - (ang_-180.)) * M_PI / 180.0) * fullTime;
    double time_001 = sin( (ang_-180.) * M_PI / 180.0)  * fullTime;
    double time_000_111 = 1-time_011-time_001;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_011+time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_011+time_001+time_000_111/2)*htim1.Init.Period/2);
  }
  else if (ang_ >= 240. && ang_ < 300.) {
    //? 001 -> 101 -> 111 -> 101 -> 001
    //? U調整、V負責輸出強度、P永不關

    double time_001 = sin( ( 60 - (ang_-240.)) * M_PI / 180.0) * fullTime;
    double time_101 = sin( (ang_-240.) * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_101-time_001;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_101+time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_001+time_101+time_000_111/2)*htim1.Init.Period/2); 
  }
  else if (ang_ >= 300. && ang_ < 360.) {
    //? 101 -> 100 -> 000 -> 100 -> 101
    //? U負責輸出強度、V永不開、P調整

    double time_101 = sin( ( 60 - (ang_-300.)) * M_PI / 180.0) * fullTime;
    double time_100 = sin( (ang_-300.) * M_PI / 180.0) * fullTime;
    double time_000_111 = 1-time_101-time_100;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (time_101+time_100+time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (time_000_111/2)*htim1.Init.Period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (time_101+time_000_111/2)*htim1.Init.Period/2);
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

void DoFoc() {
  new_ang_time = micros();
  ang_temp = readAng(angShift);
  if (ang_pre == -100) {
    ang_pre = ang_temp;
    old_ang_time = new_ang_time-1;
  }

  d_ang = ang_temp - ang_pre;
  if (d_ang > 180) {
    d_ang -= 360;
  } else if (d_ang < -180) {
    d_ang += 360;
  }
  int dTime = (int)new_ang_time-(int)old_ang_time;
  if (dTime <= 0) {
    dTime += 65535;
  }
    
  ang_speed = LowPassFilter_Update(&filter_Speed, qfp_fmul(qfp_fdiv(d_ang,dTime),2777.777777777778f));
  // ang_speed = d_ang/dTime*2777.777777777778f;
  
  ang_pre = ang_temp;
  old_ang_time = new_ang_time;
  // angToRad = ang_temp*7*0.01745329251f;
  angToRad = qfp_fmul(qfp_fmul(ang_temp, 7), 0.01745329251f);

  // uint16_t W_add = 0;
  // uint16_t U_add = 0;
  // for (int i = 0; i < ADC_BUF_SIZE_BUFFER; i++) {
  //   W_add += adc_dma_buffer[2*i];
  //   U_add += adc_dma_buffer[2*i+1];
  // }
  // W_add /= ADC_BUF_SIZE_BUFFER;
  // U_add /= ADC_BUF_SIZE_BUFFER;

  // current_W = qfp_fmul(adc_dma_buffer[0]-adc_bios_W, 0.0008056640625f);
  // current_U = qfp_fmul(adc_dma_buffer[1]-adc_bios_U, 0.0008056640625f);
  // current_V = qfp_fsub(-current_U, current_W);
  current_W = LowPassFilter_Update(&filter_W, (float)(adc_dma_buffer[0]-adc_bios_W)*0.0008056640625f);
  current_U = LowPassFilter_Update(&filter_U, (float)(adc_dma_buffer[1]-adc_bios_U)*0.0008056640625f);
  current_V = LowPassFilter_Update(&filter_V, -current_U - current_W);



  I_alpha = current_V;
  I_beta = qfp_fmul(
    qfp_fadd(
      qfp_fmul(current_U,2), 
      current_V
    ),_1_SQRT3
  );
  // I_beta = qfp_fadd(current_U*2, current_V);

  cosRad = qfp_fcos(angToRad);
  sinRad = qfp_fsin(angToRad);
  // cosRad = cos(angToRad);
  // sinRad = sin(angToRad);

  I_d = qfp_fadd( qfp_fmul(I_alpha, cosRad), qfp_fmul(I_beta, sinRad));
  I_q = qfp_fsub( qfp_fmul(I_beta, cosRad), qfp_fmul(I_alpha, sinRad));
  // I_d = I_alpha*cosRad + I_beta*sinRad;
  // I_q = -I_alpha*sinRad + I_beta*cosRad;
  
  float d_ang = positon_Target - ang_temp;
  if (d_ang > 180) {
    d_ang = 360 - d_ang;
  } else if (d_ang < -180) {
    d_ang += 360;
  }

  // speed__Target = qfp_fsin(qfp_fmul(logCount++/30, 0.01745329251f))*10;

  // if (logCount++ > 500) {
  //   // printf("%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f, %.2f, %.2f\n", 
  //   //   ang_temp,I_q, I_d,current_W, current_U, current_V, ang_speed, U2, U3
  //   // );
  //   // positon_Target += 30;
  //   // if (positon_Target >= 360) {
  //   //   positon_Target -= 360;
  //   // }
  //   logCount = 0;
    
  //   // speed__Target += test123;
  //   // // if (speed__Target > 10 || speed__Target < -10) {
  //   // //   speed__Target = -speed__Target;
  //   // // }
  //   // if (speed__Target > 10 || speed__Target < -10) {
  //   //   test123 = -test123;
  //   // }
  //   // Iq__Target += 0.1;
  //   positon_Target += 30;
  //   if (positon_Target >= 360) {
  //     positon_Target -= 360;
  //   }
  //   // if (ang_Target == 90) {
  //   //   ang_Target = 270;
  //   // } else {
  //   //   ang_Target = 90;
  //   // }
  //   // speed__Target = -speed__Target;
  //   // if (speed__Target == 0) {
  //   //   speed__Target = 1./60.;
  //   // }
  //   // if (speed__Target == 1./60.) {
  //   //   speed__Target = -1./60.;
  //   // } else {
  //   //   speed__Target = 1./60.;
  //   // }
  // } 


  if (deviceMode == DeviceMode_PositionMode) {
    float ang_error = qfp_fsub(positon_Target, ang_temp);
    if (ang_error > 180) {
      ang_error = qfp_fsub(360.0f , ang_error);
    } else if (ang_error < -180) {
      ang_error= qfp_fadd(ang_error, 360.0f);
    }
    // PID
    // speed__Target = PIDController_process(&PID__position, ang_error);
    // MIT
    Iq__Target = exf_map[(int)positon_Target] + exf + qfp_fadd(qfp_fmul(0.01f, ang_error) , qfp_fmul(0.15f ,-ang_speed));
    // Iq__Target = 0.f + 0.0005f * (fabs(ang_error)*ang_error) + 0.04f * (-ang_speed*fabs(ang_speed));
  }
  else {
    if (speed__Target > 1 || speed__Target < -1) {
      PID__velocity.P = 0.048;
      PID__velocity.I = 0.0925;
      // Iq__Target = exf_map[(int)ang_temp] + PIDController_process(&PID__velocity, qfp_fsub(speed__Target,ang_speed));
      Iq__Target = PIDController_process(&PID__velocity, qfp_fsub(speed__Target,ang_speed));
    } 
    else if (fabs(speed__Target) > 0.5 && fabs(speed__Target) <= 1) {
      PID__velocity.P = 0.085;
      PID__velocity.I = 0.75;
      // Iq__Target = exf_map[(int)ang_temp] + PIDController_process(&PID__velocity, qfp_fsub(speed__Target,ang_speed));
      Iq__Target = PIDController_process(&PID__velocity, qfp_fsub(speed__Target,ang_speed));
    } 
    else {
      // PID__velocity.P = 0.2;
      // PID__velocity.I = 3.6;

      PID__velocity.P = 0.4;
      PID__velocity.I = 0.08;
      PID__velocity.D = 0.001;

      // PID__velocity.P = 0.523;
      // PID__velocity.I = 0.025;
      // PID__velocity.D = 0.00;

      Iq__Target = exf_map[(int)ang_temp] + PIDController_process(&PID__velocity, qfp_fsub(speed__Target,ang_speed));
      // Iq__Target = PIDController_process(&PID__velocity, qfp_fsub(speed__Target,ang_speed));
    }
  }
  // if (speed__Target > 1 || speed__Target < -1) {
  //   PID__velocity.P = 0.048;
  //   PID__velocity.I = 0.0925;
  //   Iq__Target = PIDController_process(&PID__velocity, qfp_fsub(speed__Target,ang_speed));
  // } 
  // else if (fabs(speed__Target) > 0.5 && fabs(speed__Target) <= 1) {
  //   PID__velocity.P = 0.085;
  //   PID__velocity.I = 0.75;
  //   Iq__Target = PIDController_process(&PID__velocity, qfp_fsub(speed__Target,ang_speed));
  // } 
  // else {
  //   // PID__velocity.P = 0.2;
  //   // PID__velocity.I = 3.6;
  //   PID__velocity.P = 0.2;
  //   PID__velocity.I = 3.6;
  //   Iq__Target =  PIDController_process(&PID__velocity, qfp_fsub(speed__Target,ang_speed));
  // }


  // MIT 測試
  // int test = 360/10;
  // positon_Target = (((int)ang_temp)/test)*test + test/2;
  // float ang_error = positon_Target - ang_temp;
  // if (ang_error > 180) {
  //   ang_error = 360.0f - ang_error;
  // } else if (ang_error < -180) {
  //   ang_error += 360;
  // }
  // Iq__Target = qfp_fadd(qfp_fmul(0.01f, ang_error) , qfp_fmul(0.15f ,-ang_speed));


  uq = PIDController_process(&PID__current_Iq, qfp_fsub(Iq__Target, I_q));
  ud = PIDController_process(&PID__current_Id, -I_d);

  u_alpha = qfp_fsub( qfp_fmul(ud, cosRad), qfp_fmul(uq, sinRad));
  u_beta = qfp_fadd( qfp_fmul(ud, sinRad), qfp_fmul(uq, cosRad));
  // u_alpha = ud*cosRad-uq*sinRad;
  // u_beta = ud*sinRad+uq*cosRad;

  Svpwm(u_alpha, u_beta);
  // printf("%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f, %.4f, %.2f\n", 
  //   ang_temp,positon_Target,
  //   ang_speed, speed__Target,
  //   I_q, Iq__Target,current_W, current_U, current_V, I_d
  // );
}
#endif