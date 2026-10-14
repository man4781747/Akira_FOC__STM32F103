#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "micro_timer.h"

// 與後代 (STM32F446RE_TEST) 的 lowpass_filter.h 相同，只把函式都改成 static inline

/**
 * @brief 低通濾波器結構體
 * 包含了濾波器運作所需的狀態變數
 */
typedef struct LowPassFilter
{
    float Tf;                     //!< 低通濾波器時間常數 (秒)
    unsigned long timestamp_prev; //!< 上次執行時的時間戳 (微秒)
    float y_prev;                 //!< 上次濾波後的輸出值
} LowPassFilter;


/**
 * @brief 初始化低通濾波器
 * @param filter 指向 LowPassFilter 結構體的指標
 * @param Tf 低通濾波器時間常數 (秒)
 */
static inline void LowPassFilter_Init(LowPassFilter* filter, float Tf) {
    filter->Tf = Tf;
    filter->y_prev = 0.0f; // 初始化上一個濾波值為0
    filter->timestamp_prev = (unsigned long)micros(); // 初始化上次時間戳為當前時間
}

/**
 * @brief 更新低通濾波器的值
 * @param filter 指向 LowPassFilter 結構體的指標
 * @param x 當前輸入值
 * @return 濾波後的輸出值
 */
static inline float LowPassFilter_Update(LowPassFilter* filter, float x) {
    unsigned long timestamp_curr = (unsigned long)micros();
    // 計算時間差，並轉換為秒
    float dt = (float)(timestamp_curr - filter->timestamp_prev) * 1e-6f;

    // 處理時間戳溢位或初始過大/過小的情況
    // 如果 dt 小於 0 (時間戳溢位), 則給予一個預設值，例如 1ms (1e-3f 秒)
    if (dt < 0.0f) {
        dt = 1e-3f;
    }
    // 如果 dt 大於 0.3 秒，則認為是長時間中斷，將濾波器重置為當前輸入值
    else if (dt > 0.3f) {
        filter->y_prev = x;
        filter->timestamp_prev = timestamp_curr;
        return x;
    }

    // 一階低通：alpha = Tf / (Tf + dt)，y = alpha * y_prev + (1 - alpha) * x，Tf 越大濾得越多
    float alpha_factor = filter->Tf / (filter->Tf + dt);
    float y_curr = alpha_factor * filter->y_prev + (1.0f - alpha_factor) * x;

    // 更新狀態變數
    filter->y_prev = y_curr;
    filter->timestamp_prev = timestamp_curr;

    return y_curr;
}

#endif // LOWPASS_FILTER_H
