#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "micro_timer.h" 


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
void LowPassFilter_Init(LowPassFilter* filter, float Tf) {
    filter->Tf = Tf;
    filter->y_prev = 0.0f; // 初始化上一個濾波值為0
    filter->timestamp_prev = micros(); // 初始化上次時間戳為當前時間
};

/**
 * @brief 更新低通濾波器的值
 * @param filter 指向 LowPassFilter 結構體的指標
 * @param x 當前輸入值
 * @return 濾波後的輸出值
 */
float LowPassFilter_Update(LowPassFilter* filter, float x) {
    unsigned long timestamp_curr = micros();
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

    // 計算濾波係數 alpha
    // 您的原始公式是 alpha = Tf / (Tf + dt)，這是一個 IIR 濾波器公式的變體
    // 另一種常見的寫法是 y = (1-alpha)*y_prev + alpha*x
    // 如果使用 y = alpha*y_prev + (1-alpha)*x，那麼 alpha = Tf/(Tf+dt) 是對的
    // 但更常見的數位濾波器公式是 y(k) = y(k-1) + (dt / (Tf + dt)) * (x(k) - y(k-1))
    // 也就是 y(k) = (1 - dt / (Tf + dt)) * y(k-1) + (dt / (Tf + dt)) * x(k)
    // 讓我們依據您原始的 alpha 計算方式來調整：
    // y = alpha * y_prev + (1.0f - alpha) * x;
    // 如果您的 alpha 定義為 Tf/(Tf+dt)，那麼它應該是 (1 - dt/(Tf+dt))。
    // 所以，您原始程式碼的公式 alpha*y_prev + (1.0f - alpha)*x; 是正確的。
    float alpha_factor = filter->Tf / (filter->Tf + dt);
    float y_curr = alpha_factor * filter->y_prev + (1.0f - alpha_factor) * x;

    // 更新狀態變數
    filter->y_prev = y_curr;
    filter->timestamp_prev = timestamp_curr;

    return y_curr;
};

#endif // LOWPASS_FILTER_H