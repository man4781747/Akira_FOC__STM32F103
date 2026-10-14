#ifndef PID_CTRL_H
#define PID_CTRL_H

#include <stdint.h> // For uint32_t (for timestamp_prev)
#include "micro_timer.h"

// 與後代 (STM32F446RE_TEST) 的 PID_Ctrl.h 相同，把函式都改成 static inline，另加 PIDController_runTs()

static inline float constrain(float amt, float low, float high) {
    if (amt < low) return low;
    if (amt > high) return high;
    return amt;
}
/**
 * @brief PID controller structure
 *
 * @param P - Proportional gain
 * @param I - Integral gain
 * @param D - Derivative gain
 * @param output_ramp - Maximum speed of change of the output value
 * @param limit - Maximum output value
 */
typedef struct {
    float P;              //!< Proportional gain
    float I;              //!< Integral gain
    float D;              //!< Derivative gain
    float output_ramp;    //!< Maximum speed of change of the output value
    float limit;          //!< Maximum output value

    // Internal state variables
    float error_prev;     //!< last tracking error value
    float output_prev;    //!< last pid output value
    float integral_prev;  //!< last integral component value
    uint32_t timestamp_prev; //!< Last execution timestamp (using uint32_t for micros() return type)
} PIDController;

static inline void PIDController_init(PIDController* ctrl, float P_val, float I_val, float D_val, float ramp_val, float limit_val) {
    ctrl->P = P_val;
    ctrl->I = I_val;
    ctrl->D = D_val;
    ctrl->output_ramp = ramp_val; // output derivative limit [volts/second]
    ctrl->limit = limit_val;      // output supply limit   [volts]

    ctrl->error_prev = 0.0f;
    ctrl->output_prev = 0.0f;
    ctrl->integral_prev = 0.0f;
    ctrl->timestamp_prev = (uint32_t)micros(); // Initialize with current time
}

/**
 * @brief Processes the PID control logic with a new error value.
 *
 * @param ctrl A pointer to the PIDController structure.
 * @param error The current tracking error.
 * @param anti_windup 非 0 時，輸出頂到 limit 而且誤差還在把它往外推的那一次不累加積分。
 * @return The calculated PID output value.
 */
static inline float PIDController_run(PIDController* ctrl, float error, int anti_windup) {
    uint32_t timestamp_now = (uint32_t)micros();
    uint32_t dt = timestamp_now - ctrl->timestamp_prev; // 處理 unsigned 差值自動溢位
    float Ts = (float)dt * 1e-6f;

    if (Ts <= 0.0f || Ts > 0.5f) {
        Ts = 1e-3f;
    }

    /* 只做一次除法，使用 invTs 重複使用 */
    float invTs = 1.0f / Ts;
    float halfTs = 0.5f * Ts;

    /* 將參數與狀態先複製到暫存變數以減少結構存取次數 */
    float P = ctrl->P;
    float I = ctrl->I;
    float D = ctrl->D;
    float limit = ctrl->limit;
    float output_prev = ctrl->output_prev;
    float error_prev = ctrl->error_prev;
    float integral_prev = ctrl->integral_prev;
    float output_ramp = ctrl->output_ramp;

    /* 比例項 */
    float proportional = P * error;

    /* 積分項 (Tustin) */
    float integral = integral_prev + I * halfTs * (error + error_prev);

    /* anti-windup (inline) */
    if (integral > limit) integral = limit;
    else if (integral < -limit) integral = -limit;

    /* 微分項，使用 invTs 減少一次除法 */
    float derivative = D * (error - error_prev) * invTs;

    /* 合成輸出並限制 (inline) */
    float output = proportional + integral + derivative;
    if (output > limit) {
        if (anti_windup && error > 0.0f) integral = integral_prev;
        output = limit;
    } else if (output < -limit) {
        if (anti_windup && error < 0.0f) integral = integral_prev;
        output = -limit;
    }

    /* 輸出斜率限制（若啟用） - 使用 invTs 避免二次除法 */
    if (output_ramp > 0.0f) {
        float output_rate = (output - output_prev) * invTs; // [unit/sec]
        if (output_rate > output_ramp) {
            output = output_prev + output_ramp * Ts;
        } else if (output_rate < -output_ramp) {
            output = output_prev - output_ramp * Ts;
        }
    }

    /* 儲存狀態 */
    ctrl->integral_prev = integral;
    ctrl->output_prev = output;
    ctrl->error_prev = error;
    ctrl->timestamp_prev = timestamp_now;

    return output;
}

static inline float PIDController_process(PIDController* ctrl, float error) {
    return PIDController_run(ctrl, error, 0);
}

/**
 * @brief 這一代 (F103) 加的：和 PIDController_run() 相同的計算，但取樣時間由呼叫者給 (固定週期的 FOC 中斷用)。
 *        沒有 FPU，micros() 與每次的除法太花時間。D = 0 時不算微分項，output_ramp = 0 時不做斜率限制
 *
 * @param Ts 取樣時間 (秒)
 * @param invTs 1 / Ts
 */
static inline float PIDController_runTs(PIDController* ctrl, float error, float Ts, float invTs, int anti_windup) {
    float limit = ctrl->limit;
    float error_prev = ctrl->error_prev;
    float integral_prev = ctrl->integral_prev;

    /* 積分項 (Tustin) */
    float integral = integral_prev + ctrl->I * (0.5f * Ts) * (error + error_prev);
    if (integral > limit) integral = limit;
    else if (integral < -limit) integral = -limit;

    float output = ctrl->P * error + integral;
    if (ctrl->D != 0.0f) {
        output += ctrl->D * (error - error_prev) * invTs;
    }
    if (output > limit) {
        if (anti_windup && error > 0.0f) integral = integral_prev;
        output = limit;
    } else if (output < -limit) {
        if (anti_windup && error < 0.0f) integral = integral_prev;
        output = -limit;
    }

    float output_ramp = ctrl->output_ramp;
    if (output_ramp > 0.0f) {
        float output_prev = ctrl->output_prev;
        float output_rate = (output - output_prev) * invTs;
        if (output_rate > output_ramp) {
            output = output_prev + output_ramp * Ts;
        } else if (output_rate < -output_ramp) {
            output = output_prev - output_ramp * Ts;
        }
    }

    ctrl->integral_prev = integral;
    ctrl->output_prev = output;
    ctrl->error_prev = error;
    return output;
}

/**
 * @brief Resets the internal state of the PID controller (integral, previous error, previous output).
 */
static inline void PIDController_reset(PIDController* ctrl) {
    ctrl->integral_prev = 0.0f;
    ctrl->output_prev = 0.0f;
    ctrl->error_prev = 0.0f;
}

#endif // PID_CTRL_H
