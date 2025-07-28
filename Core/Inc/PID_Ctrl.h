#ifndef PID_CTRL_H
#define PID_CTRL_H

#include <stdint.h> // For uint32_t (for timestamp_prev)
#include "micro_timer.h" 
#include "qfplib-m3.h"

float constrain(float amt, float low, float high) {
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

/**
 * @brief Initializes a PIDController instance.
 *
 * @param ctrl A pointer to the PIDController structure to initialize.
 * @param P_val Proportional gain.
 * @param I_val Integral gain.
 * @param D_val Derivative gain.
 * @param ramp_val Maximum speed of change of the output value.
 * @param limit_val Maximum output value.
 */
void PIDController_init(PIDController* ctrl, float P_val, float I_val, float D_val, float ramp_val, float limit_val) {
    ctrl->P = P_val;
    ctrl->I = I_val;
    ctrl->D = D_val;
    ctrl->output_ramp = ramp_val; // output derivative limit [volts/second]
    ctrl->limit = limit_val;      // output supply limit   [volts]

    ctrl->error_prev = 0.0f;
    ctrl->output_prev = 0.0f;
    ctrl->integral_prev = 0.0f;
    ctrl->timestamp_prev = micros(); // Initialize with current time
};

/**
 * @brief Processes the PID control logic with a new error value.
 *
 * This function simulates the operator() overload in C++.
 *
 * @param ctrl A pointer to the PIDController structure.
 * @param error The current tracking error.
 * @return The calculated PID output value.
 */
float PIDController_process(PIDController* ctrl, float error) {
    // calculate the time from the last call
    uint32_t timestamp_now = micros();


    float Ts = qfp_fmul(qfp_fsub(timestamp_now, ctrl->timestamp_prev), 1e-6f);
    // float Ts = (float)(timestamp_now - ctrl->timestamp_prev) * 1e-6f; // Convert microseconds to seconds

    // quick fix for strange cases (micros overflow or large initial dt)
    if (Ts <= 0.0f || Ts > 0.5f) { // Time wrapped around, error, or very large dt
        Ts = 1e-3f; // Default to 1ms, or a more appropriate small value
    }

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p = P *e(k)
    float proportional = qfp_fmul(ctrl->P , error);
    // float proportional = ctrl->P * error;

    // Tustin transform of the integral part
    // u_ik = u_ik_1 + I*Ts/2*(ek + ek_1)
    float integral = qfp_fadd(ctrl->integral_prev, qfp_fmul(qfp_fmul(ctrl->I , Ts),qfp_fmul(0.5f, qfp_fadd(error , ctrl->error_prev))));
    // float integral = ctrl->integral_prev + ctrl->I * Ts * 0.5f * (error + ctrl->error_prev);


    // antiwindup - limit the output
    integral = constrain(integral, -ctrl->limit, ctrl->limit);

    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = qfp_fdiv(qfp_fmul(ctrl->D , qfp_fsub(error , ctrl->error_prev)) , Ts);
    // float derivative = ctrl->D * (error - ctrl->error_prev) / Ts;

    // sum all the components
    float output = qfp_fadd(qfp_fadd(proportional , integral) , derivative);
    // float output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    output = constrain(output, -ctrl->limit, ctrl->limit);

    // if output ramp defined
    if (ctrl->output_ramp > 0.0f) {
        // limit the acceleration by ramping the output
        float output_rate = (output - ctrl->output_prev) / Ts;
        if (output_rate > ctrl->output_ramp) {
            output = ctrl->output_prev + ctrl->output_ramp * Ts;
        } else if (output_rate < -ctrl->output_ramp) {
            output = ctrl->output_prev - ctrl->output_ramp * Ts;
        }
    }
    
    // Saving for the next pass
    ctrl->integral_prev = integral;
    ctrl->output_prev = output;
    ctrl->error_prev = error;
    ctrl->timestamp_prev = timestamp_now;

    return output;
};

/**
 * @brief Resets the internal state of the PID controller (integral, previous error, previous output).
 *
 * @param ctrl A pointer to the PIDController structure.
 */
void PIDController_reset(PIDController* ctrl) {
    ctrl->integral_prev = 0.0f;
    ctrl->output_prev = 0.0f;
    ctrl->error_prev = 0.0f;
};

#endif // PID_CTRL_H