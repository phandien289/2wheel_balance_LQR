/*
 * pid.c
 *
 *  Created on: May 19, 2026
 *      Author: admin
 */

#include "pid.h"

void PID_Init(PIDController* pid, float P, float I, float D,
              float ramp, float limit)
{
    pid->P            = P;
    pid->I            = I;
    pid->D            = D;
    pid->output_ramp  = ramp;
    pid->limit        = limit;
    pid->error_prev   = 0.0f;
    pid->output_prev  = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = HAL_GetTick();
}

void PID_Reset(PIDController* pid)
{
    pid->error_prev    = 0.0f;
    pid->output_prev   = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = HAL_GetTick();
}

float PID_Update(PIDController* pid, float error)
{
    // --- Tính thời gian từ lần gọi trước [giây] ---
    uint32_t timestamp_now = HAL_GetTick();
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-3f;  // ms → giây

    // Xử lý trường hợp bất thường
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

    // --- P: tỉ lệ ---
    float proportional = pid->P * error;

    // --- I: tích phân (Tustin) ---
    float integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);

    // Anti-windup
    integral = _constrain(integral, -pid->limit / 3.0f, pid->limit / 3.0f);

    // --- D: vi phân ---
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    // --- Tổng output ---
    float output = proportional + integral + derivative;

    // Giới hạn output
    output = _constrain(output, -pid->limit, pid->limit);

    // --- Output ramp ---
    if (pid->output_ramp > 0.0f) {
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * Ts;
    }

    // --- Lưu lại cho vòng tiếp theo ---
    pid->integral_prev  = integral;
    pid->output_prev    = output;
    pid->error_prev     = error;
    pid->timestamp_prev = timestamp_now;

    return output;
}
