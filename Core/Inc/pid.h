/*
 * pid.h
 *
 *  Created on: May 19, 2026
 *      Author: admin
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

// Giới hạn giá trị trong khoảng [low, high]
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// Cấu trúc PID
typedef struct {
    // Tham số
    float P;
    float I;
    float D;
    float output_ramp;  // Giới hạn tốc độ thay đổi output [đơn vị/giây]
    float limit;        // Giới hạn output tối đa

    // Trạng thái nội bộ
    float error_prev;
    float output_prev;
    float integral_prev;
    uint32_t timestamp_prev;
} PIDController;

// Khởi tạo PID
void PID_Init(PIDController* pid, float P, float I, float D, float ramp, float limit);

// Reset trạng thái
void PID_Reset(PIDController* pid);

// Tính output: truyền vào error, trả về output
float PID_Update(PIDController* pid, float error);


#endif /* INC_PID_H_ */
