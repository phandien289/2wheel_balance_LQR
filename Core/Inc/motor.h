/*
 * motor.h
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>        // <--- thêm dòng này
#include "stm32f1xx_hal.h"


typedef enum {
    MOTOR_1 = 0,    //bánh bên phải
    MOTOR_2         //bánh bên trái
} MotorId;

void Motor_Init(void);
void Motor_SetSpeed(MotorId id, int16_t speed);

#endif /* INC_MOTOR_H_ */
