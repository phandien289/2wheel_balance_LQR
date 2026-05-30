/*
 * param.h
 *
 *  Created on: Apr 10, 2026
 *      Author: admin
 */

#ifndef INC_PARAM_H_
#define INC_PARAM_H_

#include <stdint.h>

#define PI                 3.1415926f

#define DEG_TO_RAD 			0.01745329252f
#define MOTOR_SPEED_MAX    3599
#define SCALE_V_TO_PWM   	(MOTOR_SPEED_MAX/12)

#define ENCODER_PPR        11       // xung / vòng / kênh
#define QUAD_FACTOR        4         // TI12: x4
#define GEAR_RATIO         30.0f
#define TICKS_PER_REV      (ENCODER_PPR * QUAD_FACTOR * GEAR_RATIO) //

#define WHEEL_DIAMETER_M   0.06f     // 6 cm
#define WHEEL_CIRC_M       (PI * WHEEL_DIAMETER_M)
#define WHEEL_BASE_M   0.292f

#define DT_TIME 0.005f
#define MOTOR_DEADBAND 100
extern int32_t g_encCount1;
extern int32_t g_encCount2;

extern float g_pos_1;
extern float g_pos_2;

extern float g_speed1_mps;
extern float g_speed2_mps;





#endif /* INC_PARAM_H_ */
