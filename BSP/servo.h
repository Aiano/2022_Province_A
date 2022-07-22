//
// Created by Aiano on 2022/7/22.
//

#ifndef INC_2022_PROVINCE_A_SERVO_H
#define INC_2022_PROVINCE_A_SERVO_H

#include "main.h"
#include "tim.h"

#define SERVO1_TIM_HANDLE &htim1
#define SERVO1_TIM_CHANNEL TIM_CHANNEL_1
#define SERVO1_MAX_DEGREE 270

#define SERVO2_TIM_HANDLE &htim1
#define SERVO2_TIM_CHANNEL TIM_CHANNEL_2
#define SERVO2_MAX_DEGREE 180

#define SERVO_MIN_ARR 500
#define SERVO_MAX_ARR 2500
#define SERVO_DELTA_ARR 2000

void servo_init();
void servo_set_degree(uint16_t servo1_degree, uint16_t servo2_degree);
#endif //INC_2022_PROVINCE_A_SERVO_H
