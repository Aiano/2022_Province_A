//
// Created by Aiano on 2022/7/22.
//

#include "servo.h"

void servo_init() {
    HAL_TIM_PWM_Start(SERVO1_TIM_HANDLE, SERVO1_TIM_CHANNEL);
    HAL_TIM_PWM_Start(SERVO2_TIM_HANDLE, SERVO2_TIM_CHANNEL);
}

void servo_set_degree(uint16_t servo1_degree, uint16_t servo2_degree) {
    if (servo1_degree > SERVO1_MAX_DEGREE) servo1_degree = SERVO1_MAX_DEGREE;
    if (servo2_degree > SERVO2_MAX_DEGREE) servo2_degree = SERVO2_MAX_DEGREE;
    if (servo1_degree < 0) servo1_degree = 0;
    if (servo2_degree < 0) servo2_degree = 0;

    static uint32_t servo1_compare, servo2_compare;
    servo1_compare = SERVO_DELTA_ARR * servo1_degree / SERVO1_MAX_DEGREE + SERVO_MIN_ARR;
    servo2_compare = SERVO_DELTA_ARR * servo2_degree / SERVO2_MAX_DEGREE + SERVO_MIN_ARR;
    __HAL_TIM_SET_COMPARE(SERVO1_TIM_HANDLE, SERVO1_TIM_CHANNEL, servo1_compare);
    __HAL_TIM_SET_COMPARE(SERVO2_TIM_HANDLE, SERVO2_TIM_CHANNEL, servo2_compare);
}


