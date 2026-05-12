/*
 * servo.c
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */

#include "servo.h"
#include "tim.h"

static uint16_t Servo_AngleToPulse(uint8_t angle)
{
    if (angle > 180)
        angle = 180;

    return 500 + angle * 2000 / 180;
}

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

    YawServo_SetAngle(90);
    ReleaseServo_SetAngle(20);
}

void YawServo_SetAngle(uint8_t angle)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, Servo_AngleToPulse(angle));
}

void ReleaseServo_SetAngle(uint8_t angle)
{
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, Servo_AngleToPulse(angle));
}
