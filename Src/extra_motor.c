#include "extra_motor.h"
#include "tim.h"
#include "main.h"

#define EXTRA_MOTOR_MAX 3600

static int16_t LimitExtraMotorSpeed(int16_t speed)
{
    if (speed > EXTRA_MOTOR_MAX)
        return EXTRA_MOTOR_MAX;

    if (speed < -EXTRA_MOTOR_MAX)
        return -EXTRA_MOTOR_MAX;

    return speed;
}

static void ExtraMotor_Set(TIM_HandleTypeDef *htim,
                           uint32_t channel,
                           GPIO_TypeDef *in1_port,
                           uint16_t in1_pin,
                           GPIO_TypeDef *in2_port,
                           uint16_t in2_pin,
                           int16_t speed)
{
    uint32_t arr;
    uint32_t duty;

    speed = LimitExtraMotorSpeed(speed);

    arr = __HAL_TIM_GET_AUTORELOAD(htim);
    duty = ((speed >= 0 ? speed : -speed) * arr) / EXTRA_MOTOR_MAX;

    if (speed > 0)
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(htim, channel, duty);
}

void ExtraMotor_Init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

    Winch_Main_SetSpeed(0);
    Winch_Pair_SetSpeed(0);
}

void Winch_Main_SetSpeed(int16_t speed)
{
    ExtraMotor_Set(&htim8,
                   TIM_CHANNEL_1,
                   GPIOC, GPIO_PIN_10,
                   GPIOC, GPIO_PIN_11,
                   speed);
}

void Winch_Pair_SetSpeed(int16_t speed)
{
    ExtraMotor_Set(&htim8,
                   TIM_CHANNEL_2,
                   GPIOC, GPIO_PIN_12,
                   GPIOD, GPIO_PIN_2,
                   speed);

    ExtraMotor_Set(&htim8,
                   TIM_CHANNEL_3,
                   GPIOA, GPIO_PIN_4,
                   GPIOA, GPIO_PIN_5,
                   -speed);
}
