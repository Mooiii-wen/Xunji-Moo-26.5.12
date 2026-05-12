/*
 * motor.c
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */
/*
 * motor.c
 *
 * 功能：
 *   1. 初始化四个底盘电机 PWM 输出；
 *   2. 提供 A/B/C/D 四个电机的速度控制函数；
 *   3. 每个电机使用“两路 PWM”控制正反转。
 *
 * 适用模块：
 *   四个麦克纳姆轮底盘电机。
 *
 * 电机控制方式：
 *   一个电机占用两个 PWM 通道。
 *
 *   speed > 0：
 *      通道1输出 PWM，通道2输出 0，占空比由 speed 决定，电机正转。
 *
 *   speed < 0：
 *      通道1输出 0，通道2输出 PWM，占空比由 -speed 决定，电机反转。
 *
 *   speed = 0：
 *      两个通道都输出 0，电机停止。
 */

#include "motor.h"
#include "tim.h"

/*
 * MOTOR_SPEED_MAX：
 *   软件层面的最大速度值。
 *
 * 参数范围：
 *   speed = -3600 ~ 3600
 *
 * 注意：
 *   这里的 3600 不是实际转速 rpm，
 *   只是程序里用于表示 PWM 占空比大小的控制量。
 */
#define MOTOR_SPEED_MAX 3600


/*
 * 函数名：Motor_Limit
 *
 * 功能：
 *   对传入的电机速度值进行限幅，防止超过允许范围。
 *
 * 参数：
 *   speed：
 *      需要限制的速度值。
 *      正数表示正转；
 *      负数表示反转；
 *      0 表示停止。
 *
 * 返回值：
 *   被限制在 -3600 ~ 3600 范围内的速度值。
 *
 * 对应模块：
 *   所有底盘电机。
 */
static int16_t Motor_Limit(int16_t speed)
{
    if (speed > MOTOR_SPEED_MAX)
    {
        return MOTOR_SPEED_MAX;
    }

    if (speed < -MOTOR_SPEED_MAX)
    {
        return -MOTOR_SPEED_MAX;
    }

    return speed;
}


/*
 * 函数名：Motor_SetTwoPWM
 *
 * 功能：
 *   使用两个 PWM 通道控制一个直流电机的正反转和速度。
 *
 * 参数：
 *   htim：
 *      定时器句柄，例如 &htim1、&htim9、&htim12。
 *
 *   ch1：
 *      电机控制通道 1。
 *
 *   ch2：
 *      电机控制通道 2。
 *
 *   speed：
 *      电机速度控制量。
 *      范围：-3600 ~ 3600。
 *      正数：正转。
 *      负数：反转。
 *      0：停止。
 *
 * 工作原理：
 *   先读取当前定时器的 ARR，也就是自动重装载值。
 *   然后根据 speed 计算 PWM 占空比。
 *
 *   举例：
 *      speed = 1800，最大值是 3600，
 *      表示输出大约 50% 占空比。
 *
 * 对应模块：
 *   底盘电机 A/B/C/D 的底层控制。
 */
static void Motor_SetTwoPWM(TIM_HandleTypeDef *htim,
                            uint32_t ch1,
                            uint32_t ch2,
                            int16_t speed)
{
    uint32_t arr;
    uint32_t duty;

    speed = Motor_Limit(speed);

    /*
     * 获取定时器的自动重装载值 ARR。
     * CubeMX 中如果 Period = 8399，那么 ARR = 8399。
     */
    arr = __HAL_TIM_GET_AUTORELOAD(htim);

    /*
     * 根据 speed 计算占空比。
     *
     * speed 绝对值越大，duty 越大，电机转得越快。
     */
    if (speed >= 0)
    {
        duty = ((uint32_t)speed * arr) / MOTOR_SPEED_MAX;
    }
    else
    {
        duty = ((uint32_t)(-speed) * arr) / MOTOR_SPEED_MAX;
    }

    if (speed > 0)
    {
        /*
         * 正转：
         *   通道1输出 PWM；
         *   通道2关闭。
         */
        __HAL_TIM_SET_COMPARE(htim, ch1, duty);
        __HAL_TIM_SET_COMPARE(htim, ch2, 0);
    }
    else if (speed < 0)
    {
        /*
         * 反转：
         *   通道1关闭；
         *   通道2输出 PWM。
         */
        __HAL_TIM_SET_COMPARE(htim, ch1, 0);
        __HAL_TIM_SET_COMPARE(htim, ch2, duty);
    }
    else
    {
        /*
         * 停止：
         *   两个通道都关闭。
         */
        __HAL_TIM_SET_COMPARE(htim, ch1, 0);
        __HAL_TIM_SET_COMPARE(htim, ch2, 0);
    }
}


/*
 * 函数名：AX_MOTOR_Init
 *
 * 功能：
 *   启动四个底盘电机对应的 PWM 通道。
 *
 * 参数：
 *   无。
 *
 * 返回值：
 *   无。
 *
 * 对应模块：
 *   底盘四个麦克纳姆轮电机。
 *
 * 使用的定时器：
 *   TIM1：
 *      电机 A、B。
 *
 *   TIM9：
 *      电机 C。
 *
 *   TIM12：
 *      电机 D。
 *
 * 调用位置：
 *   在 main.c 的 APP_Init() 中调用。
 */
void AX_MOTOR_Init(void)
{
    /*
     * 启动电机 A 和电机 B 的 PWM。
     * 电机 A：TIM1_CH1 / TIM1_CH2。
     * 电机 B：TIM1_CH3 / TIM1_CH4。
     */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /*
     * 启动电机 C 的 PWM。
     * 电机 C：TIM9_CH1 / TIM9_CH2。
     */
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    /*
     * 启动电机 D 的 PWM。
     * 电机 D：TIM12_CH1 / TIM12_CH2。
     */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    /*
     * 初始化后先让全部电机停止，防止上电瞬间乱转。
     */
    AX_MOTOR_A_SetSpeed(0);
    AX_MOTOR_B_SetSpeed(0);
    AX_MOTOR_C_SetSpeed(0);
    AX_MOTOR_D_SetSpeed(0);
}


/*
 * 函数名：AX_MOTOR_A_SetSpeed
 *
 * 功能：
 *   控制电机 A 的速度和方向。
 *
 * 参数：
 *   speed：
 *      -3600 ~ 3600。
 *      正数：正转。
 *      负数：反转。
 *      0：停止。
 *
 * 对应模块：
 *   麦克纳姆底盘左前轮。
 */
void AX_MOTOR_A_SetSpeed(int16_t speed)
{
    Motor_SetTwoPWM(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, speed);
}


/*
 * 函数名：AX_MOTOR_B_SetSpeed
 *
 * 功能：
 *   控制电机 B 的速度和方向。
 *
 * 参数：
 *   speed：
 *      -3600 ~ 3600。
 *
 * 对应模块：
 *   麦克纳姆底盘右前轮。
 */
void AX_MOTOR_B_SetSpeed(int16_t speed)
{
    Motor_SetTwoPWM(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, speed);
}


/*
 * 函数名：AX_MOTOR_C_SetSpeed
 *
 * 功能：
 *   控制电机 C 的速度和方向。
 *
 * 参数：
 *   speed：
 *      -3600 ~ 3600。
 *
 * 对应模块：
 *   麦克纳姆底盘左后轮。
 */
void AX_MOTOR_C_SetSpeed(int16_t speed)
{
    Motor_SetTwoPWM(&htim9, TIM_CHANNEL_1, TIM_CHANNEL_2, speed);
}


/*
 * 函数名：AX_MOTOR_D_SetSpeed
 *
 * 功能：
 *   控制电机 D 的速度和方向。
 *
 * 参数：
 *   speed：
 *      -3600 ~ 3600。
 *
 * 对应模块：
 *   麦克纳姆底盘右后轮。
 */
void AX_MOTOR_D_SetSpeed(int16_t speed)
{
    Motor_SetTwoPWM(&htim12, TIM_CHANNEL_1, TIM_CHANNEL_2, speed);
}
