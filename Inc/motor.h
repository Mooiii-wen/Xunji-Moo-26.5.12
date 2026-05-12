/*
 * motor.h
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_MOTOR_H
#define __AX_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//接口函数
void AX_MOTOR_Init(void); //电机PWM控制初始化
void AX_MOTOR_A_SetSpeed(int16_t speed);   //电机A控制
void AX_MOTOR_B_SetSpeed(int16_t speed);   //电机B控制
void AX_MOTOR_C_SetSpeed(int16_t speed);   //电机C控制
void AX_MOTOR_D_SetSpeed(int16_t speed);   //电机D控制

#endif

/******************* (C) 版权 2023 XTARK **************************************/
 /* INC_MOTOR_H_ */
