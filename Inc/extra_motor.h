/*
 * extra_motor.h
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */

#ifndef __EXTRA_MOTOR_H
#define __EXTRA_MOTOR_H

#include "stm32f4xx.h"

void ExtraMotor_Init(void);

void Winch_Main_SetSpeed(int16_t speed);
void Winch_Pair_SetSpeed(int16_t speed);

#endif /* INC_EXTRA_MOTOR_H_ */
