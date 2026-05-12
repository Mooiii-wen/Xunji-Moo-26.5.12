/*
 * servo.h
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */

#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx.h"

void Servo_Init(void);

void YawServo_SetAngle(uint8_t angle);
void ReleaseServo_SetAngle(uint8_t angle);

#endif
