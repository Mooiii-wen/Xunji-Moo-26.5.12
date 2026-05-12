/*
 * led.h
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */


#ifndef __AX_LED_H
#define __AX_LED_H

#include "main.h"

void AX_LED_Init(void);

#define AX_LED_Red_On()        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define AX_LED_Red_Off()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define AX_LED_Red_Toggle()    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0)

#define AX_LED_Green_On()      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define AX_LED_Green_Off()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define AX_LED_Green_Toggle()  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1)

#endif
