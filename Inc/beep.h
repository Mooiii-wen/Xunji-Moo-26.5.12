/*
 * beep.h
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_BEEP_H
#define __AX_BEEP_H

#include "main.h"

void AX_BEEP_Init(void);

#define AX_BEEP_On()      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET)
#define AX_BEEP_Off()     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET)
#define AX_BEEP_Toggle()  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0)

#endif
