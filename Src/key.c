/*
 * key.c
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */

#include "key.h"

void AX_KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

uint8_t AX_KEY_Scan(void)
{
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        return 1;
    }

    return 0;
}
