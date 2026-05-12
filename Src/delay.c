/*
 * delay.c
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */
#include "delay.h"
#include "main.h"

void AX_DELAY_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void AX_Delayus(uint16_t us)
{
    uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

void AX_Delayms(uint16_t ms)
{
    HAL_Delay(ms);
}
