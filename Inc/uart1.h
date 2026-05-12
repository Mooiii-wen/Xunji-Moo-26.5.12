#include "uart1.h"
#include "usart.h"
#include <stdio.h>

void AX_UART1_Init(uint32_t baud)
{
    (void)baud;
    /*
       USART1 初始化交给 CubeMX 生成的 MX_USART1_UART_Init()
       不要在这里重复初始化。
    */
}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

int fgetc(FILE *f)
{
    uint8_t ch;
    HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY);
    return ch;
}
