/*
 * line.c
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */
/*
 * line.c
 *
 * 功能：
 *   1. 初始化四路红外循迹传感器输入引脚；
 *   2. 读取四路传感器状态；
 *   3. 将四路状态组合成一个 0~15 的数值返回。
 *
 * 对应模块：
 *   四路红外巡线模块。
 *
 * 接线方案：
 *   L1 -> PC2，最左侧循迹传感器；
 *   L2 -> PC3，左中循迹传感器；
 *   L3 -> PC4，右中循迹传感器；
 *   L4 -> PC5，最右侧循迹传感器。
 *
 * 逻辑说明：
 *   默认认为：
 *      黑线 = 低电平 GPIO_PIN_RESET；
 *      白底 = 高电平 GPIO_PIN_SET。
 *
 * 如果你们的循迹模块逻辑相反，
 * 只需要把 GPIO_PIN_RESET 改成 GPIO_PIN_SET。
 */

#include "line.h"


/*
 * 函数名：AX_LINE_Init
 *
 * 功能：
 *   初始化四路循迹传感器的 GPIO。
 *
 * 参数：
 *   无。
 *
 * 返回值：
 *   无。
 *
 * 对应模块：
 *   四路红外循迹模块。
 *
 * 使用引脚：
 *   PC2、PC3、PC4、PC5。
 *
 * GPIO 模式：
 *   GPIO_MODE_INPUT：
 *      普通输入模式。
 *
 *   GPIO_PULLUP：
 *      开启内部上拉。
 *      这样在传感器没有主动拉低时，引脚保持高电平。
 *
 * 调用位置：
 *   main.c 中的 APP_Init()。
 */
void AX_LINE_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*
     * 使用 GPIOC，所以要先打开 GPIOC 时钟。
     */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*
     * 一次性配置 PC2、PC3、PC4、PC5 四个输入引脚。
     */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}


/*
 * 函数名：AX_LINE_GetData
 *
 * 功能：
 *   读取四路循迹传感器状态，并组合成一个 8 位数据返回。
 *
 * 参数：
 *   无。
 *
 * 返回值：
 *   uint8_t 类型，范围 0x00 ~ 0x0F。
 *
 * 返回值各位含义：
 *   bit0，也就是 0x01：L1，最左传感器；
 *   bit1，也就是 0x02：L2，左中传感器；
 *   bit2，也就是 0x04：L3，右中传感器；
 *   bit3，也就是 0x08：L4，最右传感器。
 *
 * 举例：
 *   返回 0x00：
 *      四个传感器都没有检测到黑线。
 *
 *   返回 0x06：
 *      L2 和 L3 检测到黑线。
 *      二进制为 0110。
 *      通常表示车身在黑线中间，适合直行。
 *
 *   返回 0x01：
 *      只有 L1 检测到黑线。
 *      表示黑线偏左较多。
 *
 *   返回 0x08：
 *      只有 L4 检测到黑线。
 *      表示黑线偏右较多。
 *
 *   返回 0x0F：
 *      四个传感器都检测到黑线。
 *      通常可以认为到达路口、停止线或任务区域。
 */
uint8_t AX_LINE_GetData(void)
{
    uint8_t re = 0;

    /*
     * L1：PC2。
     * 如果 PC2 为低电平，认为 L1 检测到黑线。
     */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
        re |= 0x01;
    }

    /*
     * L2：PC3。
     */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET)
    {
        re |= 0x02;
    }

    /*
     * L3：PC4。
     */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET)
    {
        re |= 0x04;
    }

    /*
     * L4：PC5。
     */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
        re |= 0x08;
    }

    return re;
}
