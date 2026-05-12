/*
 * line.h
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_LINE_H
#define __AX_LINE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//接口函数
void AX_LINE_Init(void);  //4路巡线传感器 初始化
uint8_t AX_LINE_GetData(void);  //4路巡线 获取一次检测数据

#endif

/* INC_LINE_H_ */
