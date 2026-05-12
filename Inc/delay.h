/*
 * delay.h
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_DELAY_H
#define __AX_DELAY_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//OpenCTR 接口函数
void AX_DELAY_Init(void); //延时初始化
void AX_Delayus(uint16_t us);  //软件微妙延时
void AX_Delayms(uint16_t ms);  //软件毫秒延时

#endif

/******************* (C) 版权 2022 XTARK **************************************/
