// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// 内部看门狗驱动
// Authors: jdh99 <jdh821@163.com>

#include "tziwdg.h"
#include "stm32f4xx.h"
#include "stm32f10x_iwdg.h"

// initRegister 初始化内部看门狗
// prescaler:分频系数
// #define IWDG_Prescaler_4            ((uint8_t)0x00)
// #define IWDG_Prescaler_8            ((uint8_t)0x01)
// #define IWDG_Prescaler_16           ((uint8_t)0x02)
// #define IWDG_Prescaler_32           ((uint8_t)0x03)
// #define IWDG_Prescaler_64           ((uint8_t)0x04)
// #define IWDG_Prescaler_128          ((uint8_t)0x05)
// #define IWDG_Prescaler_256          ((uint8_t)0x06)
// reload:重载计数值
static void initRegister(uint8_t prescaler,uint16_t reload);

// TZIWdgInit 初始化内部看门狗
// timeout超时时间.单位:s
// 注意:超时时间不允许为0,如果设置为0,则内部看门狗失效
void TZIWdgInit(int timeout) {
    if (timeout == 0) {
        return;
    }

    // 40K内部时钟
   	initRegister(IWDG_Prescaler_256, timeout * 40000 / 256);	
    IWDG_Enable();
}

// initRegister 初始化内部看门狗
// prescaler:分频系数
// #define IWDG_Prescaler_4            ((uint8_t)0x00)
// #define IWDG_Prescaler_8            ((uint8_t)0x01)
// #define IWDG_Prescaler_16           ((uint8_t)0x02)
// #define IWDG_Prescaler_32           ((uint8_t)0x03)
// #define IWDG_Prescaler_64           ((uint8_t)0x04)
// #define IWDG_Prescaler_128          ((uint8_t)0x05)
// #define IWDG_Prescaler_256          ((uint8_t)0x06)
// reload:重载计数值
static void initRegister(uint8_t prescaler,uint16_t reload) {
   	// 访问之前要首先使能寄存器写
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 					
	IWDG_SetPrescaler(prescaler);						
	IWDG_SetReload(reload);										
	IWDG_ReloadCounter();										
}

// TZIWdgFeed 喂狗
void TZIWdgFeed(void) {
    IWDG_ReloadCounter();
}
