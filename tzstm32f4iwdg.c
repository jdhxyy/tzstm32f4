// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// �ڲ����Ź�����
// Authors: jdh99 <jdh821@163.com>

#include "tziwdg.h"
#include "stm32f4xx.h"
#include "stm32f10x_iwdg.h"

// initRegister ��ʼ���ڲ����Ź�
// prescaler:��Ƶϵ��
// #define IWDG_Prescaler_4            ((uint8_t)0x00)
// #define IWDG_Prescaler_8            ((uint8_t)0x01)
// #define IWDG_Prescaler_16           ((uint8_t)0x02)
// #define IWDG_Prescaler_32           ((uint8_t)0x03)
// #define IWDG_Prescaler_64           ((uint8_t)0x04)
// #define IWDG_Prescaler_128          ((uint8_t)0x05)
// #define IWDG_Prescaler_256          ((uint8_t)0x06)
// reload:���ؼ���ֵ
static void initRegister(uint8_t prescaler,uint16_t reload);

// TZIWdgInit ��ʼ���ڲ����Ź�
// timeout��ʱʱ��.��λ:s
// ע��:��ʱʱ�䲻����Ϊ0,�������Ϊ0,���ڲ����Ź�ʧЧ
void TZIWdgInit(int timeout) {
    if (timeout == 0) {
        return;
    }

    // 40K�ڲ�ʱ��
   	initRegister(IWDG_Prescaler_256, timeout * 40000 / 256);	
    IWDG_Enable();
}

// initRegister ��ʼ���ڲ����Ź�
// prescaler:��Ƶϵ��
// #define IWDG_Prescaler_4            ((uint8_t)0x00)
// #define IWDG_Prescaler_8            ((uint8_t)0x01)
// #define IWDG_Prescaler_16           ((uint8_t)0x02)
// #define IWDG_Prescaler_32           ((uint8_t)0x03)
// #define IWDG_Prescaler_64           ((uint8_t)0x04)
// #define IWDG_Prescaler_128          ((uint8_t)0x05)
// #define IWDG_Prescaler_256          ((uint8_t)0x06)
// reload:���ؼ���ֵ
static void initRegister(uint8_t prescaler,uint16_t reload) {
   	// ����֮ǰҪ����ʹ�ܼĴ���д
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 					
	IWDG_SetPrescaler(prescaler);						
	IWDG_SetReload(reload);										
	IWDG_ReloadCounter();										
}

// TZIWdgFeed ι��
void TZIWdgFeed(void) {
    IWDG_ReloadCounter();
}
