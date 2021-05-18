// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4��io����
// Authors: jdh99 <jdh821@163.com>

#ifndef TZSTM32F4IO_H
#define TZSTM32F4IO_H

#include <stdint.h>

// MCU��io��
typedef enum {
    STM32F4_PA0 = 0, STM32F4_PA1, STM32F4_PA2, STM32F4_PA3, STM32F4_PA4, STM32F4_PA5, STM32F4_PA6, STM32F4_PA7,
    STM32F4_PA8, STM32F4_PA9, STM32F4_PA10, STM32F4_PA11, STM32F4_PA12, STM32F4_PA13, STM32F4_PA14, STM32F4_PA15,
    STM32F4_PB0, STM32F4_PB1, STM32F4_PB2, STM32F4_PB3, STM32F4_PB4, STM32F4_PB5, STM32F4_PB6, STM32F4_PB7, 
    STM32F4_PB8, STM32F4_PB9, STM32F4_PB10, STM32F4_PB11, STM32F4_PB12, STM32F4_PB13, STM32F4_PB14, STM32F4_PB15,
    STM32F4_PC0, STM32F4_PC1, STM32F4_PC2, STM32F4_PC3, STM32F4_PC4, STM32F4_PC5, STM32F4_PC6, STM32F4_PC7,
    STM32F4_PC8, STM32F4_PC9, STM32F4_PC10, STM32F4_PC11, STM32F4_PC12, STM32F4_PC13, STM32F4_PC14, STM32F4_PC15,
    STM32F4_PD0, STM32F4_PD1, STM32F4_PD2, STM32F4_PD3, STM32F4_PD4, STM32F4_PD5, STM32F4_PD6, STM32F4_PD7,
    STM32F4_PD8, STM32F4_PD9, STM32F4_PD10, STM32F4_PD11, STM32F4_PD12, STM32F4_PD13, STM32F4_PD14, STM32F4_PD15,
    STM32F4_PE0, STM32F4_PE1, STM32F4_PE2, STM32F4_PE3, STM32F4_PE4, STM32F4_PE5, STM32F4_PE6, STM32F4_PE7,
    STM32F4_PE8, STM32F4_PE9, STM32F4_PE10, STM32F4_PE11, STM32F4_PE12, STM32F4_PE13, STM32F4_PE14, STM32F4_PE15,
    STM32F4_PF0, STM32F4_PF1, STM32F4_PF2, STM32F4_PF3, STM32F4_PF4, STM32F4_PF5, STM32F4_PF6, STM32F4_PF7,
    STM32F4_PF8, STM32F4_PF9, STM32F4_PF10, STM32F4_PF11, STM32F4_PF12, STM32F4_PF13, STM32F4_PF14, STM32F4_PF15,
    STM32F4_PG0, STM32F4_PG1, STM32F4_PG2, STM32F4_PG3, STM32F4_PG4, STM32F4_PG5, STM32F4_PG6, STM32F4_PG7,
    STM32F4_PG8, STM32F4_PG9, STM32F4_PG10, STM32F4_PG11, STM32F4_PG12, STM32F4_PG13, STM32F4_PG14, STM32F4_PG15,
    STM32F4_PH0, STM32F4_PH1, STM32F4_PH2, STM32F4_PH3, STM32F4_PH4, STM32F4_PH5, STM32F4_PH6, STM32F4_PH7,
    STM32F4_PH8, STM32F4_PH9, STM32F4_PH10, STM32F4_PH11, STM32F4_PH12, STM32F4_PH13, STM32F4_PH14, STM32F4_PH15,
    STM32F4_PI0, STM32F4_PI1, STM32F4_PI2, STM32F4_PI3, STM32F4_PI4, STM32F4_PI5, STM32F4_PI6, STM32F4_PI7,
    STM32F4_PI8, STM32F4_PI9, STM32F4_PI10, STM32F4_PI11, STM32F4_PI12, STM32F4_PI13, STM32F4_PI14, STM32F4_PI15,
    STM32F4_INVALID_PIN = 0xFFFF
} Stm32f4IO;

#endif
