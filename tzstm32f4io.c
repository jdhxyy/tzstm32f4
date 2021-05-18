// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4的io驱动
// Authors: jdh99 <jdh821@163.com>

#include "tzstm32f4io.h"
#include "tzio.h"
#include "stm32f4xx.h"

#include <string.h>

// 最大支持中断的通道数
#define IRQ_CALLBACK_NUM_MAX 16

#pragma pack(1)

typedef struct {
    int pin;
    TZEmptyFunc callback;
    uint8_t irqChannel;
    int priority;
} IrqCallback;

#pragma pack()

// 中断线数组
static IrqCallback gIrqCallback[IRQ_CALLBACK_NUM_MAX] = {0};

static inline void getPort(Stm32f4IO pin, GPIO_TypeDef** port, uint32_t* gpioPin, uint32_t* rcc);
static void getIrqParam(int pin, uint8_t* extiPortSourceGpio, uint8_t* extiPinSource, uint32_t* extiLine, 
    uint8_t* irqChannel);
static void irqHandler(uint8_t bit);

// TZIOConfigOutput 设置为输出
void TZIOConfigOutput(int pin, TZIOPullMode pullMode, TZIOOutMode outMode) {
    TZ_UNUSED(outMode);

    GPIO_TypeDef* port;
    uint32_t gpioPin;
    uint32_t rcc;
    getPort((Stm32f4IO)pin, &port, &gpioPin, &rcc);

    GPIOPuPd_TypeDef pullModeGet;
    switch(pullMode) {
    case TZIO_NOPULL: pullModeGet = GPIO_PuPd_NOPULL; break;
    case TZIO_PULLDOWN: pullModeGet = GPIO_PuPd_DOWN; break;
    case TZIO_PULLUP: pullModeGet = GPIO_PuPd_UP; break;
    }

    GPIOOType_TypeDef outModeGet = (outMode == TZIO_OUT_PP) ? GPIO_OType_PP : GPIO_OType_OD;

    GPIO_InitTypeDef s;
    RCC_AHB1PeriphClockCmd(rcc, ENABLE);
    s.GPIO_Mode = GPIO_Mode_OUT;
    s.GPIO_PuPd = pullModeGet;
    s.GPIO_OType = outModeGet;		
    s.GPIO_Speed = GPIO_Speed_100MHz;
    s.GPIO_Pin = gpioPin;
    GPIO_Init(port, &s);
}

static inline void getPort(Stm32f4IO pin, GPIO_TypeDef** port, uint32_t* gpioPin, uint32_t* rcc) {
    if (pin <= STM32F4_PA15) {
        *port = GPIOA;
        *gpioPin = pin - STM32F4_PA0;
        *rcc = RCC_AHB1Periph_GPIOA;
    } else if (pin <= STM32F4_PB15) {
        *port = GPIOB;
        *gpioPin = pin - STM32F4_PB0;
        *rcc = RCC_AHB1Periph_GPIOB;
    } else if (pin <= STM32F4_PC15) {
        *port = GPIOC;
        *gpioPin = pin - STM32F4_PC0;
        *rcc = RCC_AHB1Periph_GPIOC;
    } else if (pin <= STM32F4_PD15) {
        *port = GPIOD;
        *gpioPin = pin - STM32F4_PD0;
        *rcc = RCC_AHB1Periph_GPIOD;
    } else if (pin <= STM32F4_PE15) {
        *port = GPIOE;
        *gpioPin = pin - STM32F4_PE0;
        *rcc = RCC_AHB1Periph_GPIOE;
    } else if (pin <= STM32F4_PF15) {
        *port = GPIOF;
        *gpioPin = pin - STM32F4_PF0;
        *rcc = RCC_AHB1Periph_GPIOF;
    } else if (pin <= STM32F4_PG15) {
        *port = GPIOG;
        *gpioPin = pin - STM32F4_PG0;
        *rcc = RCC_AHB1Periph_GPIOG;
    } else if (pin <= STM32F4_PH15) {
        *port = GPIOH;
        *gpioPin = pin - STM32F4_PH0;
        *rcc = RCC_AHB1Periph_GPIOH;
    } else if (pin <= STM32F4_PI15) {
        *port = GPIOI;
        *gpioPin = pin - STM32F4_PI0;
        *rcc = RCC_AHB1Periph_GPIOI;
    }
    *gpioPin = (1 << *gpioPin);
}

// TZIOConfigInput 设置为输入
void TZIOConfigInput(int pin, TZIOPullMode pullMode) {
    GPIO_TypeDef* port;
    uint32_t gpioPin;
    uint32_t rcc;
    getPort((Stm32f4IO)pin, &port, &gpioPin, &rcc);

    GPIOPuPd_TypeDef pullModeGet;
    switch(pullMode) {
    case TZIO_NOPULL: pullModeGet = GPIO_PuPd_NOPULL; break;
    case TZIO_PULLDOWN: pullModeGet = GPIO_PuPd_DOWN; break;
    case TZIO_PULLUP: pullModeGet = GPIO_PuPd_UP; break;
    }

    GPIO_InitTypeDef s;
    RCC_AHB1PeriphClockCmd(rcc, ENABLE);
    s.GPIO_Mode = GPIO_Mode_IN;	
    s.GPIO_PuPd = pullModeGet;
    s.GPIO_Pin = gpioPin;
    GPIO_Init(port, &s);
}

// TZIOSetHigh 输出高电平
void TZIOSetHigh(int pin) {
    GPIO_TypeDef* port;
    uint32_t gpioPin;
    uint32_t rcc;
    getPort((Stm32f4IO)pin, &port, &gpioPin, &rcc);
    GPIO_SetBits(port, gpioPin);
}

// TZIOSetLow 输出低电平
void TZIOSetLow(int pin) {
    GPIO_TypeDef* port;
    uint32_t gpioPin;
    uint32_t rcc;
    getPort((Stm32f4IO)pin, &port, &gpioPin, &rcc);
    GPIO_ResetBits(port, gpioPin);
}

// TZIOSet 输出电平
void TZIOSet(int pin, bool level) {
    GPIO_TypeDef* port;
    uint32_t gpioPin;
    uint32_t rcc;
    getPort((Stm32f4IO)pin, &port, &gpioPin, &rcc);
    GPIO_WriteBit(port, gpioPin, (BitAction)level);
}

// TZIOToggle 输出跳变信号
void TZIOToggle(int pin) {
    GPIO_TypeDef* port;
    uint32_t gpioPin;
    uint32_t rcc;
    getPort((Stm32f4IO)pin, &port, &gpioPin, &rcc);
    GPIO_ToggleBits(port, gpioPin);
}

// TZIOReadInputPin 读取输入引脚电平
bool TZIOReadInputPin(int pin) {
    GPIO_TypeDef* port;
    uint32_t gpioPin;
    uint32_t rcc;
    getPort((Stm32f4IO)pin, &port, &gpioPin, &rcc);
    return GPIO_ReadInputDataBit(port, gpioPin) == 1;
}

// TZIOReadOutputPin 读取输出引脚电平
bool TZIOReadOutputPin(int pin) {
    GPIO_TypeDef* port;
    uint32_t gpioPin;
    uint32_t rcc;
    getPort((Stm32f4IO)pin, &port, &gpioPin, &rcc);
    return GPIO_ReadOutputDataBit(port, gpioPin) == 1;
}

// TZIOConfigIrq 配置中断模式
// 本函数会配置io为输入,不用提前配置.且配置完成后已经使能中断
bool TZIOConfigIrq(int pin, TZIOPullMode pullMode, TZIOIrqPolarity polarity, int priority, TZEmptyFunc callback) {
    int bit = pin % 16;
    if (callback == NULL || gIrqCallback[bit].callback != NULL) {
        return false;
    }

    TZIOConfigInput(pin, pullMode);

    EXTITrigger_TypeDef trigger;
    switch (polarity) {
    case TZIO_LO_TO_HI: trigger = EXTI_Trigger_Rising; break;
    case TZIO_HI_TO_LO: trigger = EXTI_Trigger_Falling; break;
    case TZIO_TOGGLE: trigger = EXTI_Trigger_Rising_Falling; break;
    }

    uint8_t extiPortSourceGpio;
    uint8_t extiPinSource;
    uint32_t extiLine;
    uint8_t irqChannel;
    getIrqParam(pin, &extiPortSourceGpio, &extiPinSource, &extiLine, &irqChannel);

    // 中断配置
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    // 配置中断源
    SYSCFG_EXTILineConfig(extiPortSourceGpio, extiPinSource);

    // 配置触发方式
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_ClearITPendingBit(extiLine);
    EXTI_InitStructure.EXTI_Line = extiLine;
    EXTI_InitStructure.EXTI_Trigger = trigger;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    gIrqCallback[bit].pin = pin;
    gIrqCallback[bit].callback = callback;
    gIrqCallback[bit].priority = priority;
    gIrqCallback[bit].irqChannel = irqChannel;

    TZIOIrqEnable(pin);
    return true;
}

static void getIrqParam(int pin, uint8_t* extiPortSourceGpio, uint8_t* extiPinSource, uint32_t* extiLine, 
    uint8_t* irqChannel) {
    int port = pin / 16;
    int bit = pin % 16;

    *extiPortSourceGpio = EXTI_PortSourceGPIOA + port;
    *extiPinSource = EXTI_PinSource0 + bit;
    *extiLine = EXTI_Line0 << bit;
    if (bit <= 4) {
        *irqChannel = EXTI0_IRQn + bit;
    } else {
        if (bit <= 9) {
            *irqChannel = EXTI9_5_IRQn;
        } else {
            *irqChannel = EXTI15_10_IRQn;
        }
    }
}

// TZIOIrqEnable 使能中断
void TZIOIrqEnable(int pin) {
    int bit = pin % 16;
    if (gIrqCallback[bit].callback == NULL) {
        return;
    }

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = gIrqCallback[bit].irqChannel;			
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = gIrqCallback[bit].priority; 		
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;           	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           		
    NVIC_Init(&NVIC_InitStructure);
}

// TZIOIrqDisable 禁止中断
void TZIOIrqDisable(int pin) {
    int bit = pin % 16;
    if (gIrqCallback[bit].callback == NULL) {
        return;
    }

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = gIrqCallback[bit].irqChannel;			
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = gIrqCallback[bit].priority; 		
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;           	
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;           		
    NVIC_Init(&NVIC_InitStructure);
}

// 中断函数
void EXTI0_IRQHandler(void) {
    irqHandler(0);
}

static void irqHandler(uint8_t bit) {
    if (bit >= IRQ_CALLBACK_NUM_MAX) {
        return;
    }
    if (gIrqCallback[bit].callback == NULL) {
        return;
    }
    uint32_t extiLine = (EXTI_Line0 << bit);
    EXTI_ClearITPendingBit(extiLine);
    gIrqCallback[bit].callback();
}

void EXTI1_IRQHandler(void) {
    irqHandler(1);
}

void EXTI2_IRQHandler(void) {
    irqHandler(2);
}

void EXTI3_IRQHandler(void) {
    irqHandler(3);
}

void EXTI4_IRQHandler(void) {
    irqHandler(4);
}

void EXTI9_5_IRQHandler(void) {
    int bit = 0;
    for (bit = 5; bit <= 9; bit++) {
        if (gIrqCallback[bit].callback != NULL) {
            break;
        }
    }
    if (bit > 9) {
        return;
    }
    irqHandler((uint8_t)bit);
}

void EXTI15_10_IRQHandler(void) {
    int bit = 0;
    for (bit = 10; bit <= 15; bit++) {
        if (gIrqCallback[bit].callback != NULL) {
            break;
        }
    }
    if (bit > 15) {
        return;
    }
    irqHandler((uint8_t)bit);
}
