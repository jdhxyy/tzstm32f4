// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4的串口驱动公共文件
// Authors: jdh99 <jdh821@163.com>

#ifndef TZSTM32F4UART_H
#define TZSTM32F4UART_H

#include "tztype.h"
#include "tzstm32f4io.h"
#include "stm32f4xx.h"

#pragma pack(1)

typedef struct {
    // DMA序号只能取值1或者2
    int Index;
    int Channel;
    int Stream;
    // 外设基地址
    uint32_t PeripheralBaseAddr;
    // 内存基地址
    uint32_t MemoryBaseAddr;
    // 内存最大字节数
    uint32_t MemorySize;
    // 中断优先级
    int Priority;
} TZStm32f4UartDma;

typedef struct {
    int MaxFrameSize;
    int BaudRate;
    // 中断优先级
    int Priority;

    // 通信引脚
    Stm32f4IO TxPin;
    Stm32f4IO RxPin;
    // 控制引脚.发送时低电平,发送完成高电平
    // 引脚为STM32F4_INVALID_PIN表示引脚无效
    Stm32f4IO TxCtrlPin;
    Stm32f4IO RxCtrlPin;
} TZStm32f4UartParam;

#pragma pack()

// TZStm32f4UartConfigDma 配置串口dma
void TZStm32f4UartConfigDma(TZStm32f4UartDma txDma, TZStm32f4UartDma rxDma);

// TZStm32f4UartGetDmaStream 获取DMA流结构体
DMA_Stream_TypeDef* TZStm32f4UartGetDmaStream(int dmaIndex, int stream);

// TZStm32f4UartConfigUart 配置串口波特率等参数
void TZStm32f4UartConfigUart(int uartIndex, int baudRate, int priority);

// TZStm32f4UartGetDmaTcif 获取DMA流的TCIF位的IT和FLAG
void TZStm32f4UartGetDmaTcif(int stream, uint32_t* dmaItTcif, uint32_t* dmaFlagTcif);

// TZStm32f4UartGetUartDevice 读取串口设备结构体
USART_TypeDef* TZStm32f4UartGetUartDevice(int uartIndex);

// TZStm32f4UartGetUartRcc 获取串口RCC
void TZStm32f4UartGetUartRcc(int uartIndex, uint32_t* rcc, bool* isAPB1);

// TZStm32f4UartGetUartIrqNumber 获取串口中断号
int TZStm32f4UartGetUartIrqNumber(int uartIndex);

// TZStm32f4UartGetPort 获取io端口信息
void TZStm32f4UartGetPort(Stm32f4IO pin, GPIO_TypeDef** port, uint32_t* gpioPin, uint32_t* rcc);

// TZStm32f4UartGetAFUart 获取GPIO_AF_UART
uint8_t TZStm32f4UartGetGpioAfUart(int uartIndex);

#endif
