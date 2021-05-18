// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4�Ĵ������������ļ�
// Authors: jdh99 <jdh821@163.com>

#ifndef TZSTM32F4UART_H
#define TZSTM32F4UART_H

#include "tztype.h"
#include "tzstm32f4io.h"
#include "stm32f4xx.h"

#pragma pack(1)

typedef struct {
    // DMA���ֻ��ȡֵ1����2
    int Index;
    int Channel;
    int Stream;
    // �������ַ
    uint32_t PeripheralBaseAddr;
    // �ڴ����ַ
    uint32_t MemoryBaseAddr;
    // �ڴ�����ֽ���
    uint32_t MemorySize;
    // �ж����ȼ�
    int Priority;
} TZStm32f4UartDma;

typedef struct {
    int MaxFrameSize;
    int BaudRate;
    // �ж����ȼ�
    int Priority;

    // ͨ������
    Stm32f4IO TxPin;
    Stm32f4IO RxPin;
    // ��������.����ʱ�͵�ƽ,������ɸߵ�ƽ
    // ����ΪSTM32F4_INVALID_PIN��ʾ������Ч
    Stm32f4IO TxCtrlPin;
    Stm32f4IO RxCtrlPin;
} TZStm32f4UartParam;

#pragma pack()

// TZStm32f4UartConfigDma ���ô���dma
void TZStm32f4UartConfigDma(TZStm32f4UartDma txDma, TZStm32f4UartDma rxDma);

// TZStm32f4UartGetDmaStream ��ȡDMA���ṹ��
DMA_Stream_TypeDef* TZStm32f4UartGetDmaStream(int dmaIndex, int stream);

// TZStm32f4UartConfigUart ���ô��ڲ����ʵȲ���
void TZStm32f4UartConfigUart(int uartIndex, int baudRate, int priority);

// TZStm32f4UartGetDmaTcif ��ȡDMA����TCIFλ��IT��FLAG
void TZStm32f4UartGetDmaTcif(int stream, uint32_t* dmaItTcif, uint32_t* dmaFlagTcif);

// TZStm32f4UartGetUartDevice ��ȡ�����豸�ṹ��
USART_TypeDef* TZStm32f4UartGetUartDevice(int uartIndex);

// TZStm32f4UartGetUartRcc ��ȡ����RCC
void TZStm32f4UartGetUartRcc(int uartIndex, uint32_t* rcc, bool* isAPB1);

// TZStm32f4UartGetUartIrqNumber ��ȡ�����жϺ�
int TZStm32f4UartGetUartIrqNumber(int uartIndex);

// TZStm32f4UartGetPort ��ȡio�˿���Ϣ
void TZStm32f4UartGetPort(Stm32f4IO pin, GPIO_TypeDef** port, uint32_t* gpioPin, uint32_t* rcc);

// TZStm32f4UartGetAFUart ��ȡGPIO_AF_UART
uint8_t TZStm32f4UartGetGpioAfUart(int uartIndex);

#endif
