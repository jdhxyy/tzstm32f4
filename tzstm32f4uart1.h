// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4����1����.����1�����ڵ��Կ�,��ʹ��DMA����,ʹ�õȴ�ʽ����
// Authors: jdh99 <jdh821@163.com>

#ifndef TZSTM32F4UART1_H
#define TZSTM32F4UART1_H

#include "tztype.h"
#include "tzstm32f4io.h"
#include "tzstm32f4uartcom.h"

#define TZSTM32F4UART1_TAG "tzstm32f4uart1"

// TZStm32f4Uart1SetMid �����ڴ�id
// ��������ñ�����.��ģ��ʹ��Ĭ���ڴ�ID
// �����ڵ��ó�ʼ�����ں���ǰ���ñ�����,����ģ��ʹ��Ĭ���ڴ�ID
void TZStm32f4Uart1SetMid(int id);

// TZStm32f4Uart1Init ��ʼ��
bool TZStm32f4Uart1Init(TZStm32f4UartParam param);

// TZStm32f4Uart1IsBusy ��ǰ�Ƿ�æµ.æµʱ���ܷ���
bool TZStm32f4Uart1IsBusy(void);

// TZStm32f4Uart1Tx ��������
void TZStm32f4Uart1Tx(uint8_t* bytes, int size);

// TZStm32f4Uart1RegisterCallback ע����ջص�����
void TZStm32f4Uart1RegisterCallback(TZDataFunc callback);

// TZStm32f4Uart1EditBaudRate �޸Ĵ��ڲ�����
void TZStm32f4Uart1EditBaudRate(int baudRate);

#endif
