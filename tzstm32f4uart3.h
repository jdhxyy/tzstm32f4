// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4����3����
// Authors: jdh99 <jdh821@163.com>

#ifndef TZSTM32F4UART3_H
#define TZSTM32F4UART3_H

#include "tztype.h"
#include "tzstm32f4io.h"
#include "tzstm32f4uartcom.h"

#define TZSTM32F4UART3_TAG "tzstm32f4uart3"

// TZStm32f4Uart3SetMid �����ڴ�id
// ��������ñ�����.��ģ��ʹ��Ĭ���ڴ�ID
// �����ڵ��ó�ʼ�����ں���ǰ���ñ�����,����ģ��ʹ��Ĭ���ڴ�ID
void TZStm32f4Uart3SetMid(int id);

// TZStm32f4Uart3Init ��ʼ��
bool TZStm32f4Uart3Init(TZStm32f4UartParam param);

// TZStm32f4Uart3IsBusy ��ǰ�Ƿ�æµ.æµʱ���ܷ���
bool TZStm32f4Uart3IsBusy(void);

// TZStm32f4Uart3Tx ��������
void TZStm32f4Uart3Tx(uint8_t* bytes, int size);

// TZStm32f4Uart3RegisterCallback ע����ջص�����
void TZStm32f4Uart3RegisterCallback(TZDataFunc callback);

// TZStm32f4Uart3EditBaudRate �޸Ĵ��ڲ�����
void TZStm32f4Uart3EditBaudRate(int baudRate);

#endif
