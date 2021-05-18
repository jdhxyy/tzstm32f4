// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4����6����
// Authors: jdh99 <jdh821@163.com>

#ifndef TZSTM32F4UART6_H
#define TZSTM32F4UART6_H

#include "tztype.h"
#include "tzstm32f4io.h"
#include "tzstm32f4uartcom.h"

#define TZSTM32F4UART6_TAG "tzstm32f4uart6"

// TZStm32f4Uart6SetMid �����ڴ�id
// ��������ñ�����.��ģ��ʹ��Ĭ���ڴ�ID
// �����ڵ��ó�ʼ�����ں���ǰ���ñ�����,����ģ��ʹ��Ĭ���ڴ�ID
void TZStm32f4Uart6SetMid(int id);

// TZStm32f4Uart6Init ��ʼ��
bool TZStm32f4Uart6Init(TZStm32f4UartParam param);

// TZStm32f4Uart6IsBusy ��ǰ�Ƿ�æµ.æµʱ���ܷ���
bool TZStm32f4Uart6IsBusy(void);

// TZStm32f4Uart6Tx ��������
void TZStm32f4Uart6Tx(uint8_t* bytes, int size);

// TZStm32f4Uart6RegisterCallback ע����ջص�����
void TZStm32f4Uart6RegisterCallback(TZDataFunc callback);

// TZStm32f4Uart6EditBaudRate �޸Ĵ��ڲ�����
void TZStm32f4Uart6EditBaudRate(int baudRate);

#endif
