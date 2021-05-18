// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4����4����
// Authors: jdh99 <jdh821@163.com>

#ifndef TZSTM32F4UART4_H
#define TZSTM32F4UART4_H

#include "tztype.h"
#include "tzstm32f4io.h"
#include "tzstm32f4uartcom.h"

#define TZSTM32F4UART4_TAG "tzstm32f4uart4"

// TZStm32f4Uart4SetMid �����ڴ�id
// ��������ñ�����.��ģ��ʹ��Ĭ���ڴ�ID
// �����ڵ��ó�ʼ�����ں���ǰ���ñ�����,����ģ��ʹ��Ĭ���ڴ�ID
void TZStm32f4Uart4SetMid(int id);

// TZStm32f4Uart4Init ��ʼ��
bool TZStm32f4Uart4Init(TZStm32f4UartParam param);

// TZStm32f4Uart4IsBusy ��ǰ�Ƿ�æµ.æµʱ���ܷ���
bool TZStm32f4Uart4IsBusy(void);

// TZStm32f4Uart4Tx ��������
void TZStm32f4Uart4Tx(uint8_t* bytes, int size);

// TZStm32f4Uart4RegisterCallback ע����ջص�����
void TZStm32f4Uart4RegisterCallback(TZDataFunc callback);

// TZStm32f4Uart4EditBaudRate �޸Ĵ��ڲ�����
void TZStm32f4Uart4EditBaudRate(int baudRate);

#endif
