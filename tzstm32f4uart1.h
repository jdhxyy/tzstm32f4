// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4串口1驱动.串口1可用于调试口,不使用DMA发送,使用等待式发送
// Authors: jdh99 <jdh821@163.com>

#ifndef TZSTM32F4UART1_H
#define TZSTM32F4UART1_H

#include "tztype.h"
#include "tzstm32f4io.h"
#include "tzstm32f4uartcom.h"

#define TZSTM32F4UART1_TAG "tzstm32f4uart1"

// TZStm32f4Uart1SetMid 设置内存id
// 如果不调用本函数.则模块使用默认内存ID
// 必须在调用初始化串口函数前调用本函数,否则模块使用默认内存ID
void TZStm32f4Uart1SetMid(int id);

// TZStm32f4Uart1Init 初始化
bool TZStm32f4Uart1Init(TZStm32f4UartParam param);

// TZStm32f4Uart1IsBusy 当前是否忙碌.忙碌时不能发送
bool TZStm32f4Uart1IsBusy(void);

// TZStm32f4Uart1Tx 发送数据
void TZStm32f4Uart1Tx(uint8_t* bytes, int size);

// TZStm32f4Uart1RegisterCallback 注册接收回调函数
void TZStm32f4Uart1RegisterCallback(TZDataFunc callback);

// TZStm32f4Uart1EditBaudRate 修改串口波特率
void TZStm32f4Uart1EditBaudRate(int baudRate);

#endif
