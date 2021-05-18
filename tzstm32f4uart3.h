// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4串口3驱动
// Authors: jdh99 <jdh821@163.com>

#ifndef TZSTM32F4UART3_H
#define TZSTM32F4UART3_H

#include "tztype.h"
#include "tzstm32f4io.h"
#include "tzstm32f4uartcom.h"

#define TZSTM32F4UART3_TAG "tzstm32f4uart3"

// TZStm32f4Uart3SetMid 设置内存id
// 如果不调用本函数.则模块使用默认内存ID
// 必须在调用初始化串口函数前调用本函数,否则模块使用默认内存ID
void TZStm32f4Uart3SetMid(int id);

// TZStm32f4Uart3Init 初始化
bool TZStm32f4Uart3Init(TZStm32f4UartParam param);

// TZStm32f4Uart3IsBusy 当前是否忙碌.忙碌时不能发送
bool TZStm32f4Uart3IsBusy(void);

// TZStm32f4Uart3Tx 发送数据
void TZStm32f4Uart3Tx(uint8_t* bytes, int size);

// TZStm32f4Uart3RegisterCallback 注册接收回调函数
void TZStm32f4Uart3RegisterCallback(TZDataFunc callback);

// TZStm32f4Uart3EditBaudRate 修改串口波特率
void TZStm32f4Uart3EditBaudRate(int baudRate);

#endif
