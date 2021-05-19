// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4����4����
// Authors: jdh99 <jdh821@163.com>

#include "tzstm32f4uart4.h"
#include "tzstm32f4uartcom.h"
#include "tzio.h"
#include "tzstm32f4io.h"
#include "tzmalloc.h"

#include "stm32f4xx.h"

#include <string.h>

// ���ں�
#define UART_INDEX 4

// ����DMA
#define TX_DMA_INDEX 1
#define TX_DMA_CHANNEL 4
#define TX_DMA_STREAM 4

// ����DMA
#define RX_DMA_INDEX 1
#define RX_DMA_CHANNEL 4
#define RX_DMA_STREAM 2

// �жϴ�����
#define TX_DMA_IRQHandler DMA1_Stream4_IRQHandler
#define UART_IRQHandler UART4_IRQHandler

//  DMA����
struct DmaParam {
    DMA_Stream_TypeDef* stream;
    uint32_t itTCIF;
    uint32_t flagTCIF;
};

static int mid = -1;
static TZStm32f4UartParam gParam;
static struct DmaParam gTxDmaParam = {0};
static struct DmaParam gRxDmaParam = {0};

// �����շ�DMA�Ļ���
static TZBufferDynamic* gTxBuffer = NULL;
static TZBufferDynamic* gRxBuffer = NULL;

static USART_TypeDef* gUartDevice = NULL;
static bool gIsTx = false;
static TZDataFunc gCallbackRx = NULL;

static bool initBuffer(void);
static void rxCtrlPinIrqHandler(void);
static void initDma(void);
static void initUart(void);

static inline void startTx(void) {
    gIsTx = true;
    if (gParam.TxCtrlPin == STM32F4_INVALID_PIN) {
        return;
    }
    TZIOSetLow(gParam.TxCtrlPin);
}

static inline void stopTx(void) {
    gIsTx = false;
    if (gParam.TxCtrlPin == STM32F4_INVALID_PIN) {
        return;
    }
    TZIOSetHigh(gParam.TxCtrlPin);
}

// TZStm32f4Uart4SetMid �����ڴ�id
// ��������ñ�����.��ģ��ʹ��Ĭ���ڴ�ID
// �����ڵ��ó�ʼ�����ں���ǰ���ñ�����,����ģ��ʹ��Ĭ���ڴ�ID
void TZStm32f4Uart4SetMid(int id) {
    if (mid == -1) {
        mid = id;
    }
}

// TZStm32f4Uart4Init ��ʼ��
bool TZStm32f4Uart4Init(TZStm32f4UartParam param) {
    if (mid == -1) {
        mid = TZMallocRegister(0, TZSTM32F4UART4_TAG, param.MaxFrameSize * 2 + TZ_BUFFER_LEN);
        if (mid == -1) {
            return false;
        }
    }

    gParam = param;
    if (initBuffer() == false) {
        mid = -1;
        gParam.TxCtrlPin = STM32F4_INVALID_PIN;
        gParam.RxCtrlPin = STM32F4_INVALID_PIN;
        return false;
    }

    gUartDevice = TZStm32f4UartGetUartDevice(UART_INDEX);
    // DMA����
    gTxDmaParam.stream = TZStm32f4UartGetDmaStream(TX_DMA_INDEX, TX_DMA_STREAM);
    TZStm32f4UartGetDmaTcif(TX_DMA_STREAM, &gTxDmaParam.itTCIF, &gTxDmaParam.flagTCIF);
    gRxDmaParam.stream = TZStm32f4UartGetDmaStream(RX_DMA_INDEX, RX_DMA_STREAM);
    TZStm32f4UartGetDmaTcif(RX_DMA_STREAM, &gRxDmaParam.itTCIF, &gRxDmaParam.flagTCIF);

    if (gParam.TxCtrlPin != STM32F4_INVALID_PIN) {
        TZIOConfigOutput(gParam.TxCtrlPin, TZIO_NOPULL, TZIO_OUT_PP);
    }
    if (gParam.RxCtrlPin != STM32F4_INVALID_PIN) {
        TZIOConfigIrq(gParam.RxCtrlPin, TZIO_PULLUP, TZIO_LO_TO_HI, gParam.Priority, rxCtrlPinIrqHandler);
    }
    stopTx();

    initDma();
    initUart();
    return true;
}

static bool initBuffer(void) {
    gTxBuffer = TZMalloc(mid, sizeof(TZBufferDynamic) + gParam.MaxFrameSize);
    if (gTxBuffer == NULL) {
        return false;
    }
    gRxBuffer = TZMalloc(mid, sizeof(TZBufferDynamic) + gParam.MaxFrameSize);
    if (gRxBuffer == NULL) {
        TZFree(gTxBuffer); gTxBuffer = NULL;
        return false;
    }
    return true;
}

static void rxCtrlPinIrqHandler(void) {
    if (mid == -1 || gParam.RxCtrlPin == STM32F4_INVALID_PIN) {
        return;
    }

    gUartDevice->SR;
    // ��USART_IT_IDLE��־
    gUartDevice->DR; 
    // �ر�DMA
    DMA_Cmd(gRxDmaParam.stream, DISABLE);
    //�����־λ
    DMA_ClearFlag(gRxDmaParam.stream, gRxDmaParam.flagTCIF);

    // ��ý���֡֡�� gCallbackRx
    gRxBuffer->len = gParam.MaxFrameSize - DMA_GetCurrDataCounter(gRxDmaParam.stream);
    if (gCallbackRx && gRxBuffer->len > 0) {
        gCallbackRx(gRxBuffer->buf, gRxBuffer->len);
    }
   
    // ���ô������ݳ���
    DMA_SetCurrDataCounter(gRxDmaParam.stream, gParam.MaxFrameSize);
    DMA_Cmd(gRxDmaParam.stream, ENABLE);
}

static void initDma(void) {
    TZStm32f4UartDma txDma;
    txDma.Index = TX_DMA_INDEX;
    txDma.Channel = TX_DMA_CHANNEL;
    txDma.Stream = TX_DMA_STREAM;
    txDma.PeripheralBaseAddr = (uint32_t)(&gUartDevice->DR);
    txDma.MemoryBaseAddr = (uint32_t)gTxBuffer->buf;
    txDma.MemorySize = gParam.MaxFrameSize;
    txDma.Priority = gParam.Priority;

    TZStm32f4UartDma rxDma;
    rxDma.Index = RX_DMA_INDEX;
    rxDma.Channel = RX_DMA_CHANNEL;
    rxDma.Stream = RX_DMA_STREAM;
    rxDma.PeripheralBaseAddr = (uint32_t)(&gUartDevice->DR);
    rxDma.MemoryBaseAddr = (uint32_t)gRxBuffer->buf;
    rxDma.MemorySize = gParam.MaxFrameSize;
    rxDma.Priority = gParam.Priority;

    TZStm32f4UartConfigDma(txDma, rxDma);
}

static void initUart(void) {
    // �򿪴��ڶ�Ӧ������ʱ��
    uint32_t uartRcc;
    bool isAPB1; 
    TZStm32f4UartGetUartRcc(UART_INDEX, &uartRcc, &isAPB1);
    if (isAPB1) {
        RCC_APB1PeriphClockCmd(uartRcc, ENABLE);
    } else {
        RCC_APB2PeriphClockCmd(uartRcc, ENABLE);
    }

    // ��ʼ�����ڲ���  
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = gParam.BaudRate; 
    // ��ʼ������ 
    USART_Init(gUartDevice, &USART_InitStructure);  

    // �����ж�  
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TZStm32f4UartGetUartIrqNumber(UART_INDEX);               
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = gParam.Priority;       
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 
    NVIC_Init(&NVIC_InitStructure);   
        
    // ����DMA��ʽ����
    USART_DMACmd(gUartDevice, USART_DMAReq_Tx, ENABLE);
    // ����DMA��ʽ����
    USART_DMACmd(gUartDevice, USART_DMAReq_Rx ,ENABLE);

    // �ж�����
    USART_ITConfig(gUartDevice, USART_IT_TC, DISABLE);
    USART_ITConfig(gUartDevice, USART_IT_RXNE, DISABLE);
    USART_ITConfig(gUartDevice, USART_IT_TXE, DISABLE);

    if (gParam.RxCtrlPin == STM32F4_INVALID_PIN) {
        // ���ô��ڿ��ƽſ��ƽ���,��ʹ�ÿ����ж�
        USART_ITConfig(gUartDevice, USART_IT_IDLE, ENABLE);
    }

    // ��������  
    USART_Cmd(gUartDevice, ENABLE);    

    // ����IO��ʱ��
    GPIO_TypeDef* txPinPort;
    uint32_t txPinGpioPin;
    uint32_t txPinRcc;
    
    GPIO_TypeDef* rxPinPort;
    uint32_t rxPinGpioPin;
    uint32_t rxPinRcc;
    
    TZStm32f4UartGetPort(gParam.TxPin, &txPinPort, &txPinGpioPin, &txPinRcc);
    TZStm32f4UartGetPort(gParam.RxPin, &rxPinPort, &rxPinGpioPin, &rxPinRcc);

    RCC_AHB1PeriphClockCmd(txPinRcc, ENABLE); 
    GPIO_PinAFConfig(txPinPort, gParam.TxPin % 16, TZStm32f4UartGetGpioAfUart(UART_INDEX));

    RCC_AHB1PeriphClockCmd(rxPinRcc, ENABLE); 
    GPIO_PinAFConfig(rxPinPort, gParam.RxPin % 16, TZStm32f4UartGetGpioAfUart(UART_INDEX));

    // ����IO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Pin = txPinGpioPin;
    GPIO_Init(txPinPort, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = rxPinGpioPin;
    GPIO_Init(rxPinPort, &GPIO_InitStructure);      
}

// ����DMA�жϴ�����
void TX_DMA_IRQHandler(void) {
    if (mid == -1) {
        return;
    }

    if (DMA_GetITStatus(gTxDmaParam.stream, gTxDmaParam.itTCIF) != RESET) {
        DMA_ClearFlag(gTxDmaParam.stream, gTxDmaParam.flagTCIF);
        DMA_Cmd(gTxDmaParam.stream, DISABLE);
        // �򿪷�������ж�,������������ֽ�
        USART_ITConfig(gUartDevice, USART_IT_TC, ENABLE);
    }
}

// �����жϴ�����
void UART_IRQHandler(void) {
    if (USART_GetITStatus(gUartDevice, USART_IT_TC) != RESET) {
        USART_ITConfig(gUartDevice, USART_IT_TC, DISABLE);
        stopTx();
    }

    // ��������ж�
    if (USART_GetITStatus(gUartDevice, USART_IT_IDLE) != RESET) {
        gUartDevice->SR;
        // ��USART_IT_IDLE��־
        gUartDevice->DR; 
        // �ر�DMA
        DMA_Cmd(gRxDmaParam.stream, DISABLE);
        //�����־λ
        DMA_ClearFlag(gRxDmaParam.stream, gRxDmaParam.flagTCIF);
        
        // ��ý���֡֡�� gCallbackRx
        gRxBuffer->len = gParam.MaxFrameSize - DMA_GetCurrDataCounter(gRxDmaParam.stream);
        if (gCallbackRx && gRxBuffer->len > 0) {
            gCallbackRx(gRxBuffer->buf, gRxBuffer->len);
        }
        
        // ���ô������ݳ���
        DMA_SetCurrDataCounter(gRxDmaParam.stream, gParam.MaxFrameSize);
        DMA_Cmd(gRxDmaParam.stream, ENABLE);
    }
}

// TZStm32f4Uart4IsBusy ��ǰ�Ƿ�æµ.æµʱ���ܷ���
bool TZStm32f4Uart4IsBusy(void) {
    return gIsTx;
}

// TZStm32f4Uart4Tx ��������
void TZStm32f4Uart4Tx(uint8_t* bytes, int size) {
    if (gIsTx || mid == -1 || size > gParam.MaxFrameSize) {
        return;
    }

    startTx();
    memcpy(gTxBuffer->buf, bytes, size);

    DMA_SetCurrDataCounter(gTxDmaParam.stream, size);
    DMA_Cmd(gTxDmaParam.stream, ENABLE);
    return;
}

// TZStm32f4Uart4RegisterCallback ע����ջص�����
void TZStm32f4Uart4RegisterCallback(TZDataFunc callback) {
    gCallbackRx = callback;
}

// TZStm32f4Uart4EditBaudRate �޸Ĵ��ڲ�����
void TZStm32f4Uart4EditBaudRate(int baudRate) {
    if (mid == -1) {
        return;
    }

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = baudRate;
    USART_Init(gUartDevice, &USART_InitStructure);
    gParam.BaudRate = baudRate;
}
