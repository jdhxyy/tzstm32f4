// Copyright 2020-2021 The TZIOT Authors. All rights reserved.
// stm32f4�Ĵ������������ļ�
// Authors: jdh99 <jdh821@163.com>

#include "tzstm32f4uartcom.h"
#include "stm32f4xx.h"

struct DmaConfig {
    uint32_t rccDma;
    uint32_t irqChannel;
    DMA_Stream_TypeDef *dmaStream;
    uint32_t dmaChannel;
    uint32_t peripheralBaseAddr;
    uint32_t memoryBaseAddr;
    uint32_t memorySize;
    int priority;
};

static struct DmaConfig getDmaConfig(TZStm32f4UartDma uartDma);
static uint32_t getIrqChannel(int dmaIndex, int stream);
static uint32_t getDmaChannel(int channel);
static void configDma(struct DmaConfig dmaConfigTx, struct DmaConfig dmaConfigRx);

// TZStm32f4UartConfigDma ���ô���dma
void TZStm32f4UartConfigDma(TZStm32f4UartDma txDma, TZStm32f4UartDma rxDma) {
    struct DmaConfig dmaConfigTx = getDmaConfig(txDma);
    struct DmaConfig dmaConfigRx = getDmaConfig(rxDma);
    configDma(dmaConfigTx, dmaConfigRx);
}

static struct DmaConfig getDmaConfig(TZStm32f4UartDma uartDma) {
    struct DmaConfig dmaConfig;
    dmaConfig.rccDma = uartDma.Index == 1 ? RCC_AHB1Periph_DMA1 : RCC_AHB1Periph_DMA2;
    dmaConfig.irqChannel = getIrqChannel(uartDma.Index, uartDma.Stream);
    dmaConfig.dmaChannel = getDmaChannel(uartDma.Channel);
    dmaConfig.dmaStream = TZStm32f4UartGetDmaStream(uartDma.Index, uartDma.Stream);
    dmaConfig.peripheralBaseAddr = uartDma.PeripheralBaseAddr;
    dmaConfig.memoryBaseAddr = uartDma.MemoryBaseAddr;
    dmaConfig.memorySize = uartDma.MemorySize;
    dmaConfig.priority = uartDma.Priority;
    return dmaConfig;
}

static uint32_t getIrqChannel(int dmaIndex, int stream) {
    if (dmaIndex == 1) {
        switch (stream) {
        case 0: return DMA1_Stream0_IRQn;
        case 1: return DMA1_Stream1_IRQn;
        case 2: return DMA1_Stream2_IRQn;
        case 3: return DMA1_Stream3_IRQn;
        case 4: return DMA1_Stream4_IRQn;
        case 5: return DMA1_Stream5_IRQn;
        case 6: return DMA1_Stream6_IRQn;
        default: return DMA1_Stream7_IRQn;
        }
    } else {
        switch (stream) {
        case 0: return DMA2_Stream0_IRQn;
        case 1: return DMA2_Stream1_IRQn;
        case 2: return DMA2_Stream2_IRQn;
        case 3: return DMA2_Stream3_IRQn;
        case 4: return DMA2_Stream4_IRQn;
        case 5: return DMA2_Stream5_IRQn;
        case 6: return DMA2_Stream6_IRQn;
        default: return DMA2_Stream7_IRQn;
        }
    }
}

static uint32_t getDmaChannel(int channel) {
    switch (channel) {
    case 0: return DMA_Channel_0;
    case 1: return DMA_Channel_1;
    case 2: return DMA_Channel_2;
    case 3: return DMA_Channel_3;
    case 4: return DMA_Channel_4;
    case 5: return DMA_Channel_5;
    case 6: return DMA_Channel_6;
    default: return DMA_Channel_7;
    }
}

// TZStm32f4UartGetDmaStream ��ȡDMA���ṹ��
DMA_Stream_TypeDef* TZStm32f4UartGetDmaStream(int dmaIndex, int stream) {
    if (dmaIndex == 1) {
        switch (stream) {
        case 0: return DMA1_Stream0;
        case 1: return DMA1_Stream1;
        case 2: return DMA1_Stream2;
        case 3: return DMA1_Stream3;
        case 4: return DMA1_Stream4;
        case 5: return DMA1_Stream5;
        case 6: return DMA1_Stream6;
        default: return DMA1_Stream7;
        }
    } else {
        switch (stream) {
        case 0: return DMA2_Stream0;
        case 1: return DMA2_Stream1;
        case 2: return DMA2_Stream2;
        case 3: return DMA2_Stream3;
        case 4: return DMA2_Stream4;
        case 5: return DMA2_Stream5;
        case 6: return DMA2_Stream6;
        default: return DMA2_Stream7;
        }
    }
}

static void configDma(struct DmaConfig dmaConfigTx, struct DmaConfig dmaConfigRx) {	 
    // ��DMA����  
    // ����DMAʱ��
    RCC_AHB1PeriphClockCmd(dmaConfigTx.rccDma, ENABLE);

    // DMA�����ж�����
    NVIC_InitTypeDef NVIC_InitStructure ;
    NVIC_InitStructure.NVIC_IRQChannel = dmaConfigTx.irqChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = dmaConfigTx.priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // DMAͨ������
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(dmaConfigTx.dmaStream);
    DMA_InitStructure.DMA_Channel = dmaConfigTx.dmaChannel; 
    // �����ַ
    DMA_InitStructure.DMA_PeripheralBaseAddr = dmaConfigTx.peripheralBaseAddr;
    // �ڴ��ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = dmaConfigTx.memoryBaseAddr;
    // dma���䷽��
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    // ����DMA�ڴ���ʱ�������ĳ���
    DMA_InitStructure.DMA_BufferSize = dmaConfigTx.memorySize;
    // ����DMA���������ģʽ��һ������
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // ����DMA���ڴ����ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // ���������ֳ�
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // �ڴ������ֳ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    // ����DMA�Ĵ���ģʽ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    // ����DMA�����ȼ���
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    // ָ�����FIFOģʽ��ֱ��ģʽ������ָ������:��ʹ��FIFOģʽ  
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;    
    // ָ����FIFO��ֵˮƽ
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;        
    // ָ����Burstת�������ڴ洫�� 
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;       
    // ָ����Burstת��������Χת�� */  
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 

    // ����DMA��ͨ��         
    DMA_Init(dmaConfigTx.dmaStream, &DMA_InitStructure);  
    // ʹ���ж�
    DMA_ITConfig(dmaConfigTx.dmaStream, DMA_IT_TC, ENABLE);   

    // ��DMA����  
    // ����DMAʱ��
    RCC_AHB1PeriphClockCmd(dmaConfigRx.rccDma, ENABLE);
    // DMAͨ������
    DMA_DeInit(dmaConfigRx.dmaStream);
    DMA_InitStructure.DMA_Channel = dmaConfigRx.dmaChannel;
    // �����ַ
    DMA_InitStructure.DMA_PeripheralBaseAddr = dmaConfigRx.peripheralBaseAddr;
    // �ڴ��ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = dmaConfigRx.memoryBaseAddr;
    // dma���䷽��
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    // ����DMA�ڴ���ʱ�������ĳ���
    DMA_InitStructure.DMA_BufferSize = dmaConfigRx.memorySize;
    // ����DMA���������ģʽ��һ������
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // ����DMA���ڴ����ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // ���������ֳ�
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // �ڴ������ֳ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // ����DMA�Ĵ���ģʽ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    // ����DMA�����ȼ���
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;

    // ָ�����FIFOģʽ��ֱ��ģʽ������ָ����������ʹ��FIFOģʽ  
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;    
    // ָ����FIFO��ֵˮƽ
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;        
    // ָ����Burstת�������ڴ洫�� 
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;       
    // ָ����Burstת��������Χת�� */  
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 

    // ����DMA��ͨ��         
    DMA_Init(dmaConfigRx.dmaStream, &DMA_InitStructure);  
    // ʹ��ͨ��
    DMA_Cmd(dmaConfigRx.dmaStream, ENABLE);
}

// TZStm32f4UartGetDmaTcif ��ȡDMA����TCIFλ��IT��FLAG
void TZStm32f4UartGetDmaTcif(int stream, uint32_t* dmaItTcif, uint32_t* dmaFlagTcif) {
    switch (stream) {
    case 0: *dmaItTcif = DMA_IT_TCIF0; *dmaFlagTcif = DMA_FLAG_TCIF0; break;
    case 1: *dmaItTcif = DMA_IT_TCIF1; *dmaFlagTcif = DMA_FLAG_TCIF1; break;
    case 2: *dmaItTcif = DMA_IT_TCIF2; *dmaFlagTcif = DMA_FLAG_TCIF2; break;
    case 3: *dmaItTcif = DMA_IT_TCIF3; *dmaFlagTcif = DMA_FLAG_TCIF3; break;
    case 4: *dmaItTcif = DMA_IT_TCIF4; *dmaFlagTcif = DMA_FLAG_TCIF4; break;
    case 5: *dmaItTcif = DMA_IT_TCIF5; *dmaFlagTcif = DMA_FLAG_TCIF5; break;
    case 6: *dmaItTcif = DMA_IT_TCIF6; *dmaFlagTcif = DMA_FLAG_TCIF6; break;
    default: *dmaItTcif = DMA_IT_TCIF7; *dmaFlagTcif = DMA_FLAG_TCIF7; break;
    }
}

// TZStm32f4UartGetUartDevice ��ȡ�����豸�ṹ��
USART_TypeDef* TZStm32f4UartGetUartDevice(int uartIndex) {
    switch (uartIndex) {
    case 1: return USART1;
    case 2: return USART2;
    case 3: return USART3;
    case 4: return UART4;
    case 5: return UART5;
    default: return USART6;
    }
}

// TZStm32f4UartGetUartRcc ��ȡ����RCC
void TZStm32f4UartGetUartRcc(int uartIndex, uint32_t* rcc, bool* isAPB1) {
    switch (uartIndex) {
    case 1: *rcc = RCC_APB2Periph_USART1; *isAPB1 = false; break;
    case 2: *rcc = RCC_APB1Periph_USART2; *isAPB1 = true; break;
    case 3: *rcc = RCC_APB1Periph_USART3; *isAPB1 = true; break;
    case 4: *rcc = RCC_APB1Periph_UART4; *isAPB1 = true; break;
    case 5: *rcc = RCC_APB1Periph_UART5; *isAPB1 = true; break;
    default: *rcc = RCC_APB2Periph_USART6; *isAPB1 = false; break;;
    }
}

// TZStm32f4UartGetUartIrqNumber ��ȡ�����жϺ�
int TZStm32f4UartGetUartIrqNumber(int uartIndex) {
    switch (uartIndex) {
    case 1: return USART1_IRQn;
    case 2: return USART2_IRQn;
    case 3: return USART3_IRQn;
    case 4: return UART4_IRQn;
    case 5: return UART5_IRQn;
    default: return USART6_IRQn;
    }
}

// TZStm32f4UartGetPort ��ȡio�˿���Ϣ
void TZStm32f4UartGetPort(Stm32f4IO pin, GPIO_TypeDef** port, uint32_t* gpioPin, uint32_t* rcc) {
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

// TZStm32f4UartGetAFUart ��ȡGPIO_AF_UART
uint8_t TZStm32f4UartGetGpioAfUart(int uartIndex) {
    switch (uartIndex) {
    case 1: return GPIO_AF_USART1;
    case 2: return GPIO_AF_USART2;
    case 3: return GPIO_AF_USART3;
    case 4: return GPIO_AF_UART4;
    case 5: return GPIO_AF_UART5;
    default: return GPIO_AF_USART6;
    }
}
