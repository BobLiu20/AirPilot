#ifndef __UART_H
#define __UART_H
#include "includes.h"

extern uint32_t txBufferTail ;
extern uint32_t txBufferHead ;
extern uint16_t Usart2_RxDMAPos;

#define USART2_RXAVAILABLE (DMA1_Stream5->NDTR != Usart2_RxDMAPos)//判断是否接收到新数据，true为接收到新数据
#define USART2_RXCLEAR     (Usart2_RxDMAPos=DMA1_Stream5->NDTR)		//清除接收缓存

void USART1_Init(uint32_t speed);
uint8_t USART1_Available(void);
uint8_t USART1_Read(void);
void USART1_Write(uint8_t ch);
void USART1_TxDMA(void);

void USART2_Init(uint32_t speed);
//uint8_t USART2_RxAvailable(void);
uint8_t USART2_Read(void);


void UART4_Init(void);

void USART3_Init(void);
uint8_t USART3_Available(void);
uint8_t USART3_Read(void);
#endif

