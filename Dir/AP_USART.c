/**
	******************************************************************************
	* @file    AP_USART.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   UASRT����
	*					 USART1:��λ���������ͨѶ					TX:PA9	RX:PA10
	*					 USART2:GPSͨѶ(ֻ��Ҫ����)									RX:PA3
	*					 USART4:����վͨѶ										TX:PC10 RX:PC11
	*					 USART3:����ͨѶ											TX:PD8  RX:PD9
	******************************************************************************
**/
#include "AP_USART.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
#define UART_BUFFER_SIZE    255
#define UART_BUFFER_GPSDATA_SIZE 1024			//GPS�������һ֡��Ż���700�ֽ�

/* variables ---------------------------------------------------------*/
volatile uint8_t Usart1_RxBuffer[UART_BUFFER_SIZE];//���ջ��棬����DMA�Զ���
uint32_t Usart1_RxDMAPos = 0;

volatile uint8_t Usart1_TxBuffer[UART_BUFFER_SIZE];
uint32_t Usart1_TxBufferTail = 0;
uint32_t Usart1_TxBufferHead = 0;

volatile uint8_t Usart2_RxBuffer[UART_BUFFER_GPSDATA_SIZE];//GPS���ݽ��ջ��棬����DMA�Զ���
uint16_t Usart2_RxDMAPos = 0;

/* function prototypes -----------------------------------------------*/

/*
*********************************************************************************************************
*                                         �������ͨѶ
*********************************************************************************************************
*/
/**
  * @brief  USART1��ʼ��
  *         ʹ�ý��պͷ���DMA
  * @param  None
  * @retval None
  */
void USART1_Init(uint32_t speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// UART
	// USART1_TX    PA9
	// USART1_RX    PA10
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10|GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// DMA TX Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;		//USART1 TX DMA:DMA2_Ch4_Stream7
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_DMA2STREAM7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_DMA2STREAM7;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_InitStructure.USART_BaudRate = speed;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	/*CONFIG DMA2_CH4_STREAM5 FOR USART1_RX*/
	/* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* DMA2 Stream5 Configuration */
	DMA_DeInit(DMA2_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Usart1_RxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = UART_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/* DMA2 IRQ channel Configuration */
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream5, ENABLE);//ʹ��DMA2_s5
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);//ʹ�ܿ���USART1��DMA
  Usart1_RxDMAPos = DMA_GetCurrDataCounter(DMA2_Stream5);

	// Transmit DMA
	/* DMA2 Stream7 Configuration */
	DMA_DeInit(DMA2_Stream7);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/* DMA2 IRQ channel Configuration */
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);
	DMA_ITConfig (DMA2_Stream7, DMA_IT_TC, ENABLE);//���ж�
	DMA_SetCurrDataCounter (DMA2_Stream7, 0);			 //�����
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  �жϺ���
  *           
  * @param  None
  * @retval None
  */
void DMA2_Stream7_IRQHandler(void)
{
	if (Usart1_TxBufferHead != Usart1_TxBufferTail)//��������Ҫ���ͣ����뷢��
		USART1_TxDMA();
	else//��������뷢�ͣ�������Ҫ�����ǣ���Ϊ������Ҳ��������
		DMA2->HIFCR = (uint32_t)((DMA_IT_TCIF7|DMA_IT_HTIF7|DMA_IT_TEIF7|DMA_IT_DMEIF7|DMA_IT_FEIF7) & 0x0F7D0F7D);//DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);//
}

/**
  * @brief  ���ݷ��ͺ���
  *         ע�������ʹ��ʱ����DMA_Cmd(DMA2_Stream7, ENABLE);֮ǰ��������������жϱ��λ����ϸ�뿴DATASHEEP
	*										ֻ�е�ENλΪ0ʱ�����ܶ������Ĵ���д����
  * @param  None
  * @retval None
  */
void USART1_TxDMA(void)
{
	DMA2->HIFCR = (uint32_t)((DMA_IT_TCIF7|DMA_IT_HTIF7|DMA_IT_TEIF7|DMA_IT_DMEIF7|DMA_IT_FEIF7) & 0x0F7D0F7D);//��ʹ�ܷ���֮ǰ��������������жϱ��
	DMA2_Stream7->M0AR = (uint32_t)&Usart1_TxBuffer[Usart1_TxBufferTail];
	if (Usart1_TxBufferHead > Usart1_TxBufferTail)
	{
			DMA2_Stream7->NDTR = Usart1_TxBufferHead - Usart1_TxBufferTail;
			Usart1_TxBufferTail = Usart1_TxBufferHead;
	}
	else
	{
			DMA2_Stream7->NDTR = UART_BUFFER_SIZE - Usart1_TxBufferTail;
			Usart1_TxBufferTail = 0;
	}
	DMA2_Stream7->CR |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
}

/**
  * @brief  �ж��Ƿ���յ���Ч����
  *         
  * @param  None
  * @retval true:���յ���Ч���ݣ�false����֮
  */
uint8_t USART1_Available(void)
{
    return ( DMA2_Stream5->NDTR != Usart1_RxDMAPos) ? true : false;//DMA_GetCurrDataCounter(DMA2_Stream5)
}

/**
  * @brief  ��ȡһ�ֽ�����
  *         
  * @param  None
  * @retval ����
  */
uint8_t USART1_Read(void)
{
    uint8_t ch;

    ch = Usart1_RxBuffer[UART_BUFFER_SIZE - Usart1_RxDMAPos];
    // go back around the buffer
    if (--Usart1_RxDMAPos == 0)
        Usart1_RxDMAPos = UART_BUFFER_SIZE;

    return ch;
}

/**
  * @brief  д��һ�ֽ����ݵ�����
  *         
  * @param  ����
  * @retval None
  */
void USART1_Write(uint8_t ch)
{
	Usart1_TxBuffer[Usart1_TxBufferHead] = ch;
	Usart1_TxBufferHead = (Usart1_TxBufferHead + 1) % UART_BUFFER_SIZE;
}

/*
*********************************************************************************************************
*                                         GPSͨѶ
*********************************************************************************************************
*/
/**
  * @brief  USART2��ʼ��
  *         ֻ��Ҫ�õ�����RX�������õ�TX
  * @param  
  * @retval 
  */
void USART2_Init(uint32_t speed)
{
  //NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	USART_InitStructure.USART_BaudRate = speed;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART_InitStructure);
	
		/*CONFIG DMA1_CH4_STREAM5 FOR USART1_RX*/
	/* Enable DMA1 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* DMA1 Stream5 Configuration */
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Usart2_RxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = UART_BUFFER_GPSDATA_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5, ENABLE);//ʹ��DMA1_s5
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);//ʹ�ܿ���USART2��DMA
  Usart2_RxDMAPos = DMA_GetCurrDataCounter(DMA1_Stream5);

	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

/**
  * @brief  ���ݶ�ȡ
  *         
  * @param  None
  * @retval ����
  */
uint8_t USART2_Read(void)
{
    uint8_t ch;

    ch = Usart2_RxBuffer[UART_BUFFER_GPSDATA_SIZE - Usart2_RxDMAPos];
    // go back around the buffer
    if (--Usart2_RxDMAPos == 0)
        Usart2_RxDMAPos = UART_BUFFER_GPSDATA_SIZE;

    return ch;
}

/*
*********************************************************************************************************
*                                         UART4 (XbeePro��������)
*********************************************************************************************************
*/
/**
  * @brief  ������������
  *         ʹ�÷���DMA��ʹ�ý����ж�
  * @param  None
  * @retval None
  */
#ifdef XBEEPRO
extern unsigned char UART_Tx_Buf[];
void UART4_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_UART4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_UART4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = 57600;//115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UART4, &USART_InitStructure);

	// Transmit DMA
  DMA_DeInit(DMA1_Stream4);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UART_Tx_Buf;	
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = 0;	//��ʱ����Ϊ0
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);	  //����UART4��DMA����

  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  USART_Cmd(UART4, ENABLE);
}

#endif


/*
*********************************************************************************************************
*                                         ����OpticalFlow
*********************************************************************************************************
*/
/**
  * @brief  ���ڹ�������
  * 				DMA���ڽ���
  * @param  None
  * @retval None
  */
#ifdef OPTICALFLOW
#define USART3_RX_BUFFSIZE	30
unsigned char USART3_Rx_Buf[USART3_RX_BUFFSIZE];
uint16_t Usart3_RxDMAPos = 0;

void USART3_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3, &USART_InitStructure);

	// RX DMA
  DMA_DeInit(DMA1_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART3_Rx_Buf;	
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = USART3_RX_BUFFSIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream1, ENABLE);//ʹ��DMA1_s1
	
	Usart3_RxDMAPos = DMA_GetCurrDataCounter(DMA1_Stream1);
	
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);	  //����USART3��DMA����

  USART_Cmd(USART3, ENABLE);
}

/**
  * @brief  �ж��Ƿ���յ���Ч����
  *         
  * @param  None
  * @retval true:���յ���Ч���ݣ�false����֮
  */
uint8_t USART3_Available(void)
{
    return ( DMA1_Stream1->NDTR != Usart3_RxDMAPos) ? true : false;//DMA_GetCurrDataCounter(DMA2_Stream5)
}

/**
  * @brief  ��ȡһ�ֽ�����
  *         
  * @param  None
  * @retval ����
  */
uint8_t USART3_Read(void)
{
    uint8_t ch;

    ch = USART3_Rx_Buf[USART3_RX_BUFFSIZE - Usart3_RxDMAPos];
    // go back around the buffer
    if (--Usart3_RxDMAPos == 0)
        Usart3_RxDMAPos = USART3_RX_BUFFSIZE;

    return ch;
}


#endif
