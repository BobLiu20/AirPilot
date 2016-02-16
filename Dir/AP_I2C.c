/**
	******************************************************************************
	* @file    AP_I2C.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   I2C通讯函数
	*					 I2C2:用于传感器的传输	PB10:SCL	PB11:SDA
	******************************************************************************
**/
#include "AP_I2C.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
#define TIMEOUT_MAX      				10000//发送超时

/* variables ---------------------------------------------------------*/
OS_FLAG_GRP I2C2_TaskRunFlagGrp;	//I2C2事件标志组

uint16_t I2C_ErrorCount;						//I2C错误累计

/* function prototypes -----------------------------------------------*/

/**
  * @brief  I2C2的初始化
  *         
  * @param  none
  * @retval none
  */
void I2C2_Init(void)
{
	OS_ERR err;
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* I2C2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	/* GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Connect I2C2 pins to AF */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	/* Configure I2C2 GPIOs */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// EV　Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_I2C2EV;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_I2C2EV;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* I2C DeInit */
	I2C_DeInit(I2C2);

	/* Set the I2C structure parameters */
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 300000;//100000;
	/* Initialize the I2C peripheral w/ selected parameters */
	I2C_Init(I2C2, &I2C_InitStruct);
	
		/* Enable the I2C peripheral */
	I2C_Cmd(I2C2, ENABLE);

	
	OSFlagCreate(&I2C2_TaskRunFlagGrp,	//创建事件标志组
						"I2C2_TaskRunFlagGrp",
						(OS_FLAGS)0,//清除所有标记
						&err);
}

/**
  * @brief  只用读取的中断
  *         专门为I2C2_TiReadReg_Buf()读取函数所写
  * @param  None
  * @retval None
  */
static uint8_t Ev_SlaveAddr,Ev_Len;
static uint16_t Ev_Addr;
static uint8_t *Ev_Buf;
static uint8_t Ev_Index=0;
void I2C2_EV_IRQHandler(void)
{
	OS_ERR err;
	uint16_t sr1_flag=I2C2->SR1;
	uint16_t sr2_flag=I2C2->SR2;//连续读取SR1和SR2可清除标志
	
	if(sr1_flag&0x0001)//SB: Start bit (Master mode)
	{
		if(Ev_Index==0)
		{
			/* 发送器件地址 */
			I2C_Send7bitAddress(I2C2,Ev_SlaveAddr, I2C_Direction_Transmitter);
			Ev_Index=1;
		}
		else if(Ev_Index==3)
		{
			/* 发送器件地址*/
			I2C_Send7bitAddress(I2C2, Ev_SlaveAddr, I2C_Direction_Receiver);
			Ev_Index=4;
		}
	}
	else if(sr1_flag&0x0002)//ADDR: Address sent (master mode)
	{
		if(Ev_Index==1)
		{
			/*发送寄存器地址*/
			I2C2->DR=Ev_Addr;//I2C_SendData(I2C2, (uint8_t)(Ev_Addr));
			I2C2->CR2 |= I2C_IT_BUF;//I2C_ITConfig(I2C2, I2C_IT_BUF, ENABLE);
			Ev_Index=2;
		}
		else if(Ev_Index==4)
		{
			if (Ev_Len == 1)
			{
				/* 不需要应答了 */
				I2C2->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);//I2C_AcknowledgeConfig(I2C2, DISABLE);
				/* 接收完成，停止 */
				I2C2->CR1 |= I2C_CR1_STOP;//I2C_GenerateSTOP(I2C2, ENABLE);
			}
			I2C2->CR2 |= I2C_IT_BUF;//I2C_ITConfig(I2C2, I2C_IT_BUF, ENABLE);
			Ev_Index=5;
		}
	}
	else if(sr1_flag&0x0040)//RxNE: Data register not empty (receivers)
	{
		if(Ev_Index>4)
		{
			*Ev_Buf=(uint8_t)I2C2->DR;//I2C_ReceiveData(I2C2);
			if(Ev_Len){Ev_Buf++;Ev_Len--;}
			if (Ev_Len == 1)
			{
				/* 不需要应答了 */
				I2C2->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);//I2C_AcknowledgeConfig(I2C2, DISABLE);
				/* 接收完成，停止 */
				I2C2->CR1 |= I2C_CR1_STOP;//I2C_GenerateSTOP(I2C2, ENABLE);
			}
			if(Ev_Len==0)//接收完成
			{
				OSFlagPost(&I2C2_TaskRunFlagGrp,//完成
					(OS_FLAGS)0x01,
					OS_OPT_POST_FLAG_SET,
					&err);
				I2C2->CR2 &= (uint16_t)~I2C_IT_BUF;//I2C_ITConfig(I2C2, I2C_IT_BUF, DISABLE);
				Ev_Index=0;
			}
		}
	}
	else if(sr1_flag&0x0080)//TxE: Data register empty (transmitters)
	{
		if(Ev_Index==2)
		{
			//I2C2->SR1 |= (uint16_t)0x0400;
			/* Generate the Start Condition */
			I2C2->CR1 |= I2C_CR1_START;//I2C_GenerateSTART(I2C2, ENABLE);
			I2C2->CR2 &= (uint16_t)~I2C_IT_BUF;//I2C_ITConfig(I2C2, I2C_IT_BUF, DISABLE);
			Ev_Index=3;
		}
	}
}


/**
  * @brief  I2C2读函数，对多个寄存器连续读取
  *         通过中断读取,不占用时间
  * @param  SlaveAddr:器件地址
  * @param  Addr:寄存器起始地址
  * @param  len:读取的寄存器个数
	* @param  *buf:数据缓存，将读取出来的数据存在这个指针数组里
  * @retval 返回1时成功，返回0x00失败
  */
uint8_t I2C2_TiReadReg_Buf(u8 SlaveAddr,uint16_t Addr,u8 len,u8 *buf)
{
	OS_ERR err,err2;
	CPU_TS ts;
	Ev_SlaveAddr=SlaveAddr;
	Ev_Addr=Addr;
	Ev_Len=len;
	Ev_Buf=buf;
	Ev_Index=0;
	I2C2->CR2 |= I2C_IT_EVT;//I2C_ITConfig(I2C2,I2C_IT_EVT,ENABLE);
	I2C2->CR1 |= I2C_CR1_START;//I2C_GenerateSTART(I2C2, ENABLE);
	OSFlagPend(&I2C2_TaskRunFlagGrp,//等待
					(OS_FLAGS)0x01,			//使用第0位作为标记事件
					2,
					OS_OPT_PEND_BLOCKING+OS_OPT_PEND_FLAG_SET_ANY,
					&ts,
					&err);
	OSFlagPost(&I2C2_TaskRunFlagGrp,//清除标记
					(OS_FLAGS)0x01,
					OS_OPT_POST_FLAG_CLR,
					&err2);
	I2C2->CR2 &= (uint16_t)~(I2C_IT_EVT|I2C_IT_BUF);//I2C_ITConfig(I2C2,I2C_IT_EVT|I2C_IT_BUF,DISABLE);
	I2C2->CR1 |= I2C_CR1_ACK;//I2C_AcknowledgeConfig(I2C2, ENABLE);
	if(err==OS_ERR_TIMEOUT)
	{
		I2C_ErrorCount++;
		return 0;//失败
	}
	else
		return 1;//成功
}

// void I2C2_ER_IRQHandler(void)
// {
// 	
// }

/**
  * @brief  I2C2写寄存器函数，对单个寄存器写
  *         
  * @param  SlaveAddr:器件地址
  * @param  Addr:寄存器地址
  * @param  Data:写入数据
  * @retval 返回0x00时写入失败
  */
uint8_t I2C2_WriteReg(u8 SlaveAddr,uint16_t Addr, uint8_t Data)
{
  uint32_t timeout = TIMEOUT_MAX;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, SlaveAddr, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, (uint8_t)(Addr));

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* Send Data */
  I2C_SendData(I2C2, Data);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* Send I2C2 STOP Condition */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* If operation is OK, return 0 */
  return 1;
}

/*
输入：器件地址，寄存器地址
返回：错误时返回0xff，正常时返回数据
*/
/**
  * @brief  I2C2读寄存器函数，对单个寄存器读
  *         
  * @param  SlaveAddr:器件地址
  * @param  Addr:寄存器地址
  * @retval 返回其的为数据，返回0x00失败
  */
uint8_t I2C2_ReadReg(u8 SlaveAddr,uint16_t Addr)
{
  uint32_t timeout = TIMEOUT_MAX;
  uint8_t Data = 0;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* 发送器件地址 */
  I2C_Send7bitAddress(I2C2,SlaveAddr+1, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /*发送寄存器地址*/
  I2C_SendData(I2C2, (uint8_t)(Addr));

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* Clear AF flag if arised */
  I2C2->SR1 |= (uint16_t)0x0400;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* 发送器件地址*/
  I2C_Send7bitAddress(I2C2, SlaveAddr+1, I2C_Direction_Receiver);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* Prepare an NACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, DISABLE);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Receive the Data */
  Data = I2C_ReceiveData(I2C2);

  /* return the read data */
  return Data;
}

/**
  * @brief  I2C2读函数，对多个寄存器连续读取
  *         
  * @param  SlaveAddr:器件地址
  * @param  Addr:寄存器起始地址
  * @param  len:读取的寄存器个数
	* @param  *buf:数据缓存，将读取出来的数据存在这个指针数组里
  * @retval 返回1时成功，返回0x00失败
  */
uint8_t I2C2_ReadReg_Buf(u8 SlaveAddr,uint16_t Addr,u8 len,u8 *buf)
{
  uint32_t timeout = TIMEOUT_MAX;

  /* 启动 */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* 发送器件地址 */
  I2C_Send7bitAddress(I2C2,SlaveAddr+1, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /*发送寄存器地址*/
  I2C_SendData(I2C2, (uint8_t)(Addr));

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* Clear AF flag if arised */
  I2C2->SR1 |= (uint16_t)0x0400;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

  /* 发送器件地址*/
  I2C_Send7bitAddress(I2C2, SlaveAddr+1, I2C_Direction_Receiver);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0x00;
  }

	while (len) 
	{
		if (len == 1)
		{
			/* 不需要应答了 */
			I2C_AcknowledgeConfig(I2C2, DISABLE);
			/* 接收完成，停止 */
			I2C_GenerateSTOP(I2C2, ENABLE);
		}
		/* Test on I2C2 EV7 and clear it */
		timeout = TIMEOUT_MAX; /* Initialize timeout value */
		while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			/* If the timeout delay is exeeded, exit with error code */
			if ((timeout--) == 0) 
				return 0x00;
		}
		*buf = I2C_ReceiveData(I2C2);
		buf++;
		len--;
	}
	
  /* 启动自动应答 */
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	
  /* return the read data */
  return 1;
}
