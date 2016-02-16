/**
	******************************************************************************
	* @file    AP_PWM.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   ң�ؽ�����PWM���񣬵��PWM��������
	*ң�ؽ������ŷֲ���(ע�⣺ң�ؽ��ջ��ϵ�ͨ��2Ϊ���ţ�ͨ��3ΪYAW���ͳ�����ĸպ��෴��������Ϊ�˱�д���������)
	*{ TIM2, GPIOA, GPIO_Pin_0, TIM_Channel_1, TIM2_IRQn, 0, },		//0		 ROLL
	*{ TIM2, GPIOA, GPIO_Pin_1, TIM_Channel_2, TIM2_IRQn, 0, },		//1		 PITCH
	*{ TIM3, GPIOA, GPIO_Pin_6, TIM_Channel_1, TIM3_IRQn, 0, },		//2		 YAW	 
	*{ TIM2, GPIOA, GPIO_Pin_2, TIM_Channel_3, TIM3_IRQn, 0, },		//3		 THROTTLE
	*{ TIM3, GPIOA, GPIO_Pin_7, TIM_Channel_2, TIM3_IRQn, 0, },   //4		 AUX1
	*{ TIM3, GPIOB, GPIO_Pin_0, TIM_Channel_3, TIM3_IRQn, 0, },   //5		 AUX2
	*{ TIM3, GPIOB, GPIO_Pin_1, TIM_Channel_4, TIM3_IRQn, 0, },   //6		 AUX3

	*	���PWM������400HZ											(���ߺ�NAZA��ͬ)								����			����		����
	*	{ TIM1, GPIOA, GPIO_Pin_8, TIM_Channel_1, TIM1_CC_IRQn	},  // M0			����		����	 	���Ͽ�ʼ��ʱ��	   9
	*	{ TIM1, GPIOA, GPIO_Pin_11, TIM_Channel_4, TIM1_CC_IRQn },  // M1			�Ҷ��		����					 10
	*	{ TIM4, GPIOB, GPIO_Pin_6, TIM_Channel_1, TIM4_IRQn			},  // M2			����		����			   11
	*	{ TIM4, GPIOB, GPIO_Pin_7, TIM_Channel_2, TIM4_IRQn			},  // M3			�ҵ��		����						3
	*	{ TIM4, GPIOB, GPIO_Pin_8, TIM_Channel_3, TIM4_IRQn			},  // M4												����6��ʱ����
	*	{ TIM4, GPIOB, GPIO_Pin_9, TIM_Channel_4, TIM4_IRQn			},  // M5												����6��ʱ����

	******************************************************************************
**/
#include "AP_PWM.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
static int16_t Rc_CaptureVal[7];
static volatile uint32_t *Motor_Time_CCR[6];//���ͨ�������ͨ������ȽϼĴ���ָ��
static volatile uint32_t *Servo_Time_CCR[2];//���	   0��1�Ǹ��������õ�

/* function prototypes -----------------------------------------------*/
static void PWM_RcCaptureCallback(u8 ch,u16 capture);

/**
  * @brief  ң�ؽ��ջ�PWM��������ʼ��
  *         ʹ��TIM2��TIM3���ж�
  * @param  None
  * @retval None
  */
void PWM_RcCaptureInit(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_TIM2TIM3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_TIM2TIM3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	//��ʱ��2NVIC
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Init(&NVIC_InitStructure); //��ʱ��3NVIC

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffff - 1;//�Զ���װ
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // HCLK=168M����ʱ����Ƶ��1M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	//��ʱ��2��ʼ��
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	//��ʱ��3��ʼ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;//�������½��ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);	//ͨ����ʼ��
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_Cmd(TIM2, ENABLE);//��ʱ����ʼ��
	TIM_Cmd(TIM3, ENABLE);
	
	
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);//ͨ���ж�ʹ��
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
}

/**
  * @brief  ��ʱ��2���жϳ���
  *         
  * @param  
  * @retval 
  */
void TIM2_IRQHandler(void)
{
	//OSIntEnter();
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) 
	{
    //TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		TIM2->SR = (uint16_t)~TIM_IT_CC1;
		PWM_RcCaptureCallback(0,TIM2->CCR1);//��0ң��ͨ����ȡ����ֵ		ROLL
  }
	else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET) 
	{
		//TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		TIM2->SR = (uint16_t)~TIM_IT_CC2;
    PWM_RcCaptureCallback(1,TIM2->CCR2);//��1ң��ͨ����ȡ����ֵ		PITCH
  }
	else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) 
	{
		//TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		TIM2->SR = (uint16_t)~TIM_IT_CC3;
    PWM_RcCaptureCallback(3,TIM2->CCR3);//��3ң��ͨ����ȡ����ֵ		thro
  }
	//OSIntExit();
}

/**
  * @brief  ��ʱ��3���жϳ���
  *         
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	//OSIntEnter();
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
	{
    //TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM3->SR = (uint16_t)~TIM_IT_CC1;
		PWM_RcCaptureCallback(2,TIM3->CCR1);//��2ң��ͨ����ȡ����ֵ		YAW
  }
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET) 
	{
		//TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		TIM3->SR = (uint16_t)~TIM_IT_CC2;
    PWM_RcCaptureCallback(4,TIM3->CCR2);//��4ң��ͨ����ȡ����ֵ		AUX1
  }
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET) 
	{
		//TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		TIM3->SR = (uint16_t)~TIM_IT_CC3;
    PWM_RcCaptureCallback(5,TIM3->CCR3);//��5ң��ͨ����ȡ����ֵ		AUX2
  }
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC4) == SET) 
	{
		//TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		TIM3->SR = (uint16_t)~TIM_IT_CC4;
    PWM_RcCaptureCallback(6,TIM3->CCR4);//��6ң��ͨ����ȡ����ֵ	  AUX3
  }
	Rc_LostCnt=0;//�������
	//OSIntExit();
}

/*�жϺ�������ô˺����õ�����ֵ
ch������������Χ��0-5,Ҳ����ͨ������0��ʼ*/
static void PWM_RcCaptureCallback(u8 ch,u16 capture)
{
	static u8 state[7] = {0,0,0,0,0,0,0};	//���
	static u16 rise[7] = {0,0,0,0,0,0,0};	//��¼������ʱ��ֵ
	static u16 fall[7] = {0,0,0,0,0,0,0};	//��¼�½���ʱ��ֵ
	if(state[ch]==0)
	{
		rise[ch]=capture;
		state[ch]=1;
	}
	else
	{
		fall[ch]=capture;
		Rc_CaptureVal[ch]=fall[ch]-rise[ch];//�õ���ͨ���Ĳ���ֵ
		if(Rc_CaptureVal[ch]<3000)
			state[ch]=0;
		else//���ִ�������
		{
			rise[ch]=capture;
			state[ch]=1;
		}
	}
}

/**
  * @brief  ��ȡ���ջ�ͨ��ֵ
  *         
  * @param  channel:ͨ��
  * @retval ��Ӧֵ
  */
int16_t PWM_RcValRead(uint8_t channel)
{
  return Rc_CaptureVal[channel];
}

/*
*********************************************************************************************************
*                                         ���(Esc)PWM����
*********************************************************************************************************
*/
/**
  * @brief  ������������ʼ��
  *         
  * @param  None
  * @retval None
  */
void PWM_EscPwmInit(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (1000000/400 - 1);//���Ƶ��400HZ
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // all TIM on F1 runs at 72MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	if(cfg.mixerConfiguration!=MULTITYPE_BI)//��������ᣬ����Ҫ����ʱ��1������������Ϊ����ģ�Ƶ��Ϊ50HZ
	{
		TIM_TimeBaseStructure.TIM_Prescaler = 168 - 1; // all TIM on F1 runs at 72MHz
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	}
	else
	{
		TIM_TimeBaseStructure.TIM_Period = (1000000/50 - 1);//���Ƶ��50HZ	
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	}

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	
	if(cfg.mixerConfiguration==MULTITYPE_HEX6X)//��������ᣬ��࿪������PWM
	{	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
	}
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 1000;	//1ms pulse width
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);			//TIM1 CH1
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);			//TIM1 CH4
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);			//TIM4 CH1
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);			//TIM4 CH2
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	if(cfg.mixerConfiguration==MULTITYPE_HEX6X)
	{	//�����6�ᣬ��࿪����ʱ��4��3��4ͨ��
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);			//TIM4 CH3
	  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);			//TIM4 CH4
	  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	
	}

	TIM_CtrlPWMOutputs(TIM1, ENABLE);	//TIM1�����ʹ��
	
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	if(cfg.mixerConfiguration==MULTITYPE_BI)//���������
  {
		Motor_Time_CCR[0]=&TIM4->CCR1;
		Motor_Time_CCR[1]=&TIM4->CCR2;
		Servo_Time_CCR[0]=&TIM1->CCR1;
		Servo_Time_CCR[1]=&TIM1->CCR4;
	}	
	else if(cfg.mixerConfiguration==MULTITYPE_QUADX)  //����
	{
		Motor_Time_CCR[0]=&TIM1->CCR1;
		Motor_Time_CCR[1]=&TIM1->CCR4;
		Motor_Time_CCR[2]=&TIM4->CCR1;
		Motor_Time_CCR[3]=&TIM4->CCR2;
	}
	else if(cfg.mixerConfiguration==MULTITYPE_HEX6X)//6��
	{
		Motor_Time_CCR[0]=&TIM1->CCR1;
		Motor_Time_CCR[1]=&TIM1->CCR4;
		Motor_Time_CCR[2]=&TIM4->CCR1;
		Motor_Time_CCR[3]=&TIM4->CCR2;
		Motor_Time_CCR[4]=&TIM4->CCR3;
		Motor_Time_CCR[5]=&TIM4->CCR4;
	}
}

/**
  * @brief  д����PWMռ�ձ�ֵ
  *         
  * @param  index:���PWM��ţ�0~5
  * @param  value:ֵ��1000~2000
  * @retval 
  */
void PWM_WriteMotor(uint8_t index, uint16_t value)
{
	if (index < 6)
	{
		*Motor_Time_CCR[index] = value;
	}
}

/**
  * @brief  д����PWM�����ź�ֵ
  *         
  * @param  index:�����ţ�0~1
  * @param  value:ֵ��1000~2000
  * @retval None
  */
void PWM_WriteServo(uint8_t index, uint16_t value)
{
	if(index<2)
		*Servo_Time_CCR[index] = value;
}
