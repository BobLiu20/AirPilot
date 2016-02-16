/**
	******************************************************************************
	* @file    AP_PWM.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   遥控接收器PWM捕获，电调PWM控制驱动
	*遥控接收引脚分布：(注意：遥控接收机上的通道2为油门，通道3为YAW，和程序里的刚好相反，程序是为了编写方便而定的)
	*{ TIM2, GPIOA, GPIO_Pin_0, TIM_Channel_1, TIM2_IRQn, 0, },		//0		 ROLL
	*{ TIM2, GPIOA, GPIO_Pin_1, TIM_Channel_2, TIM2_IRQn, 0, },		//1		 PITCH
	*{ TIM3, GPIOA, GPIO_Pin_6, TIM_Channel_1, TIM3_IRQn, 0, },		//2		 YAW	 
	*{ TIM2, GPIOA, GPIO_Pin_2, TIM_Channel_3, TIM3_IRQn, 0, },		//3		 THROTTLE
	*{ TIM3, GPIOA, GPIO_Pin_7, TIM_Channel_2, TIM3_IRQn, 0, },   //4		 AUX1
	*{ TIM3, GPIOB, GPIO_Pin_0, TIM_Channel_3, TIM3_IRQn, 0, },   //5		 AUX2
	*{ TIM3, GPIOB, GPIO_Pin_1, TIM_Channel_4, TIM3_IRQn, 0, },   //6		 AUX3

	*	电调PWM驱动，400HZ											(接线和NAZA相同)								两轴			四轴		六轴
	*	{ TIM1, GPIOA, GPIO_Pin_8, TIM_Channel_1, TIM1_CC_IRQn	},  // M0			左舵机		右上	 	右上开始逆时针	   9
	*	{ TIM1, GPIOA, GPIO_Pin_11, TIM_Channel_4, TIM1_CC_IRQn },  // M1			右舵机		左上					 10
	*	{ TIM4, GPIOB, GPIO_Pin_6, TIM_Channel_1, TIM4_IRQn			},  // M2			左电机		左下			   11
	*	{ TIM4, GPIOB, GPIO_Pin_7, TIM_Channel_2, TIM4_IRQn			},  // M3			右电机		右下						3
	*	{ TIM4, GPIOB, GPIO_Pin_8, TIM_Channel_3, TIM4_IRQn			},  // M4												当是6轴时启用
	*	{ TIM4, GPIOB, GPIO_Pin_9, TIM_Channel_4, TIM4_IRQn			},  // M5												当是6轴时启用

	******************************************************************************
**/
#include "AP_PWM.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
static int16_t Rc_CaptureVal[7];
static volatile uint32_t *Motor_Time_CCR[6];//电机通道的输出通道捕获比较寄存器指针
static volatile uint32_t *Servo_Time_CCR[2];//舵机	   0和1是给两轴舵机用的

/* function prototypes -----------------------------------------------*/
static void PWM_RcCaptureCallback(u8 ch,u16 capture);

/**
  * @brief  遥控接收机PWM输出捕获初始化
  *         使用TIM2和TIM3，中断
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
  NVIC_Init(&NVIC_InitStructure);	//定时器2NVIC
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Init(&NVIC_InitStructure); //定时器3NVIC

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffff - 1;//自动重装
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // HCLK=168M，此时计数频率1M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	//定时器2初始化
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	//定时器3初始化

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
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;//上升沿下降沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);	//通道初始化
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
	
	TIM_Cmd(TIM2, ENABLE);//定时器初始化
	TIM_Cmd(TIM3, ENABLE);
	
	
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);//通道中断使能
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
}

/**
  * @brief  定时器2的中断程序
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
		PWM_RcCaptureCallback(0,TIM2->CCR1);//第0遥控通道获取捕获值		ROLL
  }
	else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET) 
	{
		//TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		TIM2->SR = (uint16_t)~TIM_IT_CC2;
    PWM_RcCaptureCallback(1,TIM2->CCR2);//第1遥控通道获取捕获值		PITCH
  }
	else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) 
	{
		//TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		TIM2->SR = (uint16_t)~TIM_IT_CC3;
    PWM_RcCaptureCallback(3,TIM2->CCR3);//第3遥控通道获取捕获值		thro
  }
	//OSIntExit();
}

/**
  * @brief  定时器3的中断程序
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
		PWM_RcCaptureCallback(2,TIM3->CCR1);//第2遥控通道获取捕获值		YAW
  }
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET) 
	{
		//TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		TIM3->SR = (uint16_t)~TIM_IT_CC2;
    PWM_RcCaptureCallback(4,TIM3->CCR2);//第4遥控通道获取捕获值		AUX1
  }
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET) 
	{
		//TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		TIM3->SR = (uint16_t)~TIM_IT_CC3;
    PWM_RcCaptureCallback(5,TIM3->CCR3);//第5遥控通道获取捕获值		AUX2
  }
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC4) == SET) 
	{
		//TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		TIM3->SR = (uint16_t)~TIM_IT_CC4;
    PWM_RcCaptureCallback(6,TIM3->CCR4);//第6遥控通道获取捕获值	  AUX3
  }
	Rc_LostCnt=0;//清除计数
	//OSIntExit();
}

/*中断函数里调用此函数得到捕获值
ch这个输入参数范围是0-5,也就是通道数从0开始*/
static void PWM_RcCaptureCallback(u8 ch,u16 capture)
{
	static u8 state[7] = {0,0,0,0,0,0,0};	//标记
	static u16 rise[7] = {0,0,0,0,0,0,0};	//记录上升沿时的值
	static u16 fall[7] = {0,0,0,0,0,0,0};	//记录下降沿时的值
	if(state[ch]==0)
	{
		rise[ch]=capture;
		state[ch]=1;
	}
	else
	{
		fall[ch]=capture;
		Rc_CaptureVal[ch]=fall[ch]-rise[ch];//得到此通道的捕获值
		if(Rc_CaptureVal[ch]<3000)
			state[ch]=0;
		else//出现错误，修正
		{
			rise[ch]=capture;
			state[ch]=1;
		}
	}
}

/**
  * @brief  读取接收机通道值
  *         
  * @param  channel:通道
  * @retval 对应值
  */
int16_t PWM_RcValRead(uint8_t channel)
{
  return Rc_CaptureVal[channel];
}

/*
*********************************************************************************************************
*                                         电调(Esc)PWM驱动
*********************************************************************************************************
*/
/**
  * @brief  电调驱动输出初始化
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
	TIM_TimeBaseStructure.TIM_Period = (1000000/400 - 1);//电调频率400HZ
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // all TIM on F1 runs at 72MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	if(cfg.mixerConfiguration!=MULTITYPE_BI)//如果是两轴，则需要将定时器1的两个口设置为舵机的，频率为50HZ
	{
		TIM_TimeBaseStructure.TIM_Prescaler = 168 - 1; // all TIM on F1 runs at 72MHz
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	}
	else
	{
		TIM_TimeBaseStructure.TIM_Period = (1000000/50 - 1);//舵机频率50HZ	
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
	
	if(cfg.mixerConfiguration==MULTITYPE_HEX6X)//如果是六轴，需多开启两个PWM
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
	{	//如果是6轴，则多开启定时器4的3和4通道
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);			//TIM4 CH3
	  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);			//TIM4 CH4
	  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	
	}

	TIM_CtrlPWMOutputs(TIM1, ENABLE);	//TIM1的输出使能
	
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	if(cfg.mixerConfiguration==MULTITYPE_BI)//如果是两轴
  {
		Motor_Time_CCR[0]=&TIM4->CCR1;
		Motor_Time_CCR[1]=&TIM4->CCR2;
		Servo_Time_CCR[0]=&TIM1->CCR1;
		Servo_Time_CCR[1]=&TIM1->CCR4;
	}	
	else if(cfg.mixerConfiguration==MULTITYPE_QUADX)  //四轴
	{
		Motor_Time_CCR[0]=&TIM1->CCR1;
		Motor_Time_CCR[1]=&TIM1->CCR4;
		Motor_Time_CCR[2]=&TIM4->CCR1;
		Motor_Time_CCR[3]=&TIM4->CCR2;
	}
	else if(cfg.mixerConfiguration==MULTITYPE_HEX6X)//6轴
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
  * @brief  写入电调PWM占空比值
  *         
  * @param  index:输出PWM序号，0~5
  * @param  value:值，1000~2000
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
  * @brief  写入舵机PWM控制信号值
  *         
  * @param  index:舵机序号，0~1
  * @param  value:值，1000~2000
  * @retval None
  */
void PWM_WriteServo(uint8_t index, uint16_t value)
{
	if(index<2)
		*Servo_Time_CCR[index] = value;
}
