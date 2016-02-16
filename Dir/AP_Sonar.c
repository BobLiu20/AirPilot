/**
	******************************************************************************
	* @file    AP_Sonar.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   超声测距
	*					 当前高度测量超声模块：TRIG:PC9   ECHO:PC8
	******************************************************************************
**/
#include "AP_Sonar.h"
#ifdef SONAR
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
#define TRIG_Pin     GPIO_Pin_9
#define TRIG_Gpio    GPIOC
#define ECHO_Pin     GPIO_Pin_8
#define ECHO_Gpio    GPIOC
#define ECHO_Exti    EXTI9_5_IRQn
#define ECHO_Line    EXTI_Line8
#define ECHO_Source  EXTI_PinSource8
#define ECHO_PS      EXTI_PortSourceGPIOC

#define SONAR_LOSTCNTMAX 5	//13*75ms=1秒，1秒内不能丢失

/* variables ---------------------------------------------------------*/
float EstAlt_Sonar;//超声测距高度,CM

static uint16_t Sonar_LostCnt;//超声波无效计数,小于一定值时超声是有效的


/* function prototypes -----------------------------------------------*/

/**
  * @brief  超声模块控制引脚初始化
  *         需要中断
  * @param  
  * @retval 
  */
void Sonar_Init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//配置外部中断需要的

	/*TRIG控制引脚初始化*/
	GPIO_InitStructure.GPIO_Pin = TRIG_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TRIG_Gpio, &GPIO_InitStructure);
	
	/*ECHO触发引脚中断配置*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = ECHO_Pin;
  GPIO_Init(ECHO_Gpio, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(ECHO_PS, ECHO_Source);

  EXTI_InitStructure.EXTI_Line = ECHO_Line;                  
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;        
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;    
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                  
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = ECHO_Exti;             
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_EXTI9_5; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_EXTI9_5;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  测距中断程序
  *         如果测距成功，则模块会输出声波来回的时间等效的高电平时间
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	static uint32_t timing_start=0;
	uint32_t timing_stop;
	static uint16_t distance_old=0;
	uint16_t distance;
	if(ECHO_Gpio->IDR & ECHO_Pin)//上升沿
	{
		timing_start = TimUsGet();
	}
	else
	{
		timing_stop = TimUsGet();
		if(timing_stop>timing_start)
		{
			distance = (timing_stop - timing_start)/59;//计算出距离，单位CM
			if(distance<200&&distance>0)//距离有效
			{
				EstAlt_Sonar = (float)distance  ;  // * 0.6f+(float)distance_old * 0.4f;
				distance_old = EstAlt_Sonar;
				
				if(Sonar_LostCnt > (SONAR_LOSTCNTMAX * 2))//这些是为了形成一个缓冲，让超声从无效到有效有一个过程
					Sonar_LostCnt = SONAR_LOSTCNTMAX * 2 - 1;
				else if(Sonar_LostCnt >= SONAR_LOSTCNTMAX)
					Sonar_LostCnt-=2;
				else
					Sonar_LostCnt=0;//清除错误计数
			}
		}
	}
	EXTI->PR = ECHO_Line;//EXTI_ClearITPendingBit(ECHO_Line);
}

/**
  * @brief  超声数据更新
  *         调用间隔不少于60ms，声音传播比较慢，你懂得！
	*					建议75ms调用一次
  * @param  None
  * @retval None
  */
void Sonar_Update(void)
{
	OS_ERR err;
	static float Last_EstAlt_Sonar = 0;
	static uint8_t counter = 0;
	GPIO_SetBits(TRIG_Gpio, TRIG_Pin);
	OSTimeDly(1,OS_OPT_TIME_DLY,&err);//不少于10us的高电平
	GPIO_ResetBits(TRIG_Gpio, TRIG_Pin);

	
	if(Sonar_LostCnt++ < SONAR_LOSTCNTMAX)//13*75=1?,1??????
		f.SONAR_AVAILABLE = 1;//???????
	else
	{
		f.SONAR_AVAILABLE = 0;//????????????
		
		if(f.VISION_AVAILABLE && VisionLanding_Altitude!=0)//????????????
		{
			f.SONAR_AVAILABLE = 1;
			EstAlt_Sonar = VisionLanding_Altitude;
			
			
		}
		
// 		if(f.OPTFLOW_AVAILABLE)//???????????,??????????,????
// 		{
// 			f.SONAR_AVAILABLE = 1;
// 			EstAlt_Sonar = OpticalFlow_Altitude;
// 			if(abs(EstAlt_Sonar - Last_EstAlt_Sonar) > 20)
// 			{
// 				counter++;
// 				if(counter<3)
// 				{
// 					EstAlt_Sonar = Last_EstAlt_Sonar;
// 					
// 				}
// 				else
// 					counter = 0;
// 			}
// 			else
// 					counter = 0;
 		//	EstAlt_Sonar = Last_EstAlt_Sonar * 0.6f + EstAlt_Sonar * 0.4f ;
 		//	Last_EstAlt_Sonar = EstAlt_Sonar;
//		}

	}

// 	if(abs(EstAlt_Sonar - Last_EstAlt_Sonar )>20)
// 		EstAlt_Sonar = Last_EstAlt_Sonar * 0.95f + EstAlt_Sonar * 0.05f ;
// 	else
		EstAlt_Sonar = Last_EstAlt_Sonar * 0.6f + EstAlt_Sonar * 0.4f ;
	Last_EstAlt_Sonar = EstAlt_Sonar;
	
}




#endif
