/**
	******************************************************************************
	* @file    AP_SonarAvoid.h
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.8
	* @brief   用于灰机四轴回避障碍物，超声避障
	******************************************************************************
**/
#include "AP_SonarAvoid.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
#define SONAR_LOSTCNTMAX 13	//13*75ms=1秒，1秒内不能丢失

/* variables ---------------------------------------------------------*/
float SonarAvoid_Distance[4];
static uint16_t SonarAvoid_LostCnt[4];//超声波无效计数,小于一定值时超声是有效的

/* function prototypes -----------------------------------------------*/
static void SonarAvoid_DistanceCallback(uint8_t num,uint32_t tim);

/**
  * @brief  初始化
  *         
  * @param  None
  * @retval None
  */
void SonarAvoid_Init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//配置外部中断需要的

	/*TRIG控制引脚初始化*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/*ECHO触发引脚中断配置*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource12);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource13);

  EXTI_InitStructure.EXTI_Line = EXTI_Line10|EXTI_Line11|EXTI_Line12|EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_EXTI15_10; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_EXTI15_10;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  中断
  *         
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	static uint32_t timing_start[4];

	if ((EXTI_GetITStatus(EXTI_Line13) != RESET))		//超声	前
	{
		if(GPIOD->IDR & GPIO_Pin_13)
			timing_start[0] = TimUsGet();
		else
			SonarAvoid_DistanceCallback(0,timing_start[0]);
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
	if ((EXTI_GetITStatus(EXTI_Line12) != RESET))		//超声	后
	{
		if(GPIOD->IDR & GPIO_Pin_12)
			timing_start[1] = TimUsGet();
		else
			SonarAvoid_DistanceCallback(1,timing_start[1]);
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	if ((EXTI_GetITStatus(EXTI_Line11) != RESET))		//超声	左
	{
		if(GPIOD->IDR & GPIO_Pin_11)
			timing_start[2] = TimUsGet();
		else
			SonarAvoid_DistanceCallback(2,timing_start[2]);
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
	if ((EXTI_GetITStatus(EXTI_Line10) != RESET))		//超声	右
	{
		if(GPIOD->IDR & GPIO_Pin_10)
			timing_start[3] = TimUsGet();
		else
			SonarAvoid_DistanceCallback(3,timing_start[3]);
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}

/**
  * @brief  获取距离回调函数
  *         
  * @param  None
  * @retval None
  */
static void SonarAvoid_DistanceCallback(uint8_t num,uint32_t tim)
{
	static u32 timing_stop[4];
	uint16_t distance;
	timing_stop[num] = TimUsGet();
	if(timing_stop[num]>tim)
	{
		distance = (timing_stop[num] - tim)/59;//计算出距离，单位CM
		if(distance<400&&distance>0)//距离有效
		{
			SonarAvoid_Distance[num]=distance;
			
			if(SonarAvoid_LostCnt[num] > (SONAR_LOSTCNTMAX * 2))//这些是为了形成一个缓冲，让超声从无效到有效有一个过程
				SonarAvoid_LostCnt[num] = SONAR_LOSTCNTMAX * 2 - 1;
			else if(SonarAvoid_LostCnt[num] >= SONAR_LOSTCNTMAX)
				SonarAvoid_LostCnt[num]-=2;
			else
				SonarAvoid_LostCnt[num]=0;//清除错误计数
		}
	}
}

/**
  * @brief  超声数据更新
  *         调用间隔不少于60ms，声音传播比较慢，你懂得！
	*					建议75ms调用一次
  * @param  None
  * @retval None
  */
void SonarAvoid_Update(void)
{
	OS_ERR err;
	uint8_t i;
	GPIO_SetBits(GPIOD,GPIO_Pin_14);
	OSTimeDly(1,OS_OPT_TIME_DLY,&err);//不少于10us的高电平
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	
	for(i=0;i<4;i++)//前 后 左 右
	{
		if(SonarAvoid_LostCnt[i]++ < SONAR_LOSTCNTMAX)//13*75=1秒，1秒内不能丢失
			f.SONARAVOID_AVAILABLE[i] = 1;//超声存在且有效
		else
		{
			f.SONARAVOID_AVAILABLE[i] = 0;//测距无效或者不存超声模块
		}
	}
}

