/**
	******************************************************************************
	* @file    AP_Sonar.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   �������
	*					 ��ǰ�߶Ȳ�������ģ�飺TRIG:PC9   ECHO:PC8
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

#define SONAR_LOSTCNTMAX 5	//13*75ms=1�룬1���ڲ��ܶ�ʧ

/* variables ---------------------------------------------------------*/
float EstAlt_Sonar;//�������߶�,CM

static uint16_t Sonar_LostCnt;//��������Ч����,С��һ��ֵʱ��������Ч��


/* function prototypes -----------------------------------------------*/

/**
  * @brief  ����ģ��������ų�ʼ��
  *         ��Ҫ�ж�
  * @param  
  * @retval 
  */
void Sonar_Init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//�����ⲿ�ж���Ҫ��

	/*TRIG�������ų�ʼ��*/
	GPIO_InitStructure.GPIO_Pin = TRIG_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TRIG_Gpio, &GPIO_InitStructure);
	
	/*ECHO���������ж�����*/
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
  * @brief  ����жϳ���
  *         ������ɹ�����ģ�������������ص�ʱ���Ч�ĸߵ�ƽʱ��
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	static uint32_t timing_start=0;
	uint32_t timing_stop;
	static uint16_t distance_old=0;
	uint16_t distance;
	if(ECHO_Gpio->IDR & ECHO_Pin)//������
	{
		timing_start = TimUsGet();
	}
	else
	{
		timing_stop = TimUsGet();
		if(timing_stop>timing_start)
		{
			distance = (timing_stop - timing_start)/59;//��������룬��λCM
			if(distance<200&&distance>0)//������Ч
			{
				EstAlt_Sonar = (float)distance  ;  // * 0.6f+(float)distance_old * 0.4f;
				distance_old = EstAlt_Sonar;
				
				if(Sonar_LostCnt > (SONAR_LOSTCNTMAX * 2))//��Щ��Ϊ���γ�һ�����壬�ó�������Ч����Ч��һ������
					Sonar_LostCnt = SONAR_LOSTCNTMAX * 2 - 1;
				else if(Sonar_LostCnt >= SONAR_LOSTCNTMAX)
					Sonar_LostCnt-=2;
				else
					Sonar_LostCnt=0;//����������
			}
		}
	}
	EXTI->PR = ECHO_Line;//EXTI_ClearITPendingBit(ECHO_Line);
}

/**
  * @brief  �������ݸ���
  *         ���ü��������60ms�����������Ƚ������㶮�ã�
	*					����75ms����һ��
  * @param  None
  * @retval None
  */
void Sonar_Update(void)
{
	OS_ERR err;
	static float Last_EstAlt_Sonar = 0;
	static uint8_t counter = 0;
	GPIO_SetBits(TRIG_Gpio, TRIG_Pin);
	OSTimeDly(1,OS_OPT_TIME_DLY,&err);//������10us�ĸߵ�ƽ
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
