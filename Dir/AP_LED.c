/**
	******************************************************************************
	* @file    AP_LED.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   LED����
	*					 ��Ҫ���ӻ��߸���LED  IOʱ������AP_LED.h��ֱ�����ã� �������������������
	******************************************************************************
**/
#include "AP_LED.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
GPIO_TypeDef* LED_GPIO_PORTS[LEDn] = {LED_ACTIVITY_GPIO_PORT, LED_GPS_GPIO_PORT};
const uint16_t LED_GPIO_PINS[LEDn] = {LED_ACTIVITY_PIN, LED_GPS_PIN};
const uint32_t LED_GPIO_CLKS[LEDn] = {LED_ACTIVITY_GPIO_CLK, LED_GPS_GPIO_CLK};

/* function prototypes -----------------------------------------------*/

/**
  * @brief  ����LED���ţ���Ҫ����LED IOʱ����AP_LED.h������
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  uint8_t i;
	GPIO_InitTypeDef  GPIO_InitStructure;
	for(i=0;i<LEDn;i++)
	{
		/* Enable the GPIO_LED Clock */
		RCC_AHB1PeriphClockCmd(LED_GPIO_CLKS[i], ENABLE);

		/* Configure the GPIO_LED pin */
		GPIO_InitStructure.GPIO_Pin = LED_GPIO_PINS[i];
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(LED_GPIO_PORTS[i], &GPIO_InitStructure);
	}
}

/**
  * @brief  ��LED
  * @param  Led: ָ�������ĸ�LED
  * @retval None
  */
void LED_On(Led_TypeDef Led)
{
	
	GPIO_SetBits(LED_GPIO_PORTS[Led],LED_GPIO_PINS[Led]);
}

/**
  * @brief  ��LED
  * @param  Led: ָ���ĸ�
  * @retval None
  */
void LED_Off(Led_TypeDef Led)
{
	GPIO_ResetBits(LED_GPIO_PORTS[Led],LED_GPIO_PINS[Led]);
}

/**
  * @brief  ��תLED�������䰵��������
  * @param  Led: ָ���ĸ�
  * @retval None
  */
void LED_Toggle(Led_TypeDef Led)
{
	GPIO_ToggleBits(LED_GPIO_PORTS[Led],LED_GPIO_PINS[Led]);
	
}
