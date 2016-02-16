#ifndef __AP_LED_H
#define __AP_LED_H
#include "includes.h"


typedef enum
{
	LED_ACT = 0,	// Blue
	LED_GPS = 1, 	// Amber
} Led_TypeDef;


#define LEDn						2

#define LED_ACTIVITY_PIN			GPIO_Pin_2
#define LED_ACTIVITY_GPIO_PORT		GPIOC
#define LED_ACTIVITY_GPIO_CLK		RCC_AHB1Periph_GPIOC

#define LED_GPS_PIN			GPIO_Pin_3
#define LED_GPS_GPIO_PORT	GPIOC
#define LED_GPS_GPIO_CLK		RCC_AHB1Periph_GPIOC

void LED_Init(void);
void LED_On(Led_TypeDef Led);
void LED_Off(Led_TypeDef Led);
void LED_Toggle(Led_TypeDef Led);


#endif
