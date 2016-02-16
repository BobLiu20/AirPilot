#include "System.h"

/*全局变量区*/


/*获取当前系统时间戳,单位时间为us*/
uint32_t OutTickCount=0;
uint32_t TimUsGet(void)
{
    register uint32_t ms, cycle_cnt;
    do 
		{
        ms = OutTickCount;//STickCtr;
        cycle_cnt = SysTick->VAL;
    } while (ms != OutTickCount);
    return (ms * 1000) + (168000 - cycle_cnt) / 168;
}

uint32_t TimMsGet(void)
{
	return OutTickCount;
}

/*延时函数，单位为1Us
* 说明：尽量少用此延时，只在少量us延时时使用，ms级以上的用OS自带的延时
*/
void DelayUs(uint32_t us)
{
    uint32_t now = TimUsGet();
    while (TimUsGet() - now < us);
}


void System_FailureMode(uint8_t mode)
{
	OS_ERR  err;
	while (1) 
	{
		OSTimeDly(50,OS_OPT_TIME_DLY,&err);
		LED_Toggle(LED_ACT);
		OSTimeDly(mode,OS_OPT_TIME_DLY,&err);
	}
}
