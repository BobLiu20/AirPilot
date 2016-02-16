#include "System.h"

/*ȫ�ֱ�����*/


/*��ȡ��ǰϵͳʱ���,��λʱ��Ϊus*/
uint32_t OutTickCount=0;
uint32_t TimUsGet(void)
{
    register uint32_t ms, cycle_cnt;
    do 
		{
        ms = OutTickCount;//�OSTickCtr;
        cycle_cnt = SysTick->VAL;
    } while (ms != OutTickCount);
    return (ms * 1000) + (168000 - cycle_cnt) / 168;
}

uint32_t TimMsGet(void)
{
	return OutTickCount;
}

/*��ʱ��������λΪ1Us
* ˵�����������ô���ʱ��ֻ������us��ʱʱʹ�ã�ms�����ϵ���OS�Դ�����ʱ
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
