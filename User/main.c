#include "includes.h"

/************************������ƿ�*************************/
static  void  AppTaskStart (void *p_arg);
static  OS_TCB   AppTaskStartTCB;
static  CPU_STK  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

int main(void)
{
	OS_ERR  err;
	OSInit(&err);
	OSTaskCreate((OS_TCB       *)&AppTaskStartTCB,              /* Create the start task                                */
							 (CPU_CHAR     *)"App Task Start",
							 (OS_TASK_PTR   )AppTaskStart, 
							 (void         *)0,
							 (OS_PRIO       )PRIO_APPTASKSTART,
							 (CPU_STK      *)&AppTaskStartStk[0],
							 (CPU_STK_SIZE  )AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10],
							 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
							 (OS_MSG_QTY    )0,
							 (OS_TICK       )0,
							 (void         *)0,
							 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
							 (OS_ERR       *)&err);
	OSStart(&err);
  while (1)
  {
  }
}

float Diy_Test1;//�����õ�

static  void  AppTaskStart (void *p_arg)
{
	OS_ERR      err;
  (void)p_arg;
	/*1ms,1000hz*/
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		while (1);
	}
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//2λ�����ȼ�2λ�����ȼ�
	BSP_Init();
	CPU_Init();
	cfg.acc_lpf_factor = 10;
	cfg.gyro_cmpf_factor = 1500; // default MWC

//	Mem_Init();
//  Math_Init();
	
	UserDefineVariables_Creat("TES1",&Diy_Test1,2);      //�����Զ������
//��һ�����������֣��������ĸ���ĸ���ڶ����Ǳ�����ַ���������Ǿ��ȣ����ж���λС��
  //AppTaskCreate();
  while (1)
	{
		OSTimeDly(1000,OS_OPT_TIME_DLY,&err);
  }
}
