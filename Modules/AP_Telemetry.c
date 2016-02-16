/**
	******************************************************************************
	* @file    AP_Telemetry.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   无线数据传输，与地面站联系
	******************************************************************************
**/
#include "AP_Telemetry.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
/* OS -----------------------------------------------*/
static  void  Telemetry_MainTask (void *p_arg);
static  OS_TCB  Telemetry_MainTaskTCB;
static  CPU_STK  Telemetry_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/

/**
  * @brief  初始化
  *         创建任务
  * @param  None
  * @retval None
  */
void Telemetry_Init(void)
{
	OS_ERR	err;
#ifdef XBEEPRO
	XbeePro_Init();
#endif
	OSTaskCreate((OS_TCB   *)&Telemetry_MainTaskTCB,  
				 (CPU_CHAR     *)"Telemetry_MainTask",
				 (OS_TASK_PTR   )Telemetry_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_TELEMETRY,
				 (CPU_STK      *)&Telemetry_MainTaskStk[0],
				 (CPU_STK_SIZE  )Telemetry_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}

/**
  * @brief  Telemetry主任务
  *         
  * @param  None
  * @retval None
  */
static  void  Telemetry_MainTask (void *p_arg)
{
	OS_ERR	err;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(20,OS_OPT_TIME_DLY,&err);
#ifdef XBEEPRO
		XbeePro_Process();
#endif
  }
}



