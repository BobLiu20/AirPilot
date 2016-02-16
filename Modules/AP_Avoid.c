/**
	******************************************************************************
	* @file    AP_Avoid.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.8
	* @brief   
	******************************************************************************
**/
#include "AP_Avoid.h"

/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
static PID_DATA Avoid_piddata[4];
static PID_PARAM Avoid_pidparam;


/* OS ----------------------------------------------------------------*/
static  void  Avoid_MainTask (void *p_arg);
static  OS_TCB  Avoid_MainTaskTCB;
static  CPU_STK  Avoid_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
static void Avoid_AngleUpdate(void);
static void Avoid_PID_Init(void);

/**
  * @brief  初始化
  *         创建任务
  * @param  None
  * @retval None
  */
void Avoid_Init(void)
{
	OS_ERR	err;
	SonarAvoid_Init();
	Avoid_PID_Init();
	OSTaskCreate((OS_TCB   *)&Avoid_MainTaskTCB,  
				 (CPU_CHAR     *)"Avoid_MainTask",
				 (OS_TASK_PTR   )Avoid_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_AVOID,
				 (CPU_STK      *)&Avoid_MainTaskStk[0],
				 (CPU_STK_SIZE  )Avoid_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}



/**
  * @brief  IMU主任务
  *         
  * @param  None
  * @retval None
  */
static  void  Avoid_MainTask (void *p_arg)
{
	OS_ERR	err;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(60,OS_OPT_TIME_PERIODIC,&err);
		SonarAvoid_Update();
		Avoid_AngleUpdate();
//		debug[0]=SonarAvoid_Distance[0];
//		debug[1]=Avoid_Angle[0];
//		debug[2]=SonarAvoid_Distance[2];
  }
}

static void Avoid_PID_Init(void)
{
	
	Avoid_pidparam.kP = 0.5f;
	Avoid_pidparam.kI = 0.0f;
	Avoid_pidparam.kD = 0.5f;
	Avoid_pidparam.Imax = 50.0f;
	Avoid_pidparam.apply_lpf = 0;
	
	PID_reset(&Avoid_piddata[0]);
	PID_reset(&Avoid_piddata[1]);
	PID_reset(&Avoid_piddata[2]);
	PID_reset(&Avoid_piddata[3]);
	
}
int16_t Avoid_Angle[4];//前  后  左   右

#define AVOID_HOLDDISTANCE 200

static void Avoid_AngleUpdate(void)
{
	uint8_t i;
	static float Pre_SonarAvoid_Distance[4];
	int16_t err[4]; 
	int16_t D_Term[4];
	static int16_t I_Term[4];
	
	static uint8_t avoid_hold_flag[4];
	
	int16_t Avoid_Error[4];
	uint32_t n_time;
	float dt;
	static uint32_t p_time=0;
	n_time=TimUsGet();
	dt=(n_time-p_time) * 0.000001f;//两次进入的时间间隔，s
	p_time=n_time;
	
	for(i=0;i<4;i++)
	{
		if(f.SONARAVOID_AVAILABLE[i]&&RC_Options[BOXAVOID])
		{
			if(abs(SonarAvoid_Distance[i] - Pre_SonarAvoid_Distance[i])<20)
				SonarAvoid_Distance[i] = Pre_SonarAvoid_Distance[i] * 0.6f + SonarAvoid_Distance[i] * 0.4f;
			else
				SonarAvoid_Distance[i] = Pre_SonarAvoid_Distance[i] * 0.9f + SonarAvoid_Distance[i] * 0.1f;
			Pre_SonarAvoid_Distance[i] = SonarAvoid_Distance[i];
			
			if(SonarAvoid_Distance[i]<AVOID_HOLDDISTANCE)
				avoid_hold_flag[i] = 1;
			else if(SonarAvoid_Distance[i] > AVOID_HOLDDISTANCE + 50)
				avoid_hold_flag[i] = 0;
			if(avoid_hold_flag[i])
			{
				Avoid_Error[i] = constrain(AVOID_HOLDDISTANCE - SonarAvoid_Distance[i] ,0 ,300);
				Avoid_Angle[i] = apply_pid(&Avoid_piddata[i], (float)Avoid_Error[i], dt, &Avoid_pidparam);
				Avoid_Angle[i] = constrain(Avoid_Angle[i],-100 ,100) ;		
			}
			else
			{
				PID_reset(&Avoid_piddata[i]);
				Avoid_Angle[i] = 0;
			}
		}
		else    //超声无效或关闭避障   前次数据清零  pid数据清零
		{
			Pre_SonarAvoid_Distance[i] = 0;   
			PID_reset(&Avoid_piddata[i]);
			Avoid_Angle[i] = 0;
		}
// 
		

		
		
//		if(f.SONARAVOID_AVAILABLE[i]==0||SonarAvoid_Distance[i]>(AVOID_HOLDDISTANCE+50)||RC_Data[AUX2]>1600)
// 		{
// 			I_Term[i] = 0;
// 			Avoid_Angle[i] = 0;
// 			Pre_SonarAvoid_Distance[i] = SonarAvoid_Distance[i];
// 			continue;//超声波无效
// 		}
// 		
// 		
// 		err[i] = AVOID_HOLDDISTANCE - SonarAvoid_Distance[i];
// 		D_Term[i] = SonarAvoid_Distance[i] - Pre_SonarAvoid_Distance[i];
// 		Pre_SonarAvoid_Distance[i] = SonarAvoid_Distance[i];
// //		I_Term[i] += err[i];
// //		I_Term[i] = constrain(I_Term[i] , -100, 100);
// 		
// 		Avoid_Angle[i] = (1.0f * constrain(err[i],-100,100) + 0.0f * I_Term[i] -  0.0f *  D_Term[i] );
// 		
// 		Avoid_Angle[i] = constrain(Avoid_Angle[i],-100,100);
	}

}

