/**
	******************************************************************************
	* @file    AP_Mixer.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   这里计算出输出值，电机和舵机控制，根据飞行器种类
	*          动力输出
	******************************************************************************
**/
#include "AP_Mixer.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
static uint8_t NumberMotor = 4;
int16_t Motor_Val[6];	 //电机
int16_t Servo_Val[2] = { 1500, 1500};//舵机

/* OS -----------------------------------------------*/
static  void  Mixer_MainTask (void *p_arg);
static  OS_TCB  Mixer_MainTaskTCB;
static  CPU_STK  Mixer_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

OS_FLAG_GRP Mixer_TaskRunFlagGrp;	//任务运行事件标志组

/* function prototypes -----------------------------------------------*/
static void Mixer_WriteServos(void);
static void Mixer_WriteMotors(void);
static void Mixer_Compute(void);


/**
  * @brief  初始化
  *         创建任务
  * @param  None
  * @retval None
  */
void Mixer_Init(void)
{
	OS_ERR	err;
	switch (cfg.mixerConfiguration) 
	{
		case MULTITYPE_BI:
				NumberMotor = 2;
				break;
		case MULTITYPE_QUADX:
				NumberMotor = 4;
				break;
		case MULTITYPE_HEX6X:
				NumberMotor = 6;
				break;
	}
	PWM_EscPwmInit();//初始化PWM
	Mixer_WriteAllMotors(cfg.mincommand);//停止所有电机
	
	OSFlagCreate(&Mixer_TaskRunFlagGrp,	//创建事件标志组
							"Mixer_TaskRunFlagGrp",
							(OS_FLAGS)0,//清除所有标记
							&err);
	
	OSTaskCreate((OS_TCB   *)&Mixer_MainTaskTCB,  
				 (CPU_CHAR     *)"Mixer_MainTask",
				 (OS_TASK_PTR   )Mixer_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_MIXER ,
				 (CPU_STK      *)&Mixer_MainTaskStk[0],
				 (CPU_STK_SIZE  )Mixer_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}

/**
  * @brief  Mixer主任务
  *         控制输出
  * @param  None
  * @retval None
  */
static  void  Mixer_MainTask (void *p_arg)
{
	OS_ERR	err;
	CPU_TS ts;
  (void)p_arg;
  while (1)
	{
		OSFlagPend(&Mixer_TaskRunFlagGrp,//等待
							(OS_FLAGS)0x01,			//使用第0位作为标记事件
							0,
							OS_OPT_PEND_BLOCKING+OS_OPT_PEND_FLAG_SET_ANY,
							&ts,
							&err);
		OSFlagPost(&Mixer_TaskRunFlagGrp,//清除标记
							(OS_FLAGS)0x01,
							OS_OPT_POST_FLAG_CLR,
							&err);
 		Mixer_Compute();
		Mixer_WriteServos();
		Mixer_WriteMotors();
  }
}

/**
  * @brief  写入舵机值
  *         
  * @param  None
  * @retval None
  */
static void Mixer_WriteServos(void)
{
	switch (cfg.mixerConfiguration)
	{
		case MULTITYPE_BI:
			PWM_WriteServo(0, Servo_Val[0]);
			PWM_WriteServo(1, Servo_Val[1]);
			break;
  }
}

/**
  * @brief  写入电机值
  *         
  * @param  None
  * @retval None
  */
static void Mixer_WriteMotors(void)
{
    uint8_t i;
    for (i = 0; i < NumberMotor; i++)
        PWM_WriteMotor(i, Motor_Val[i]);
}

/**
  * @brief  所以电机填入同一值
  *         
  * @param  mc,写入的值
  * @retval None
  */
void Mixer_WriteAllMotors(int16_t mc)
{
    uint8_t i;
    // Sends commands to all Motor_Vals
    for (i = 0; i < NumberMotor; i++)
        Motor_Val[i] = mc;
    Mixer_WriteMotors();
}


/**
  * @brief  输出值计算
  *         
  * @param  None
  * @retval None
  */
#define PIDMIX(R, P, Y) RC_Command[THROTTLE] + axisPID[ROLL] * R + axisPID[PITCH] * P + cfg.yaw_direction * axisPID[YAW] * Y
static void Mixer_Compute(void)
{
	int16_t maxMotor;
	uint8_t i;
	static float preservol;
	static float preservor;
	if(NumberMotor > 3)
	{ //防止当YAW修正的时候“YAW”出现跳跃
		axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(RC_Command[YAW]), +100 + abs(RC_Command[YAW]));
	}
  switch (cfg.mixerConfiguration) 
	{
		case MULTITYPE_BI:
			    Motor_Val[0] = PIDMIX(+1, 0, 0);        //LEFT
          Motor_Val[1] = PIDMIX(-1, 0, 0);        //RIGHT
					Servo_Val[0]  = constrain(1500 + (-1 * axisPID[YAW]) - axisPID[PITCH], 1020, 2000); //LEFT
					Servo_Val[1]  = constrain(1500 + (-1 * axisPID[YAW]) + axisPID[PITCH], 1020, 2000); //RIGHT    delete it(YAW_DIRECTION),we use -1 to out BI and we change +-
					Servo_Val[0]  =  preservol * 0.95f + (float)Servo_Val[0]*0.05f;//低通滤波
					preservol = Servo_Val[0];			
					Servo_Val[1]  =  preservor * 0.95f + (float)Servo_Val[1]*0.05f;
					preservor = Servo_Val[1];
          break;
    case MULTITYPE_QUADX:
					Motor_Val[0] = PIDMIX(-1, -1, +1);      //FRONT_R
					Motor_Val[1] = PIDMIX(+1, -1, -1);      //FRONT_L
					Motor_Val[2] = PIDMIX(+1, +1, +1);      //REAR_L
					Motor_Val[3] = PIDMIX(-1, +1, -1);      //REAR_R
					break;
		case MULTITYPE_HEX6X:
					Motor_Val[0] = PIDMIX(-4/5,-9/10,+1); //FRONT_R
					Motor_Val[1] = PIDMIX(+4/5,-9/10,-1); //FRONT_L 
					Motor_Val[2] = PIDMIX(+4/5 ,+0 ,+1);  //LEFT
					Motor_Val[3] = PIDMIX(+4/5,+9/10,-1); //REAR_L 
					Motor_Val[4] = PIDMIX(-4/5,+9/10,+1); //REAR_R 
					Motor_Val[5] = PIDMIX(-4/5 ,+0 ,-1);  //RIGHT 
					
					break;
	}

	maxMotor = Motor_Val[0];
	for(i=1;i<NumberMotor;i++)	 //求出四个电机中的最大值，保存在maxMotor
		if(Motor_Val[i] > maxMotor)
			maxMotor = Motor_Val[i];
	for(i=0;i<NumberMotor;i++)
	{
		if(maxMotor>cfg.maxthrottle) //如果有一个电机最大值大于电机最大值，即最高速，则四个电机值同时减少相同值以进行修正
			Motor_Val[i] -=	maxMotor - cfg.maxthrottle;
		Motor_Val[i] = constrain(Motor_Val[i], cfg.minthrottle, cfg.maxthrottle);//将值限定在最大最小值内
		if ((RC_Data[THROTTLE]) < cfg.mincheck)//当油门处于最小时 
		{
			if (!Feature(FEATURE_MOTOR_STOP)) //如果没有开启油门最小，电机停转
				Motor_Val[i] = cfg.minthrottle;	  //电机以慢速转，当油门最小时
      else
				Motor_Val[i] = cfg.mincommand;	  //当油门最小时，电机停转
    }
		if (!f.ARMED)						  //没解锁则电机停转
			Motor_Val[i] = cfg.mincommand;
// 		if(RC_Data[AUX1]<1300)
// 			Motor_Val[5] = 1000;//   测试六轴单电机停转开关
//		Motor_Val[i] = 1000;
	}
}
