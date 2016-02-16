/**
	******************************************************************************
	* @file    AP_Mixer.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   �����������ֵ������Ͷ�����ƣ����ݷ���������
	*          �������
	******************************************************************************
**/
#include "AP_Mixer.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
static uint8_t NumberMotor = 4;
int16_t Motor_Val[6];	 //���
int16_t Servo_Val[2] = { 1500, 1500};//���

/* OS -----------------------------------------------*/
static  void  Mixer_MainTask (void *p_arg);
static  OS_TCB  Mixer_MainTaskTCB;
static  CPU_STK  Mixer_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

OS_FLAG_GRP Mixer_TaskRunFlagGrp;	//���������¼���־��

/* function prototypes -----------------------------------------------*/
static void Mixer_WriteServos(void);
static void Mixer_WriteMotors(void);
static void Mixer_Compute(void);


/**
  * @brief  ��ʼ��
  *         ��������
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
	PWM_EscPwmInit();//��ʼ��PWM
	Mixer_WriteAllMotors(cfg.mincommand);//ֹͣ���е��
	
	OSFlagCreate(&Mixer_TaskRunFlagGrp,	//�����¼���־��
							"Mixer_TaskRunFlagGrp",
							(OS_FLAGS)0,//������б��
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
  * @brief  Mixer������
  *         �������
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
		OSFlagPend(&Mixer_TaskRunFlagGrp,//�ȴ�
							(OS_FLAGS)0x01,			//ʹ�õ�0λ��Ϊ����¼�
							0,
							OS_OPT_PEND_BLOCKING+OS_OPT_PEND_FLAG_SET_ANY,
							&ts,
							&err);
		OSFlagPost(&Mixer_TaskRunFlagGrp,//������
							(OS_FLAGS)0x01,
							OS_OPT_POST_FLAG_CLR,
							&err);
 		Mixer_Compute();
		Mixer_WriteServos();
		Mixer_WriteMotors();
  }
}

/**
  * @brief  д����ֵ
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
  * @brief  д����ֵ
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
  * @brief  ���Ե������ͬһֵ
  *         
  * @param  mc,д���ֵ
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
  * @brief  ���ֵ����
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
	{ //��ֹ��YAW������ʱ��YAW��������Ծ
		axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(RC_Command[YAW]), +100 + abs(RC_Command[YAW]));
	}
  switch (cfg.mixerConfiguration) 
	{
		case MULTITYPE_BI:
			    Motor_Val[0] = PIDMIX(+1, 0, 0);        //LEFT
          Motor_Val[1] = PIDMIX(-1, 0, 0);        //RIGHT
					Servo_Val[0]  = constrain(1500 + (-1 * axisPID[YAW]) - axisPID[PITCH], 1020, 2000); //LEFT
					Servo_Val[1]  = constrain(1500 + (-1 * axisPID[YAW]) + axisPID[PITCH], 1020, 2000); //RIGHT    delete it(YAW_DIRECTION),we use -1 to out BI and we change +-
					Servo_Val[0]  =  preservol * 0.95f + (float)Servo_Val[0]*0.05f;//��ͨ�˲�
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
	for(i=1;i<NumberMotor;i++)	 //����ĸ�����е����ֵ��������maxMotor
		if(Motor_Val[i] > maxMotor)
			maxMotor = Motor_Val[i];
	for(i=0;i<NumberMotor;i++)
	{
		if(maxMotor>cfg.maxthrottle) //�����һ��������ֵ���ڵ�����ֵ��������٣����ĸ����ֵͬʱ������ֵͬ�Խ�������
			Motor_Val[i] -=	maxMotor - cfg.maxthrottle;
		Motor_Val[i] = constrain(Motor_Val[i], cfg.minthrottle, cfg.maxthrottle);//��ֵ�޶��������Сֵ��
		if ((RC_Data[THROTTLE]) < cfg.mincheck)//�����Ŵ�����Сʱ 
		{
			if (!Feature(FEATURE_MOTOR_STOP)) //���û�п���������С�����ͣת
				Motor_Val[i] = cfg.minthrottle;	  //���������ת����������Сʱ
      else
				Motor_Val[i] = cfg.mincommand;	  //��������Сʱ�����ͣת
    }
		if (!f.ARMED)						  //û��������ͣת
			Motor_Val[i] = cfg.mincommand;
// 		if(RC_Data[AUX1]<1300)
// 			Motor_Val[5] = 1000;//   �������ᵥ���ͣת����
//		Motor_Val[i] = 1000;
	}
}
