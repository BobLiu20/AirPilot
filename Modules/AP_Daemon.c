/**
	******************************************************************************
	* @file    AP_Daemon.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   �ػ�����
	*					 ĬĬ�ĺǻ��Żһ��İ�ȫ
	******************************************************************************
**/
#include "AP_Daemon.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
uint16_t Rc_LostCnt;//ң���źŶ�ʧ����
uint16_t Handle_LostCnt;//�ֱ��źŶ�ʧ����

uint16_t LED_FlashBit;//��¼����Ҫ��˸��λ
uint8_t NotOkToArmBit;//��ĳһλΪ1ʱ���򲻿ɽ���
uint8_t NeedToWriteFlash;//����ֵΪ1ʱ����Ҫд��FLASH����

/* OS -----------------------------------------------*/
static  void  Daemon_MainTask (void *p_arg);
static  OS_TCB  Daemon_MainTaskTCB;
static  CPU_STK  Daemon_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
static void Daemon_Rc(void);
static void Daemon_Handle(void);
static void LED_ActFlash(void);
static void LED_GPSFlash(void);
static void Daemon_Armed(void);


/**
  * @brief  ��ʼ��
  *         ��������
  * @param  None
  * @retval None
  */
void Daemon_Init(void)
{
	OS_ERR	err;
	Battery_Init();//�����ⶨ
	OSTaskCreate((OS_TCB   *)&Daemon_MainTaskTCB,  
				 (CPU_CHAR     *)"Daemon_MainTask",
				 (OS_TASK_PTR   )Daemon_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_DAEMON ,
				 (CPU_STK      *)&Daemon_MainTaskStk[0],
				 (CPU_STK_SIZE  )Daemon_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}

/**
  * @brief  IMU������
  *         
  * @param  None
  * @retval None
  */
static  void  Daemon_MainTask (void *p_arg)
{
	OS_ERR	err;
	CPU_SR_ALLOC();
  (void)p_arg;
  while (1)
	{
		OSTimeDly(20,OS_OPT_TIME_DLY,&err);
		Daemon_Handle();//�ֱ��źŶ�ʧ�ػ�
		Daemon_Rc();		//ң���źŶ�ʧ����
		LED_ActFlash(); //LED1
		LED_GPSFlash(); //LED2
		Daemon_Armed(); //����������
		Battery_Read();//��ȡ����
		if(NeedToWriteFlash)//������д��FLASH
		{
			OSSchedLock(&err);//��������
// 			I2C2->CR2 &= (uint16_t)~(I2C_IT_EVT|I2C_IT_BUF);//�ر�I2C�ж�
// 			I2C2->CR1 |= I2C_CR1_ACK;//ʹ���Զ�Ӧ��
			CPU_CRITICAL_ENTER();
			Flash_WriteParams(1);//δ֪ԭ���ڽ���FLASH����ʱ����ر�I2C�ж�
			CPU_CRITICAL_EXIT();
			OSSchedUnlock(&err);
			NeedToWriteFlash=0;
		}
  }
}

/**
  * @brief  ң�����ź��ػ�
  *         20ms����һ��
  * @param  None
  * @retval None
  */
static void Daemon_Rc(void)
{
	uint8_t i;
	if(Feature(FEATURE_FAILSAFE))
	{
		if(Rc_LostCnt > (5 * cfg.failsafe_delay))
		{
			f.RC_AVAILABLE=0;//������ң�ؽ��ջ�
			f.RC_CTRL_MODE=0;//��ң����ģʽ
			if(f.HANDLE_AVAILABLE)
			{
				f.HANDLE_CTRL_MODE=1;//�����ֱ�ʱǿ�ƽ����ֱ�ģʽ
				return ;
			}
			
			if(f.ARMED)//ʧȥ�źų����趨ֵ���������趨��ָ��ֵ
			{
				if(Sensors(SENSOR_ACC))cfg.activate[BOXACC]=0xffff;//������ѹ��ʱǿ�ƿ�������ģʽ
				for(i=0;i<3;i++)
					RC_Data[i] = cfg.midrc;	//��ROLL,PITCH,YAWǿ��Ϊ1500�����м�
				RC_Data[THROTTLE] = cfg.failsafe_throttle;//ǿ�ƽ�ʧЧ������������ֵ��ֵ
				if(Rc_LostCnt > (5 * (cfg.failsafe_delay + cfg.failsafe_off_delay)))//��ʧ�źų�����½ʱ�䣬��رյ��,��Ϊ�Լ�����½
				{
					f.ARMED = 0;		//����
					NOTOKTOARM_SET(NOTARMBIT_LOSTRC);//���������
				}
			}
			else if(!f.ARMED)//δ����״̬�¶�ʧ�ź�
			{
				f.ARMED = 0;		//����
				NOTOKTOARM_SET(NOTARMBIT_LOSTRC);//���������
			}
		}
		else
		{
			f.RC_AVAILABLE=1;//����ң�ؽ��ջ�
			NOTOKTOARM_CLR(NOTARMBIT_LOSTRC);//�����ֹ����λ
		}
		Rc_LostCnt++;	//��������ۼӣ���ң�ؽ������ź��ж�ʵʱ��0
	}
}

/**
  * @brief  �ֱ��ź��ػ�
  *         20ms����һ��
  * @param  None
  * @retval None
  */
static void Daemon_Handle(void)
{
	uint8_t i;
	if(Handle_LostCnt > 50)//����1�����ź�
	{
		f.HANDLE_AVAILABLE=0;//�������ֱ�����
		f.HANDLE_CTRL_MODE=0;//���ֱ�����
		if(f.RC_AVAILABLE)
		{
			f.RC_CTRL_MODE=1;//����ң������ǿ�ƽ���ң��ģʽ
		}
	}
	else
	{
		f.HANDLE_AVAILABLE=1;//����ң�ؽ��ջ�
		NOTOKTOARM_CLR(NOTARMBIT_LOSTRC);//�����ֹ����λ
	}
	Handle_LostCnt++;	//��������ۼӣ����ֱ����մ�ʵʱ��0
}

/**
  * @brief  LED_ACTָʾ��
  *         20ms����һ��
  * @param  
  * @retval 
  */
static void LED_ActFlash(void)
{
	static uint8_t count=0;
	if(LED_FlashBit & LEDFLASH_MAGCAL)//У׼�����У�������˸
	{
		if(((count++)%5)==0)	//100ms
			LED_Toggle(LED_ACT);
	}
	else if(LED_FlashBit & LEDFLASH_SENSORCAL)//У׼���ٶȻ�������
	{
		if(((count++)%3)==0)	//60ms
			LED_Toggle(LED_ACT);
	}
	else if(LED_FlashBit & LEDFLASH_AIRSLOPE)//����ǰ�����������б,���߼��ٶ�δУ׼
	{
		if(((count++)%25)==0)	//500ms
			LED_Toggle(LED_ACT);
	}
	else if(LED_FlashBit & LEDFLASH_ARMD)//����״̬������
	{
		LED_On(LED_ACT);
	}
	else
	{
		LED_Off(LED_ACT);
	}
}

/**
  * @brief  GPSָʾ��
  *         ������������6��ʱ��LED������������˸��Ӧ�Ĵ���������2������������˸����
	*					6��������ֱ�ӳ�����
  * @param  
  * @retval 
  */
static void LED_GPSFlash(void)
{
	static uint8_t count=0;
	
	if(GPS_numSat==0)GPS_numSat=1;
	
	if(GPS_numSat>5)//���ǳ���5�ž�ֱ�ӳ�����
	{
		LED_On(LED_GPS);
		return;
	}
	count++;
	if(count%5==0)
	{
		if(count/5<GPS_numSat*2)
			LED_Toggle(LED_GPS);
		else
			LED_Off(LED_GPS);
	}
	else if(count>(2000/20))//3s
		count=0;
}

/**
  * @brief  ���������������Ƿ��������
  *         
  * @param  None
  * @retval None
  */
static void Daemon_Armed(void)
{
// 	if(abs(angle[ROLL])>1000||abs(angle[PITCH])>1000)		//��������100��֤���ɻ�ʧ�£����ϰѵ�����˲��������Է�ֹ�ɻ����˻��ڵ���ת
// 	{
// 		f.ARMED = 0;		//����
// 		NOTOKTOARM_SET(NOTARMBIT_ANGLE100);//���������
// 	}
// 	else
		NOTOKTOARM_CLR(NOTARMBIT_ANGLE100);//�ͷŽ�ֹ����λ
	
	if(f.ACC_CALIBRATED)//����ǰ��������б����25�ȣ����������
		NOTOKTOARM_CLR(NOTARMBIT_ANGLE25);
	else
		NOTOKTOARM_SET(NOTARMBIT_ANGLE25);
	
	if(LED_FlashBit&(LEDFLASH_MAGCAL|LEDFLASH_SENSORCAL))//�����ڽ��д�����У׼ʱ�����������
		NOTOKTOARM_SET(NOTARMBIT_CALING);
	else
		NOTOKTOARM_CLR(NOTARMBIT_CALING);
	
	if(I2C_ErrorCount>100)							//I2C����������100�����������
		NOTOKTOARM_SET(NOTARMBIT_I2CERR);
	else
		NOTOKTOARM_CLR(NOTARMBIT_I2CERR);

	if(NotOkToArmBit)//����λ��Ϊ0�����������
		f.OK_TO_ARM=0;
	else
		f.OK_TO_ARM=1;//�������
}
