/**
	******************************************************************************
	* @file    AP_RC.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   RC(Radio Control)
	*					 ң�����źŽ����봦��
	*					 ��ң�ؽ��ջ���ȡ��ͨ�����ݱ�����RC_RawData[]��
	*					 Ȼ����AP_EasyCtrl.c����д���
	******************************************************************************
**/
#include "AP_RC.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
int16_t RC_RawData[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 }; // interval [1000;2000]//������ң����8��ͨ����ԭʼֵ

/* OS -----------------------------------------------*/
static  void  RC_MainTask (void *p_arg);
static  OS_TCB  RC_MainTaskTCB;
static  CPU_STK  RC_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
static void RC_ComputeRcData(void);
static void RC_DataSynth(void);

/**
  * @brief  ��ʼ��
  *         ��������
  * @param  None
  * @retval None
  */
void RC_Init(void)
{
	OS_ERR	err;
	PWM_RcCaptureInit();//��ʼ��
	OSTaskCreate((OS_TCB   *)&RC_MainTaskTCB,
				 (CPU_CHAR     *)"RC_MainTask",
				 (OS_TASK_PTR   )RC_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_RC ,
				 (CPU_STK      *)&RC_MainTaskStk[0],
				 (CPU_STK_SIZE  )RC_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}

/**
  * @brief  RC������
  *         
  * @param  None
  * @retval None
  */
static  void  RC_MainTask (void *p_arg)
{
	OS_ERR	err;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(20,OS_OPT_TIME_PERIODIC,&err);
		if(f.RC_AVAILABLE)//��ң������Чʱ,���ػ������н���ȷ��ң�ؽ��ջ��Ƿ���Ч
		{
			RC_ComputeRcData();//���ݶ�ȡRC_RawData[8]
			if(f.RC_CTRL_MODE)
				RC_DataSynth();//ң������Ϲ���
		}
  }
}

/**
  * @brief  ��ȡֵRcData
  *         
  * @param  None
  * @retval None
  */
static void RC_ComputeRcData(void)
{
	static int16_t RC_Data4Values[6][4], RC_DataMean[6];
	static uint8_t rc4ValuesIndex = 0;
	uint8_t chan, a;

  rc4ValuesIndex++;
  for (chan = 0; chan < 6; chan++)
	{
		RC_Data4Values[chan][rc4ValuesIndex % 4] = PWM_RcValRead(chan);//��ȡ�������Ĳ���ֵ
		if(RC_Data4Values[chan][rc4ValuesIndex % 4] < 750 || RC_Data4Values[chan][rc4ValuesIndex % 4] > 2250)//��ֹԽ��
			RC_Data4Values[chan][rc4ValuesIndex % 4] = cfg.midrc;
		RC_DataMean[chan] = RC_Data4Values[chan][0];
		for (a = 1; a < 4; a++)
				RC_DataMean[chan] += RC_Data4Values[chan][a];//��μ���ǰ���ε��ܺ�

		RC_DataMean[chan] = (RC_DataMean[chan] + 2) / 4;//��ƽ��
		if (RC_DataMean[chan] < RC_RawData[chan] - 3)	//��ֹ΢С����
				RC_RawData[chan] = RC_DataMean[chan] + 2;	//RC_Data���鱣����8��ͨ����ֵ
		if (RC_DataMean[chan] > RC_RawData[chan] + 3)
				RC_RawData[chan] = RC_DataMean[chan] - 2;
  }
}

/**
  * @brief  ң��������������
  *         
  * @param  None
  * @retval None
  */
static void RC_DataSynth(void)
{
	static uint8_t rcDelayCommand;
	if(RC_RawData[THROTTLE] < cfg.mincheck)//���Ŵ������ʱ
	{
		rcDelayCommand++;
		if (RC_RawData[YAW] < cfg.mincheck && RC_RawData[PITCH] < cfg.mincheck && !f.ARMED) 
		{//���:���½�,�ұ�:�£���û����
			if (rcDelayCommand == 20) 	 //Ҳ����˵������Ҫ����20x20ms=0.4s����Ч
			{
				CalibratingG = 1000;		//У׼������
				CalibratingB=10;			//У׼��ѹ�ƣ�Ҳ�����趨��ѹ��ԭ��
        if (Feature(FEATURE_GPS))
					GPS_reset_home_position();//�����趨ԭ��
      }
		}
		else if(!f.ARMED && RC_RawData[YAW] < cfg.mincheck && RC_RawData[PITCH] > cfg.maxcheck && RC_RawData[ROLL] > cfg.maxcheck)
		{//���:���Ͻǣ��ұ�:���½ǣ�û����
			if (rcDelayCommand == 20) 
			{
      }
		}
		else if ((RC_RawData[YAW] < cfg.mincheck || (cfg.retarded_arm == 1 && RC_RawData[ROLL] < cfg.mincheck)) && f.ARMED)//����
		{	//����
			if (rcDelayCommand == 20)
        f.ARMED = 0;	//����	
		}
		else if ((RC_RawData[YAW] > cfg.maxcheck || (RC_RawData[ROLL] > cfg.maxcheck && cfg.retarded_arm == 1)) && RC_RawData[PITCH] < cfg.maxcheck && !f.ARMED )//û�����ڽ���У׼��У׼���� 
		{	 //����
			if (rcDelayCommand == 20) 
			{
				if(f.OK_TO_ARM)//����������ܽ���
				{
					f.ARMED = 1;
					HeadFreeModeHold = heading;
				}
      }
    }
		else
			rcDelayCommand = 0;	 //�����������ʱ�����
	}
	else if(RC_RawData[THROTTLE] > cfg.maxcheck && !f.ARMED)//���������û�н���
	{
		if (RC_RawData[YAW] < cfg.mincheck && RC_RawData[PITCH] < cfg.mincheck) 
		{   //��ߣ����½ǣ��ұߣ���
			if (rcDelayCommand == 20)
				CalibratingA = 400;		//�������ٶȼ�У׼		   400
      rcDelayCommand++;
    }
		else if (RC_RawData[YAW] > cfg.maxcheck && RC_RawData[PITCH] < cfg.mincheck) 
		{   //��ߣ����½ǣ������ұߣ���
			if (rcDelayCommand == 20)
				f.CALIBRATE_MAG = 1;   //��������У׼����
        rcDelayCommand++;
    }
		else if (RC_RawData[PITCH] > cfg.maxcheck) 			  //�����ĸ��Ǽ��ٶȼ�΢��
		{		//��ߣ��ϡ������ұߣ���
			cfg.angleTrim[PITCH] += 2;	//�Ƕ�΢������		ǰ
      WRITE_TO_FLASH;//������д��FLASH
    }
		else if (RC_RawData[PITCH] < cfg.mincheck) 
		{		//��ߣ��¡������ұߣ���
			cfg.angleTrim[PITCH] -= 2;	//�Ƕ�΢����С	    ��
      WRITE_TO_FLASH;//������д��FLASH
		}
		else if (RC_RawData[ROLL] > cfg.maxcheck) 
		{		//��ߣ��м䡣�����ұߣ����Ͻ�
			cfg.angleTrim[ROLL] += 2;	//΢��				��
      WRITE_TO_FLASH;//������д��FLASH
    }
		else if (RC_RawData[ROLL] < cfg.mincheck) 
		{		//��ߣ��м䡣�����ұߣ����Ͻ�
			cfg.angleTrim[ROLL] -= 2;	//΢��				��
      WRITE_TO_FLASH;//������д��FLASH
    }
		else
				rcDelayCommand = 0;
	}
}
