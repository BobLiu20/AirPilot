/**
	******************************************************************************
	* @file    AP_RC.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   RC(Radio Control)
	*					 遥控器信号接收与处理
	*					 从遥控接收机获取的通道数据保存在RC_RawData[]里
	*					 然后将在AP_EasyCtrl.c里进行处理
	******************************************************************************
**/
#include "AP_RC.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
int16_t RC_RawData[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 }; // interval [1000;2000]//保存着遥控器8个通道的原始值

/* OS -----------------------------------------------*/
static  void  RC_MainTask (void *p_arg);
static  OS_TCB  RC_MainTaskTCB;
static  CPU_STK  RC_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
static void RC_ComputeRcData(void);
static void RC_DataSynth(void);

/**
  * @brief  初始化
  *         创建任务
  * @param  None
  * @retval None
  */
void RC_Init(void)
{
	OS_ERR	err;
	PWM_RcCaptureInit();//初始化
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
  * @brief  RC主任务
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
		if(f.RC_AVAILABLE)//当遥控器有效时,在守护进程中进行确定遥控接收机是否有效
		{
			RC_ComputeRcData();//数据读取RC_RawData[8]
			if(f.RC_CTRL_MODE)
				RC_DataSynth();//遥控器组合功能
		}
  }
}

/**
  * @brief  获取值RcData
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
		RC_Data4Values[chan][rc4ValuesIndex % 4] = PWM_RcValRead(chan);//获取接收器的捕获值
		if(RC_Data4Values[chan][rc4ValuesIndex % 4] < 750 || RC_Data4Values[chan][rc4ValuesIndex % 4] > 2250)//防止越界
			RC_Data4Values[chan][rc4ValuesIndex % 4] = cfg.midrc;
		RC_DataMean[chan] = RC_Data4Values[chan][0];
		for (a = 1; a < 4; a++)
				RC_DataMean[chan] += RC_Data4Values[chan][a];//这次加上前三次的总和

		RC_DataMean[chan] = (RC_DataMean[chan] + 2) / 4;//求平均
		if (RC_DataMean[chan] < RC_RawData[chan] - 3)	//防止微小抖动
				RC_RawData[chan] = RC_DataMean[chan] + 2;	//RC_Data数组保存着8个通道的值
		if (RC_DataMean[chan] > RC_RawData[chan] + 3)
				RC_RawData[chan] = RC_DataMean[chan] - 2;
  }
}

/**
  * @brief  遥控器动作处理函数
  *         
  * @param  None
  * @retval None
  */
static void RC_DataSynth(void)
{
	static uint8_t rcDelayCommand;
	if(RC_RawData[THROTTLE] < cfg.mincheck)//油门处于最低时
	{
		rcDelayCommand++;
		if (RC_RawData[YAW] < cfg.mincheck && RC_RawData[PITCH] < cfg.mincheck && !f.ARMED) 
		{//左边:左下角,右边:下，在没解锁
			if (rcDelayCommand == 20) 	 //也就是说本动作要保持20x20ms=0.4s才有效
			{
				CalibratingG = 1000;		//校准陀螺仪
				CalibratingB=10;			//校准气压计，也就是设定气压计原点
        if (Feature(FEATURE_GPS))
					GPS_reset_home_position();//重新设定原点
      }
		}
		else if(!f.ARMED && RC_RawData[YAW] < cfg.mincheck && RC_RawData[PITCH] > cfg.maxcheck && RC_RawData[ROLL] > cfg.maxcheck)
		{//左边:左上角，右边:右下角，没解锁
			if (rcDelayCommand == 20) 
			{
      }
		}
		else if ((RC_RawData[YAW] < cfg.mincheck || (cfg.retarded_arm == 1 && RC_RawData[ROLL] < cfg.mincheck)) && f.ARMED)//上锁
		{	//上锁
			if (rcDelayCommand == 20)
        f.ARMED = 0;	//锁定	
		}
		else if ((RC_RawData[YAW] > cfg.maxcheck || (RC_RawData[ROLL] > cfg.maxcheck && cfg.retarded_arm == 1)) && RC_RawData[PITCH] < cfg.maxcheck && !f.ARMED )//没有正在进行校准且校准正常 
		{	 //解锁
			if (rcDelayCommand == 20) 
			{
				if(f.OK_TO_ARM)//允许解锁才能解锁
				{
					f.ARMED = 1;
					HeadFreeModeHold = heading;
				}
      }
    }
		else
			rcDelayCommand = 0;	 //清除动作保持时间计数
	}
	else if(RC_RawData[THROTTLE] > cfg.maxcheck && !f.ARMED)//油门最大且没有解锁
	{
		if (RC_RawData[YAW] < cfg.mincheck && RC_RawData[PITCH] < cfg.mincheck) 
		{   //左边：左下角，右边：上
			if (rcDelayCommand == 20)
				CalibratingA = 400;		//触发加速度计校准		   400
      rcDelayCommand++;
    }
		else if (RC_RawData[YAW] > cfg.maxcheck && RC_RawData[PITCH] < cfg.mincheck) 
		{   //左边：右下角，，，右边：上
			if (rcDelayCommand == 20)
				f.CALIBRATE_MAG = 1;   //电子罗盘校准请求
        rcDelayCommand++;
    }
		else if (RC_RawData[PITCH] > cfg.maxcheck) 			  //下面四个是加速度计微调
		{		//左边：上。。。右边：上
			cfg.angleTrim[PITCH] += 2;	//角度微调增大		前
      WRITE_TO_FLASH;//将参数写入FLASH
    }
		else if (RC_RawData[PITCH] < cfg.mincheck) 
		{		//左边：下。。。右边：上
			cfg.angleTrim[PITCH] -= 2;	//角度微调减小	    后
      WRITE_TO_FLASH;//将参数写入FLASH
		}
		else if (RC_RawData[ROLL] > cfg.maxcheck) 
		{		//左边：中间。。。右边：右上角
			cfg.angleTrim[ROLL] += 2;	//微调				右
      WRITE_TO_FLASH;//将参数写入FLASH
    }
		else if (RC_RawData[ROLL] < cfg.mincheck) 
		{		//左边：中间。。。右边：左上角
			cfg.angleTrim[ROLL] -= 2;	//微调				左
      WRITE_TO_FLASH;//将参数写入FLASH
    }
		else
				rcDelayCommand = 0;
	}
}
