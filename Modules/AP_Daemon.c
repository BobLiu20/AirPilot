/**
	******************************************************************************
	* @file    AP_Daemon.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   守护进程
	*					 默默的呵护着灰机的安全
	******************************************************************************
**/
#include "AP_Daemon.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
uint16_t Rc_LostCnt;//遥控信号丢失计数
uint16_t Handle_LostCnt;//手柄信号丢失计数

uint16_t LED_FlashBit;//记录着需要闪烁的位
uint8_t NotOkToArmBit;//当某一位为1时，则不可解锁
uint8_t NeedToWriteFlash;//当此值为1时，需要写入FLASH操作

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
  * @brief  初始化
  *         创建任务
  * @param  None
  * @retval None
  */
void Daemon_Init(void)
{
	OS_ERR	err;
	Battery_Init();//电量测定
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
  * @brief  IMU主任务
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
		Daemon_Handle();//手柄信号丢失守护
		Daemon_Rc();		//遥控信号丢失保护
		LED_ActFlash(); //LED1
		LED_GPSFlash(); //LED2
		Daemon_Armed(); //解锁允许保护
		Battery_Read();//读取电量
		if(NeedToWriteFlash)//将参数写入FLASH
		{
			OSSchedLock(&err);//锁调度器
// 			I2C2->CR2 &= (uint16_t)~(I2C_IT_EVT|I2C_IT_BUF);//关闭I2C中断
// 			I2C2->CR1 |= I2C_CR1_ACK;//使能自动应答
			CPU_CRITICAL_ENTER();
			Flash_WriteParams(1);//未知原因，在进行FLASH操作时必须关闭I2C中断
			CPU_CRITICAL_EXIT();
			OSSchedUnlock(&err);
			NeedToWriteFlash=0;
		}
  }
}

/**
  * @brief  遥控器信号守护
  *         20ms调用一次
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
			f.RC_AVAILABLE=0;//不存在遥控接收机
			f.RC_CTRL_MODE=0;//非遥控器模式
			if(f.HANDLE_AVAILABLE)
			{
				f.HANDLE_CTRL_MODE=1;//存在手柄时强制进入手柄模式
				return ;
			}
			
			if(f.ARMED)//失去信号超过设定值，则将油门设定到指定值
			{
				if(Sensors(SENSOR_ACC))cfg.activate[BOXACC]=0xffff;//存在气压计时强制开启自稳模式
				for(i=0;i<3;i++)
					RC_Data[i] = cfg.midrc;	//将ROLL,PITCH,YAW强制为1500，即中间
				RC_Data[THROTTLE] = cfg.failsafe_throttle;//强制将失效保护特殊油门值赋值
				if(Rc_LostCnt > (5 * (cfg.failsafe_delay + cfg.failsafe_off_delay)))//丢失信号超过着陆时间，则关闭电机,认为自己已着陆
				{
					f.ARMED = 0;		//上锁
					NOTOKTOARM_SET(NOTARMBIT_LOSTRC);//不允许解锁
				}
			}
			else if(!f.ARMED)//未解锁状态下丢失信号
			{
				f.ARMED = 0;		//上锁
				NOTOKTOARM_SET(NOTARMBIT_LOSTRC);//不允许解锁
			}
		}
		else
		{
			f.RC_AVAILABLE=1;//存在遥控接收机
			NOTOKTOARM_CLR(NOTARMBIT_LOSTRC);//清除禁止解锁位
		}
		Rc_LostCnt++;	//错误次数累加，由遥控接收器信号中断实时清0
	}
}

/**
  * @brief  手柄信号守护
  *         20ms调用一次
  * @param  None
  * @retval None
  */
static void Daemon_Handle(void)
{
	uint8_t i;
	if(Handle_LostCnt > 50)//超过1秒无信号
	{
		f.HANDLE_AVAILABLE=0;//不存在手柄控制
		f.HANDLE_CTRL_MODE=0;//非手柄控制
		if(f.RC_AVAILABLE)
		{
			f.RC_CTRL_MODE=1;//存在遥控器则强制进入遥控模式
		}
	}
	else
	{
		f.HANDLE_AVAILABLE=1;//存在遥控接收机
		NOTOKTOARM_CLR(NOTARMBIT_LOSTRC);//清除禁止解锁位
	}
	Handle_LostCnt++;	//错误次数累加，由手柄接收处实时清0
}

/**
  * @brief  LED_ACT指示灯
  *         20ms调用一次
  * @param  
  * @retval 
  */
static void LED_ActFlash(void)
{
	static uint8_t count=0;
	if(LED_FlashBit & LEDFLASH_MAGCAL)//校准磁阻中，进入闪烁
	{
		if(((count++)%5)==0)	//100ms
			LED_Toggle(LED_ACT);
	}
	else if(LED_FlashBit & LEDFLASH_SENSORCAL)//校准加速度或陀螺仪
	{
		if(((count++)%3)==0)	//60ms
			LED_Toggle(LED_ACT);
	}
	else if(LED_FlashBit & LEDFLASH_AIRSLOPE)//解锁前，机身过于倾斜,或者加速度未校准
	{
		if(((count++)%25)==0)	//500ms
			LED_Toggle(LED_ACT);
	}
	else if(LED_FlashBit & LEDFLASH_ARMD)//解锁状态，常亮
	{
		LED_On(LED_ACT);
	}
	else
	{
		LED_Off(LED_ACT);
	}
}

/**
  * @brief  GPS指示灯
  *         在卫星数少于6颗时，LED将在两秒内闪烁相应的次数，比如2颗则两秒内闪烁两次
	*					6颗以上则直接常亮了
  * @param  
  * @retval 
  */
static void LED_GPSFlash(void)
{
	static uint8_t count=0;
	
	if(GPS_numSat==0)GPS_numSat=1;
	
	if(GPS_numSat>5)//卫星超过5颗就直接常亮了
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
  * @brief  解锁保护，管理是否允许解锁
  *         
  * @param  None
  * @retval None
  */
static void Daemon_Armed(void)
{
// 	if(abs(angle[ROLL])>1000||abs(angle[PITCH])>1000)		//超过正负100度证明飞机失事，马上把电机关了并锁定，以防止飞机翻了还在地上转
// 	{
// 		f.ARMED = 0;		//上锁
// 		NOTOKTOARM_SET(NOTARMBIT_ANGLE100);//不允许解锁
// 	}
// 	else
		NOTOKTOARM_CLR(NOTARMBIT_ANGLE100);//释放禁止解锁位
	
	if(f.ACC_CALIBRATED)//解锁前，机身倾斜超过25度，不允许解锁
		NOTOKTOARM_CLR(NOTARMBIT_ANGLE25);
	else
		NOTOKTOARM_SET(NOTARMBIT_ANGLE25);
	
	if(LED_FlashBit&(LEDFLASH_MAGCAL|LEDFLASH_SENSORCAL))//当正在进行传感器校准时，不允许解锁
		NOTOKTOARM_SET(NOTARMBIT_CALING);
	else
		NOTOKTOARM_CLR(NOTARMBIT_CALING);
	
	if(I2C_ErrorCount>100)							//I2C出错数超过100，不允许解锁
		NOTOKTOARM_SET(NOTARMBIT_I2CERR);
	else
		NOTOKTOARM_CLR(NOTARMBIT_I2CERR);

	if(NotOkToArmBit)//所有位都为0，则允许解锁
		f.OK_TO_ARM=0;
	else
		f.OK_TO_ARM=1;//允许解锁
}
