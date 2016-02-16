/**
	******************************************************************************
	* @file    AP_EasyCtrl.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   用户控制模块，可用于手柄姿态简易控制等，脱离遥控器
	*					 现在手柄和遥控器互相配合
	*					 当遥控器和手柄同时存在时，以AUX2为标志，决定使用谁
	*					 当只存在其中一种时，都可单独适用
	*					 如果在使用过程中，此控制突然失效，处理为：另一种存在，强制切换到另一种
	*					 另一种不存在，自动降落
	******************************************************************************
**/
#include "AP_EasyCtrl.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
int16_t HandAttitude_CtrlAngle[2];	//控制的角度，-35----+35
static int16_t HandAttitude_RcDataTh=1000;//模拟油门的数值1000-2000

int16_t RC_Data[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 }; //实际使用的通道数据,范围为1000-2000

static uint8_t GPSFollow_Mode;//1为GPS跟随模式

static uint16_t OneKey_Roll_Trig;//用于手柄触发一键翻滚

/* OS -----------------------------------------------*/
static  void  EasyCtrl_MainTask (void *p_arg);
static  OS_TCB  EasyCtrl_MainTaskTCB;
static  CPU_STK  EasyCtrl_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
void OneKey_Roll(void);

/**
  * @brief  初始化
  *         创建任务
  * @param  None
  * @retval None
  */
void EasyCtrl_Init(void)
{
	OS_ERR	err;
	OSTaskCreate((OS_TCB   *)&EasyCtrl_MainTaskTCB,  
				 (CPU_CHAR     *)"EasyCtrl_MainTask",
				 (OS_TASK_PTR   )EasyCtrl_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_EASYCTRL ,
				 (CPU_STK      *)&EasyCtrl_MainTaskStk[0],
				 (CPU_STK_SIZE  )EasyCtrl_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}

/**
  * @brief  EasyCtrl主任务
  *         
  * @param  None
  * @retval None
  */
static  void  EasyCtrl_MainTask (void *p_arg)
{
	OS_ERR	err;
	uint8_t i;
	static uint8_t first_in=1;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(2,OS_OPT_TIME_DLY,&err);//ms
		if((f.HANDLE_AVAILABLE==0)||(f.RC_AVAILABLE && (RC_RawData[AUX2] > 1600)))//使用遥控器控制
		{
			f.RC_CTRL_MODE=1;
			f.HANDLE_CTRL_MODE=0;
		}
		else if(f.HANDLE_AVAILABLE)//手柄模式
		{
			f.RC_CTRL_MODE=0;
			f.HANDLE_CTRL_MODE=1;
		}
		else//未知，求BUG
		{
			f.RC_CTRL_MODE=0;
			f.HANDLE_CTRL_MODE=0;
		}
		
		if(f.RC_CTRL_MODE)//使用遥控器
		{
			for(i=0;i<8;i++)RC_Data[i]=RC_RawData[i];
			HandAttitude_CtrlAngle[0]=0;
			HandAttitude_CtrlAngle[1]=0;
		}
		else if(f.HANDLE_CTRL_MODE)//使用手柄
		{
			RC_Data[ROLL]=1500;
			RC_Data[PITCH]=1500;
			RC_Data[YAW]=1500;
			RC_Data[THROTTLE]=HandAttitude_RcDataTh;
			if(GPSFollow_Mode)
				RC_Data[AUX1]=1100;//开启跟随模式时就将AUX1到最低，请在调参软件里选择低则开启GPS
			else
				RC_Data[AUX1]=1500;//中间，需要开启气压计和磁阻
			RC_Data[AUX2]=1500;
		}
		else//迫降
		{
			HandAttitude_CtrlAngle[0]=0;
			HandAttitude_CtrlAngle[1]=0;
		}
		
		
		if(RC_Options[BOXROLL]||OneKey_Roll_Trig)
		{
			if(first_in&&RC_Data[THROTTLE]>1300)
			{
				first_in=0;
				OneKey_RollStep = 1;
			}
			if(OneKey_Roll_Trig++>500)//1s
				OneKey_Roll_Trig=0;
		}
		else
		{
			first_in=1;
			OneKey_RollStep = 0;
		}
		
		OneKey_Roll();//一键翻滚
  }
}

/**
  * @brief  手柄姿态控制
  *         
  * @param  接收到的数据指针，在XBEE里
  * @retval None
  */
void HandAttitude_Ctrl(uint8_t *hand_data)
{
	static uint8_t first_in=0,first_in2=0;
	static int32_t GPS_HoldOffset[2];
	static double Point_Distance=0;
	int32_t gps_coord[2];
	static int32_t gps_coord2[2];
	int16_t mag_bear;
	float val;
	Handle_LostCnt=0;//实时清0错误计数

	
	if(f.HANDLE_CTRL_MODE==0)return;//没开启此模式
	
	//普通无GPS下的姿态控制    和    跟随
	if(hand_data[1]==0x30||hand_data[1]==0x31)
	{
		if(!(hand_data[7]&0x01))//没有开启悬停
		{
			HandAttitude_CtrlAngle[ROLL]=constrain((hand_data[2]-35),-35,35);//单位度
			HandAttitude_CtrlAngle[PITCH]=constrain((hand_data[3]-35),-35,35);
			magHold=(hand_data[4]<<8)|hand_data[5];
		}
		else//悬停,无姿态控制
		{
			HandAttitude_CtrlAngle[ROLL]=0;
			HandAttitude_CtrlAngle[PITCH]=0;
		}
		
		if(hand_data[6]==1)//按上升键
		{
			if(HandAttitude_RcDataTh<1450)
			{
				if(f.ARMED)HandAttitude_RcDataTh+=5;
				f.BARO_MODE = 0;
				f.SONAR_MODE = 0;
				AltHold_Baro=50.0f;
				AltHold_Sonar = 50.0f;
			}
			else
			{
				AltHold_Baro+=3.0f;//应该控制在1秒增加半米
				AltHold_Sonar += 3.0f;
			}
		}
		else if(hand_data[6]==2)//按下降键
		{
			if(f.SONAR_MODE)//超声，
			{
				if(AltHold_Sonar>-50.0f)//只有超声才会进行关闭电机判断
				{
					AltHold_Sonar-=3.0f;
				}
				else
				{
					if(EstAlt_Sonar<20.0f)
						if(HandAttitude_RcDataTh>1000)HandAttitude_RcDataTh-=5;
				}
			}
			AltHold_Baro-=3.0f;//气压
		}
		else if(hand_data[6]==4)//行动键双击，则进行上锁或者解锁
		{
			if(f.ARMED)
				f.ARMED = 0;		//上锁
			else if(f.OK_TO_ARM&&!f.ARMED)
			{
				HandAttitude_RcDataTh=1000;
				AltHold_Baro=0;
				AltHold_Sonar=0.0f;
				CalibratingG = 1000;		//校准陀螺仪	1000
				CalibratingB = 10;			//校准气压计，也就是设定气压计原点
				f.ARMED = 1;		//解锁
			}
		}
		else if(hand_data[6]==3)//两个按键长按1秒，校准
		{
			if(!f.ARMED)
			{
				CalibratingA = 400;		//触发加速度计校准
			}
		}
		//飞机跟随手柄运动
		if(hand_data[1]==0x31)//跟随
		{
			gps_coord[LON]=(hand_data[11] << 24) + (hand_data[10] << 16) + (hand_data[9] << 8) + hand_data[8];
			gps_coord[LAT]=(hand_data[15] << 24) + (hand_data[14] << 16) + (hand_data[13] << 8) + hand_data[12];
			if(first_in)
			{
				first_in = 0;
				GPS_HoldOffset[LON]=GPS_coord[LON]-gps_coord[LON];
				GPS_HoldOffset[LAT]=GPS_coord[LAT]-gps_coord[LAT];
			}
			else
			{
				if(abs(GPS_hold[LON]-(gps_coord[LON]+GPS_HoldOffset[LON]))>50||abs(GPS_hold[LAT]-(gps_coord[LAT]+GPS_HoldOffset[LAT]))>50)
				{
					GPS_hold[LON]=gps_coord[LON]+GPS_HoldOffset[LON];//经度
					GPS_hold[LAT]=gps_coord[LAT]+GPS_HoldOffset[LAT];//纬度
					GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
				}
			}
			GPSFollow_Mode = 1;
		}
		else
		{
			first_in = 1;
			if(hand_data[7]&0x01)//悬停
				GPSFollow_Mode = 1;
			else
				GPSFollow_Mode = 0;
		}
		
		if((hand_data[7]&0x04)&&f.SONAR_AVAILABLE  &&f.ARMED )//一键起飞或者一键降落
		{
			if(EstAlt_Sonar<15.0f)//take off
			{
				//magHold = heading-7;
				if (magHold > 180)
					magHold = magHold - 360;
				else if (magHold < -180)
					magHold = magHold + 360;
				HandAttitude_RcDataTh = 1450;
				AltHold_TakeOff = 150.0f;
				f.AUTOTAKEOFF=1;
				f.AUTOLANDING=0;
			}
			else//landing
			{
				f.AUTOLANDING=1;
				f.AUTOTAKEOFF=0;
			}
		}
		
		
		if(hand_data[7]&0x02)OneKey_Roll_Trig=1;//触发一键翻滚
		
		first_in2 = 1;
	}
	else if(hand_data[1]==0x32)//指点飞行
	{
		HandAttitude_CtrlAngle[ROLL]=0;
		HandAttitude_CtrlAngle[PITCH]=0;

		if(first_in2)
		{
			first_in2 = 0;//保存第一次距离
			
			gps_coord2[LON]=(hand_data[11] << 24) + (hand_data[10] << 16) + (hand_data[9] << 8) + hand_data[8];//手柄的GPS坐标
			gps_coord2[LAT]=(hand_data[15] << 24) + (hand_data[14] << 16) + (hand_data[13] << 8) + hand_data[12];
			Point_Distance = sqrt((gps_coord2[LON] - GPS_coord[LON])*(gps_coord2[LON] - GPS_coord[LON])+(gps_coord2[LAT] - GPS_coord[LAT])*(gps_coord2[LAT] - GPS_coord[LAT]));
		}
		else
		{
			mag_bear = (hand_data[4]<<8)|hand_data[5];
			gps_coord[LON] = gps_coord2[LON] + Point_Distance*sinf((float)mag_bear*2.0f*PI/360.0f);//经度
			gps_coord[LAT] = gps_coord2[LAT] + Point_Distance*cosf((float)mag_bear*2.0f*PI/360.0f);//纬度
			if(abs(GPS_hold[LON]-gps_coord[LON])>50||abs(GPS_hold[LAT]-gps_coord[LAT])>50)
			{
				GPS_hold[LON]=gps_coord[LON];//经度
				GPS_hold[LAT]=gps_coord[LAT];//纬度
				GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
			}
		}
		GPSFollow_Mode = 1;
	}
	else
	{
		GPSFollow_Mode = 0;
	}


//  	debug[0]=GPS_hold[LAT];
//  	debug[1]=mag_bear;
// 	debug[2]=Point_Distance;
// 	debug[3]=Point_Distance*cosf((float)mag_bear*2.0f*PI/360.0f);//纬度
}

uint8_t OneKey_RollStep;//指示翻滚的步，当进入翻滚模式时，将关闭自稳，进入3D模式
void OneKey_Roll(void)
{
	static uint8_t step1_tim;
	static float alt_last=100;
	switch (OneKey_RollStep)
	{
		case 1://第一步,瞬间增大油门一定时间，让灰机上升点
		{
			if(step1_tim++>100)//100x2ms=500ms
			{
				step1_tim=0;
				OneKey_RollStep=2;
				if(step1_tim ==1)
				{
					if(f.SONAR_MODE)
						alt_last = AltHold_Sonar;
					else
						alt_last = AltHold_Baro;
				}
			}
			RC_Data[THROTTLE]=2000;
		}
		break;
		case 2://第二步,左翻
		{
			RC_Data[ROLL]=2000;//向左翻滚
			if(angle[ROLL]<-200&&angle[ROLL]>-900)//翻滚到这个角度时，重新进入自稳模式
			{
				OneKey_RollStep=0;
				if(f.SONAR_MODE)
					AltHold_Sonar = alt_last;
				else
					AltHold_Baro = alt_last;
			}
		}
		break;
		default:OneKey_RollStep=0;
	}
}



