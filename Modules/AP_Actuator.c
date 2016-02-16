/**
	******************************************************************************
	* @file    AP_Actuator.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   神经中枢
	******************************************************************************
**/
#include "AP_Actuator.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
uint8_t RC_Options[CHECKBOXITEMS];//遥控器辅助通道的功能选项
int16_t RC_Command[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 后面这三个是用RC_Data-1500得到的

static uint8_t dynP8[3], dynD8[3];//dynI8[3], 
int16_t axisPID[3];
static float errorAngleI[2] = { 0, 0 };

int16_t HeadFreeModeHold;		//无头模式

uint16_t CycleTime = 1000;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

static int16_t InitialThrottleHold;



/* OS -----------------------------------------------*/
static  void  Actuator_MainTask (void *p_arg);
static  OS_TCB  Actuator_MainTaskTCB;
static  CPU_STK  Actuator_MainTaskStk[APP_CFG_TASK_START_STK_SIZE*4];

/* function prototypes -----------------------------------------------*/
static void Actuator_ModeState(void);
static void Actuator_AnnexCode(void);
static void Actuator_MainSynth(void);

/**
  * @brief  初始化
  *         创建任务
  * @param  None
  * @retval None
  */
void Actuator_Init(void)
{
	OS_ERR	err;
	OSTaskCreate((OS_TCB   *)&Actuator_MainTaskTCB,  
				 (CPU_CHAR     *)"Actuator_MainTask",
				 (OS_TASK_PTR   )Actuator_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_ACTUATOR ,
				 (CPU_STK      *)&Actuator_MainTaskStk[0],
				 (CPU_STK_SIZE  )Actuator_MainTaskStk[APP_CFG_TASK_START_STK_SIZE*4 / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}

/**
  * @brief  Actuator主任务
  *         核心任务！整个飞控的中枢
  * @param  None
  * @retval None
  */
static  void  Actuator_MainTask (void *p_arg)
{
	OS_ERR	err;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(2,OS_OPT_TIME_DLY,&err);
		Actuator_ModeState();//根据AUX设置模式
		Actuator_AnnexCode();
		Actuator_MainSynth();//综合
		OSFlagPost(&Mixer_TaskRunFlagGrp,//电机舵机输出
							(OS_FLAGS)0x01,
							OS_OPT_POST_FLAG_SET,
							&err);
  }
}


/**
  * @brief  模式选项，根据AUX通道数据启用关闭各种模式功能
  *         
  * @param  None
  * @retval None
  */
static void Actuator_ModeState(void)
{
	uint8_t i;
	uint16_t auxState = 0;
	for(i = 0; i < 4; i++)	   //得到下辅助通道的状态，每3位保存一个通道
		auxState |= (RC_Data[AUX1 + i] < 1300) << (3 * i) | (1300 < RC_Data[AUX1 + i] && RC_Data[AUX1 + i] < 1700) << (3 * i + 1) | (RC_Data[AUX1 + i] > 1700) << (3 * i + 2);
	for(i = 0; i < CHECKBOXITEMS; i++)//在这里更新RC_Options[]的数据，各种模式...
		RC_Options[i] = (auxState & cfg.activate[i]) > 0;

	if (RC_Options[BOXACC] && Sensors(SENSOR_ACC))
	{
		if (!f.ACC_MODE) 	//开启自稳
		{
			errorAngleI[ROLL] = 0;
			errorAngleI[PITCH] = 0;
			f.ACC_MODE = 1;
		}
	} 
	else
		f.ACC_MODE = 0;

// 	if (Sensors(SENSOR_BARO))//如果存在气压计 
// 	{
// 		if (RC_Options[BOXBARO] ) //是否开启定高				RC_Options[BOXBARO] 暂时不用气压定高模式
// 		{
// 			if (!f.BARO_MODE) 			//功能打开后只会进一次
// 			{
// 				f.BARO_MODE = 1;		//开启定高模式
// 				AltHold_Baro = EstAlt_Baro;		//将当前高度设定为保持高度
// 				InitialThrottleHold = RC_Command[THROTTLE];	//当前油门设定为保持油门
// 				errorAltitudeI = 0;	
// 				BaroPID = 0;
// 			}
// // #ifdef SONAR
// // 			if (sonar.flag) //超声数据是不是有效
// // 			{	
// // 					 
// // 				if (!f.SONAR_MODE) 
// // 				{
// // 					f.SONAR_MODE = 1;		//开启定高模式
// // 					SonarAltHold_Baro = sonar.Distance;		//将当前高度设定为保持高度
// // 								//    InitialThrottleHold = RC_Command[THROTTLE];	//当前油门设定为保持油门
// // 										//errorAltitudeI = 0;
// // 					SonarPID = 0;
// // 					PreSonarDistance = sonar.Distance;
// // 					I_Term=0;
// // 				}
// // 			} 
// // 			else				   //不开启   更重要是数据无效要把这个开关关了  下次有效才会重新初始化新的定高数据
// // 			{
// // 				if(f.SONAR_MODE)
// // 				{
// // 					f.BARO_MODE=0;	   //在超声波无效瞬间，将这个变为0，让气压计定高维持此刻高度
// // 					f.SONAR_MODE = 0;
// // 				}
// // 			}
// // #endif
// 		}
// 		else				   //定高功能没开就把两个模式的开关都关了
// 		{
// 			f.BARO_MODE = 0;
// //			f.SONAR_MODE = 0;
// 			AltHold_Baro = EstAlt_Baro;    //在没定高的时候做这个赋值，可以避免定高开启时PID有错误输出
// 		}
// 	}


		
	if (Sensors(SENSOR_MAG))//如果存在电子罗盘 
	{
		if (RC_Options[BOXMAG]) //定否开启定向
		{
			if (!f.MAG_MODE) 
			{
				f.MAG_MODE = 1;	  //开启定向
				magHold = heading;//将当前方向设定为保持方向
			}
		} 
		else
			f.MAG_MODE = 0;		  //不开启
		if (RC_Options[BOXHEADFREE])//无头模式
		{
			if (!f.HEADFREE_MODE)  //其定向的方位可实时手动改变
			{		
				f.HEADFREE_MODE = 1;//开启此模式
			}
		} 
		else 
		{
			f.HEADFREE_MODE = 0;
		}
		if (RC_Options[BOXHEADADJ])
		{
			HeadFreeModeHold = heading; //获取新的方向
		}
	}

	if (Sensors(SENSOR_GPS)) //存在GPS设备
	{
		if (f.GPS_FIX && GPS_numSat >= 5) //如果GPS值有效和卫星数大于5
		{

			if (RC_Options[BOXGPSHOLD]&&!((VisionLanding_Angle[0] || VisionLanding_Angle[1]) && nav_mode==NAV_MODE_POSHOLD)) 	   //&&!(f.AUTOTAKEOFF && EstAlt_Baro<400.0f)
			{//在悬停模式，且视觉有效时，关闭GPS
				if (!f.GPS_HOLD_MODE)
				{
					f.GPS_HOLD_MODE = 1;   //开启
					GPS_hold[LAT] = GPS_coord[LAT];	 //保存当前所在坐标
					GPS_hold[LON] = GPS_coord[LON];
					GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
					nav_mode = NAV_MODE_POSHOLD;
					XbeePro_ImportanceInfo = XbeePro_ImportanceInfo | 0x01;//发送定点位置给上位机
				}
			} 
			else 
			{
				f.GPS_HOLD_MODE = 0;
			}
			
	    if(RC_Options[BOXGPSHOME]) //是否开启GO HOME
			{
				if(!f.GPS_HOME_MODE)
				{
					f.GPS_HOME_MODE = 1;
					GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);//设定灰机的下一个地点还是当前地点,即原地保持
					nav_mode = NAV_MODE_WP;
				}
			}
			else 
			{
				f.GPS_HOME_MODE = 0;
			}
		}
	}
	Xbee_Debug[0]=f.GPS_HOLD_MODE;
	Xbee_Debug[1]=EstAlt_Sonar;
	Xbee_Debug[2]=AltHold_Sonar;
}

/**
  * @brief  
  *         
  * @param  
  * @param  
  * @retval 
  */
#define BREAKPOINT 1500
static void Actuator_AnnexCode(void)
{
  uint16_t tmp, tmp2;
  uint8_t axis, prop1, prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if (RC_Data[THROTTLE] < BREAKPOINT) 
	{
		prop2 = 100;
  }
	else 
	{
		if (RC_Data[THROTTLE] < 2000) 
		{
			prop2 = 100 - (uint16_t) cfg.dynThrPID * (RC_Data[THROTTLE] - BREAKPOINT) / (2000 - BREAKPOINT);
    }
		else 
		{
			prop2 = 100 - cfg.dynThrPID;
    }
  }

	for (axis = 0; axis < 3; axis++) 
	{
		tmp = min(abs(RC_Data[axis] - cfg.midrc), 500);
		if (axis != 2)  // ROLL & PITCH
		{
			if (cfg.deadband) 
			{
				if (tmp > cfg.deadband) 
				{
					tmp -= cfg.deadband;
				}
				else
				{
					tmp = 0;
				}
			}
      tmp2 = tmp / 100;
      RC_Command[axis] = LookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (LookupPitchRollRC[tmp2 + 1] - LookupPitchRollRC[tmp2]) / 100;
      prop1 = 100 - (uint16_t) cfg.rollPitchRate * tmp / 500;
      prop1 = (uint16_t) prop1 *prop2 / 100;
    }
		else                // YAW
		{
			if (cfg.yawdeadband) 
			{
				if (tmp > cfg.yawdeadband)
				{
					tmp -= cfg.yawdeadband;
				}
				else 
				{
					tmp = 0;
				}
			}
      RC_Command[axis] = tmp;
      prop1 = 100 - (uint16_t) cfg.yawRate * tmp / 500;
    }
    dynP8[axis] = (uint16_t) cfg.P8[axis] * prop1 / 100;
    dynD8[axis] = (uint16_t) cfg.D8[axis] * prop1 / 100;
    if (RC_Data[axis] < cfg.midrc)
      RC_Command[axis] = -RC_Command[axis];
	}

	tmp = constrain(RC_Data[THROTTLE], cfg.mincheck, 2000);
	tmp = (uint32_t) (tmp - cfg.mincheck) * 1000 / (2000 - cfg.mincheck);       // [MINCHECK;2000] -> [0;1000]
	tmp2 = tmp / 100;
	RC_Command[THROTTLE] = LookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (LookupThrottleRC[tmp2 + 1] - LookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

	RawRcCommand_Val = RC_Command[THROTTLE];
	
	//在一键翻滚下的第一步，需要暂时关闭定高
	if(f.BARO_MODE&&OneKey_RollStep!=1)RC_Command[THROTTLE]=Altitude_HoldRcCommand;//在定高模式下使用此值

	if(f.HEADFREE_MODE) 
	{
		float radDiff = (heading - HeadFreeModeHold) * M_PI / 180.0f;
		float cosDiff = cosf(radDiff);
		float sinDiff = sinf(radDiff);
		int16_t RC_Command_PITCH = RC_Command[PITCH] * cosDiff + RC_Command[ROLL] * sinDiff;
		RC_Command[ROLL] = RC_Command[ROLL] * cosDiff - RC_Command[PITCH] * sinDiff;
		RC_Command[PITCH] = RC_Command_PITCH;
	}
	if ((CalibratingA > 0 && Sensors(SENSOR_ACC)) || (CalibratingG > 0))
	{
		LEDFLASH_SET(LEDFLASH_SENSORCAL);//设置闪烁
	}
	else 
	{
		LEDFLASH_CLR(LEDFLASH_SENSORCAL);
		if (!f.SMALL_ANGLES_25&&!f.ARMED)//在解锁前，灰机倾角超过25
		{
			f.ACC_CALIBRATED = 0;//未校准,当解锁前判断到机身倾斜，则认为是加速度计为校准
			LEDFLASH_SET(LEDFLASH_AIRSLOPE);
		}
		else
		{
			f.ACC_CALIBRATED = 1;//已校准
			LEDFLASH_CLR(LEDFLASH_AIRSLOPE);
		}
		if (f.ARMED)
			LEDFLASH_SET(LEDFLASH_ARMD);
		else
			LEDFLASH_CLR(LEDFLASH_ARMD);
  }
}

/**
  * @brief  
  *         
  * @param  
  * @retval 
  */
#define GYRO_RATE_SCALE    1998.0f/(32767.0f / 4.0f)	   //乘后输出单位 ：度每秒
#define  MainSynth_FILTER     (1.0f / ( 2 * M_PI * 10 )) // 20hz以上滤除的滤波因数   最后一个数改变可以设置滤除频率
static void Actuator_MainSynth(void)
{
	uint8_t axis;
	float error, errorAngle ;//	  ,PerrorAngle 
	float PTerm, ITerm,Level_PTerm,Level_ITerm;
	float RateError ,target_rate ,gyro_rate, DTerm;
	static float last_RateError[3] = { 0.0, 0.0, 0.0 };
	static float last_DTerm[2] = { 0.0, 0.0 };
	static int16_t errorGyroI[3] = { 0, 0, 0 };
	int16_t delta, deltaSum;
	static int16_t delta1[3], delta2[3];
	static int16_t lastGyro[2] = { 0, 0 };
	static uint8_t clean_piderr=0;

	float dt=0.0;
	static uint32_t pre_time;//上一次的时间戳
	uint32_t cur_time;			 //当前的时间戳
	cur_time=TimUsGet();
	CycleTime=cur_time-pre_time;
	dt=CycleTime * 0.000001;//dt单位为s
	pre_time=cur_time;

	if(!f.ARMED)
	{
		clean_piderr=1;
		errorGyroI[ROLL] = 0;
    errorGyroI[PITCH] = 0;
		errorGyroI[YAW] = 0;
		errorAngleI[ROLL] = 0;
		errorAngleI[PITCH] = 0;
	}
	if(clean_piderr && RC_RawData[THROTTLE]<1300)
	{
		//clean_piderr=0;
		errorGyroI[ROLL] = 0;
    errorGyroI[PITCH] = 0;
		errorGyroI[YAW] = 0;
		errorAngleI[ROLL] = 0;
		errorAngleI[PITCH] = 0;
	}
	else
		clean_piderr=0;

/*定向处理*/
	if (Sensors(SENSOR_MAG)) 	 //如果存在MAG
	{
		if (abs(RC_Command[YAW]) < 70 && f.MAG_MODE) //如果YAW离中间不超过70的且开启了定向模式，则
		{
			int16_t dif = heading - magHold;		//得到差值
			if (dif <= -180)						//修正
				dif += 360;
			if (dif >= +180)
				dif -= 360;
			if (f.SMALL_ANGLES_25)
				RC_Command[YAW] -= dif * cfg.P8[PIDMAG] / 30;    // 18 deg
		}
		else
			magHold = heading;							//   识别为YAW变动，保存新的heading值，新方向
	}

	/*定高功能*/
// 	if (Sensors(SENSOR_BARO)) 
// 	{
// 		if (f.BARO_MODE)  //进入定高处理||f.SONAR_MODE
// 		{
// 			if (abs(RC_Command[THROTTLE] - InitialThrottleHold) > 20)  
// 			{
// 				f.BARO_MODE = 0;   // so that a new althold reference is defined
// //				f.SONAR_MODE = 0;
// 			}

// // #ifdef SONAR
// // 				if(f.SONAR_MODE)	   //在超声数据有效下   加上超声定高PID项
// // 					RC_Command[THROTTLE] = InitialThrottleHold + SonarPID;   // BaroPID +加上PID出来的值
// // 				else
// // #endif
// 			RC_Command[THROTTLE] = InitialThrottleHold + BaroPID;

// 				if(EasyLanding_MODE)		  //一键降落
// 				{ 
// 					if(EstAlt_Baro <200 &&f.BARO_MODE)		   //快到达了，减速，增大D值 
// 						cfg.D8[PIDALT] = D_ALT;
// 				#ifdef SONAR
// 					else if(sonar.Distance<150&&f.SONAR_MODE)
// 						cfg.D8[PIDALT] = D_ALT;
// 				#endif
// 					else
// 						cfg.D8[PIDALT] = 50;
					
//						if(EstAlt_Baro<100 && accZ<-10)
//						{
//							f.ARMED = 0;		//上锁
//							cfg.D8[PIDALT] = D_ALT;
//							EasyLanding_MODE = 0;
//						}
// 					if(f.SONAR_MODE)	  //如果超声有效，用其进行降落
// 					{
// 					#ifdef SONAR
// 						SonarAltHold_Baro-=constrain( SonarAltHold_Baro * 0.1f ,0.0 ,100.0) * dt;
// 					#endif
// 					}
// 					else
// 					{
// 						AltHold_Baro -= 	constrain( ( AltHold_Baro - (-200) ) * 0.1f ,0 ,100) * dt;   //每秒降落的距离为当前高度*0.1 ， 且限制最大降落速度为0.5m/s
// 					}
//		}
// 		if(EasyLaunching_MODE)  	  //一键起飞   控制台启用该功能只需要将这个标志位置一  再设置一个	  AltHold_Baro  就是一键起飞 非常简单
// 		{
// 			cfg.D8[PIDALT] = 20; 
// 			if(abs(EstAlt_Baro - AltHold_Baro)<200 )		   //快到达了，减速，增大D值  起飞比降落要简单的多
// 			{
// 				cfg.D8[PIDALT] = D_ALT;
// 				EasyLaunching_MODE=0;
// 			}
// 		}
//  }

	/*GPS*/
	if(Sensors(SENSOR_GPS))
	{
		static u8 GPS_coord_Saved = 0;
		// 检查我们是否要进入导航
		if (( !f.GPS_HOLD_MODE) || (!f.GPS_FIX_HOME)) 	 //  !f.GPS_HOME_MODE &&
		{
			//如果不需要，则清除所有与导航相关的参数等
			GPS_reset_nav();
		}
		else 
		{
			float sin_yaw_y = sinf(heading * 0.0174532925f);
			float cos_yaw_x = cosf(heading * 0.0174532925f);
			if (cfg.nav_slew_rate)
			{
				nav_rated[LON] += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -cfg.nav_slew_rate, cfg.nav_slew_rate); // TODO check this on uint8
				nav_rated[LAT] += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]),-cfg.nav_slew_rate, cfg.nav_slew_rate);

//					nav_rated[LON] = last_nav_rated[LON] + (dt / (GPS_filter + dt)) * (nav_rated[LON] - last_nav_rated[LON]);
//					last_nav_rated[LON] =   nav_rated[LON] ;	   //保存这个用来对D_Term进行低通滤波
//					nav_rated[LAT] = last_nav_rated[LAT] + (dt / (GPS_filter + dt)) * (nav_rated[LAT] - last_nav_rated[LAT]);
//					last_nav_rated[LAT] =   nav_rated[LAT] ;	   //保存这个用来对D_Term进行低通滤波

				GPS_angle[ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
				GPS_angle[PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
			}
			else
			{
				GPS_angle[ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
				GPS_angle[PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
			}

			if(nav_mode == NAV_MODE_POSHOLD)			  //在悬停模式下启用GPS姿态控制模式
			{
				if(abs(RC_Command[ROLL])<20 && abs(RC_Command[PITCH])<20)	  //这个是用于在悬停下，人控制遥控器移动飞机，定点位置也跟着更新
				{
					GPS_Speed_Mode = 0;
					if(!GPS_coord_Saved && abs(actual_speed[LAT])<100 && abs(actual_speed[LON])<100)		   //还没保存悬停位置
					{
						GPS_hold[LAT] = GPS_coord[LAT];	 //保存当前所在坐标
						GPS_hold[LON] = GPS_coord[LON];
						GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
						GPS_coord_Saved = 1;
					}
				}
				else	 //有控制介入  飞行器按照  RC_Command的速度飞行   比如 200 就是200cm/s的速度
				{
					GPS_coord_Saved	= 0;   //下次会更新悬停位置
					GPS_Speed_Mode = 1;
					GPS_target_speed[LAT] = (RC_Command[PITCH] * cos_yaw_x - RC_Command[ROLL] * sin_yaw_y);
					GPS_target_speed[LON] = (RC_Command[PITCH] * sin_yaw_y + RC_Command[ROLL] * cos_yaw_x);
				}
			}
	
//				GPS_angle[ROLL] = last_GPS_angle[ROLL] + (dt / (GPS_filter + dt)) * (GPS_angle[ROLL] - last_GPS_angle[ROLL]);
//				last_GPS_angle[ROLL] =   GPS_angle[ROLL] ;	   //保存这个用来对D_Term进行低通滤波
//				GPS_angle[PITCH] = last_GPS_angle[PITCH] + (dt / (GPS_filter + dt)) * (GPS_angle[PITCH] - last_GPS_angle[PITCH]);
//				last_GPS_angle[PITCH] =   GPS_angle[PITCH] ;	   //保存这个用来对D_Term进行低通滤波
		}
	}
	
	// **** ROLL=0 & PITCH=1 & YAW=2 PID ****    
	for (axis = 0; axis < 3; axis++) 
	{
		if (axis < 2) //自稳模式
		{
			//以下是全新的双重嵌套PID控制算法
			if(f.ACC_MODE&&!OneKey_RollStep)//OneKey_RollStep不为0时说明进入一键翻滚，需关闭自稳
			{
				if(Sensors(SENSOR_GPS) && f.GPS_HOLD_MODE && nav_mode==NAV_MODE_POSHOLD)
					errorAngle = constrain( GPS_angle[axis], -300, +300) - angle[axis] + cfg.angleTrim[axis];
				else
					errorAngle = constrain( RC_Command[axis] + GPS_angle[axis], -300, +300) - angle[axis] + cfg.angleTrim[axis];
				//人工姿态控制
				if(f.HANDLE_AVAILABLE)errorAngle+=(HandAttitude_CtrlAngle[axis]*5);//存在手柄时，加上手柄控制量
				
				errorAngle -= OpticalFlow_Angle[axis];
				errorAngle -= VisionLanding_Angle[axis];
				//errorAngle -= (Avoid_Angle[2*(1-axis)]-Avoid_Angle[2*(1-axis)+1]);
				if(axis==0) errorAngle += Avoid_Angle[2];//errorAngle -= (Avoid_Angle[3]-Avoid_Angle[2]);
				if(axis==1) errorAngle -= Avoid_Angle[0];//(Avoid_Angle[0]-Avoid_Angle[1]);
			}
			else
				errorAngle = 3 * constrain( RC_Command[axis] , -500, +500);

			
				//attPID = updatePID( attCmd[axis],  sensors.attitude200Hz[axis], dt, holdIntegrators, &systemConfig.PID[ROLL_ATT_PID ] );
			 //注意 ： 在这里的cfg.P值是调试软件里的值放大了十倍   其中 POS的P被放大100倍
			 //D值没有被放大  
			 //I值被放大了1000倍    只有导航相关的3个I参数被放大100倍
				Level_PTerm = errorAngle * cfg.P8[PIDLEVEL] / 10 /10 ;	 //由于ANGLE是放大了十倍的值 所以要除以10	 
//				Level_PTerm = constrain(Level_PTerm, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);		
		
				errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle * 10 * dt, -1500, +1500);  		//相当于一秒积出一个errorAngle 的十倍 (25度 2500) 所以I应取0.01级别的参数
			
				Level_ITerm = (errorAngleI[axis] * cfg.I8[PIDLEVEL])/1000;     // 记得I被放大了1000倍，所以要除以1000以还原我设置的I参数，0.01级别的 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

				target_rate = Level_PTerm + Level_ITerm ;	 //输出为： 误差（度）*P

			gyro_rate =  gyroData[axis] * GYRO_RATE_SCALE;	   //转换后单位为 度每秒

			RateError =  target_rate - gyro_rate ; 
			PTerm = RateError * dynP8[axis] / 10; 		  //p: 1.4 *10(被调试软将乘的)  /100  =0.14

			DTerm  = (RateError- last_RateError[axis]) / dt;
	
			//D项对高频噪声十分敏感，有可能会破坏整个控制器的稳定，所以要进行低通滤波
			DTerm = last_DTerm[axis] + (dt / (MainSynth_FILTER + dt)) * (DTerm - last_DTerm[axis]);
									
			last_DTerm[axis] =   DTerm ;	   //保存这个用来对D_Term进行低通滤波
			last_RateError[axis]  = RateError;	//算D_Term本来就需要用到前一次的值
					
			DTerm =  dynD8[axis] *  DTerm / 1000;			//d:25   除以10000 =0.0025
			axisPID[axis] =PTerm + DTerm;
		} 
		else 
		{                // YAW axis
			error = (int32_t) RC_Command[axis] * 10 * 8 / cfg.P8[axis];  //32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
      error -= gyroData[axis];
      errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000);     // WindUp // 16 bits is ok here    
      if (abs(gyroData[axis]) > 640)
				errorGyroI[axis] = 0;
			ITerm = (errorGyroI[axis] / 125 * cfg.I8[axis]) >> 6;       // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000


      PTerm = RC_Command[axis] * 2 ;
			PTerm -= (int32_t) gyroData[axis] * dynP8[axis] / 10 / 8;       // 32 bits is needed for calculation
		
			delta = gyroData[axis] - lastGyro[axis];        //16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
			lastGyro[axis] = gyroData[axis];
			deltaSum = delta1[axis] + delta2[axis] + delta;
			delta2[axis] = delta1[axis];
			delta1[axis] = delta;

			DTerm = ((int32_t) deltaSum * dynD8[axis]) >> 5;        //32 bits is needed for calculation
			axisPID[axis] = PTerm + ITerm - DTerm;
			if(cfg.mixerConfiguration==MULTITYPE_HEX6X)         //六轴单电机停转关键代码
				axisPID[axis] = constrain(axisPID[axis] , -200 ,200);
	
    }
  }
}
