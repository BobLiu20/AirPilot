/**
	******************************************************************************
	* @file    AP_Actuator.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   ������
	******************************************************************************
**/
#include "AP_Actuator.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
uint8_t RC_Options[CHECKBOXITEMS];//ң��������ͨ���Ĺ���ѡ��
int16_t RC_Command[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW ��������������RC_Data-1500�õ���

static uint8_t dynP8[3], dynD8[3];//dynI8[3], 
int16_t axisPID[3];
static float errorAngleI[2] = { 0, 0 };

int16_t HeadFreeModeHold;		//��ͷģʽ

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
  * @brief  ��ʼ��
  *         ��������
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
  * @brief  Actuator������
  *         �������������ɿص�����
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
		Actuator_ModeState();//����AUX����ģʽ
		Actuator_AnnexCode();
		Actuator_MainSynth();//�ۺ�
		OSFlagPost(&Mixer_TaskRunFlagGrp,//���������
							(OS_FLAGS)0x01,
							OS_OPT_POST_FLAG_SET,
							&err);
  }
}


/**
  * @brief  ģʽѡ�����AUXͨ���������ùرո���ģʽ����
  *         
  * @param  None
  * @retval None
  */
static void Actuator_ModeState(void)
{
	uint8_t i;
	uint16_t auxState = 0;
	for(i = 0; i < 4; i++)	   //�õ��¸���ͨ����״̬��ÿ3λ����һ��ͨ��
		auxState |= (RC_Data[AUX1 + i] < 1300) << (3 * i) | (1300 < RC_Data[AUX1 + i] && RC_Data[AUX1 + i] < 1700) << (3 * i + 1) | (RC_Data[AUX1 + i] > 1700) << (3 * i + 2);
	for(i = 0; i < CHECKBOXITEMS; i++)//���������RC_Options[]�����ݣ�����ģʽ...
		RC_Options[i] = (auxState & cfg.activate[i]) > 0;

	if (RC_Options[BOXACC] && Sensors(SENSOR_ACC))
	{
		if (!f.ACC_MODE) 	//��������
		{
			errorAngleI[ROLL] = 0;
			errorAngleI[PITCH] = 0;
			f.ACC_MODE = 1;
		}
	} 
	else
		f.ACC_MODE = 0;

// 	if (Sensors(SENSOR_BARO))//���������ѹ�� 
// 	{
// 		if (RC_Options[BOXBARO] ) //�Ƿ�������				RC_Options[BOXBARO] ��ʱ������ѹ����ģʽ
// 		{
// 			if (!f.BARO_MODE) 			//���ܴ򿪺�ֻ���һ��
// 			{
// 				f.BARO_MODE = 1;		//��������ģʽ
// 				AltHold_Baro = EstAlt_Baro;		//����ǰ�߶��趨Ϊ���ָ߶�
// 				InitialThrottleHold = RC_Command[THROTTLE];	//��ǰ�����趨Ϊ��������
// 				errorAltitudeI = 0;	
// 				BaroPID = 0;
// 			}
// // #ifdef SONAR
// // 			if (sonar.flag) //���������ǲ�����Ч
// // 			{	
// // 					 
// // 				if (!f.SONAR_MODE) 
// // 				{
// // 					f.SONAR_MODE = 1;		//��������ģʽ
// // 					SonarAltHold_Baro = sonar.Distance;		//����ǰ�߶��趨Ϊ���ָ߶�
// // 								//    InitialThrottleHold = RC_Command[THROTTLE];	//��ǰ�����趨Ϊ��������
// // 										//errorAltitudeI = 0;
// // 					SonarPID = 0;
// // 					PreSonarDistance = sonar.Distance;
// // 					I_Term=0;
// // 				}
// // 			} 
// // 			else				   //������   ����Ҫ��������ЧҪ��������ع���  �´���Ч�Ż����³�ʼ���µĶ�������
// // 			{
// // 				if(f.SONAR_MODE)
// // 				{
// // 					f.BARO_MODE=0;	   //�ڳ�������Ч˲�䣬�������Ϊ0������ѹ�ƶ���ά�ִ˿̸߶�
// // 					f.SONAR_MODE = 0;
// // 				}
// // 			}
// // #endif
// 		}
// 		else				   //���߹���û���Ͱ�����ģʽ�Ŀ��ض�����
// 		{
// 			f.BARO_MODE = 0;
// //			f.SONAR_MODE = 0;
// 			AltHold_Baro = EstAlt_Baro;    //��û���ߵ�ʱ���������ֵ�����Ա��ⶨ�߿���ʱPID�д������
// 		}
// 	}


		
	if (Sensors(SENSOR_MAG))//������ڵ������� 
	{
		if (RC_Options[BOXMAG]) //����������
		{
			if (!f.MAG_MODE) 
			{
				f.MAG_MODE = 1;	  //��������
				magHold = heading;//����ǰ�����趨Ϊ���ַ���
			}
		} 
		else
			f.MAG_MODE = 0;		  //������
		if (RC_Options[BOXHEADFREE])//��ͷģʽ
		{
			if (!f.HEADFREE_MODE)  //�䶨��ķ�λ��ʵʱ�ֶ��ı�
			{		
				f.HEADFREE_MODE = 1;//������ģʽ
			}
		} 
		else 
		{
			f.HEADFREE_MODE = 0;
		}
		if (RC_Options[BOXHEADADJ])
		{
			HeadFreeModeHold = heading; //��ȡ�µķ���
		}
	}

	if (Sensors(SENSOR_GPS)) //����GPS�豸
	{
		if (f.GPS_FIX && GPS_numSat >= 5) //���GPSֵ��Ч������������5
		{

			if (RC_Options[BOXGPSHOLD]&&!((VisionLanding_Angle[0] || VisionLanding_Angle[1]) && nav_mode==NAV_MODE_POSHOLD)) 	   //&&!(f.AUTOTAKEOFF && EstAlt_Baro<400.0f)
			{//����ͣģʽ�����Ӿ���Чʱ���ر�GPS
				if (!f.GPS_HOLD_MODE)
				{
					f.GPS_HOLD_MODE = 1;   //����
					GPS_hold[LAT] = GPS_coord[LAT];	 //���浱ǰ��������
					GPS_hold[LON] = GPS_coord[LON];
					GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
					nav_mode = NAV_MODE_POSHOLD;
					XbeePro_ImportanceInfo = XbeePro_ImportanceInfo | 0x01;//���Ͷ���λ�ø���λ��
				}
			} 
			else 
			{
				f.GPS_HOLD_MODE = 0;
			}
			
	    if(RC_Options[BOXGPSHOME]) //�Ƿ���GO HOME
			{
				if(!f.GPS_HOME_MODE)
				{
					f.GPS_HOME_MODE = 1;
					GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);//�趨�һ�����һ���ص㻹�ǵ�ǰ�ص�,��ԭ�ر���
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
	
	//��һ�������µĵ�һ������Ҫ��ʱ�رն���
	if(f.BARO_MODE&&OneKey_RollStep!=1)RC_Command[THROTTLE]=Altitude_HoldRcCommand;//�ڶ���ģʽ��ʹ�ô�ֵ

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
		LEDFLASH_SET(LEDFLASH_SENSORCAL);//������˸
	}
	else 
	{
		LEDFLASH_CLR(LEDFLASH_SENSORCAL);
		if (!f.SMALL_ANGLES_25&&!f.ARMED)//�ڽ���ǰ���һ���ǳ���25
		{
			f.ACC_CALIBRATED = 0;//δУ׼,������ǰ�жϵ�������б������Ϊ�Ǽ��ٶȼ�ΪУ׼
			LEDFLASH_SET(LEDFLASH_AIRSLOPE);
		}
		else
		{
			f.ACC_CALIBRATED = 1;//��У׼
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
#define GYRO_RATE_SCALE    1998.0f/(32767.0f / 4.0f)	   //�˺������λ ����ÿ��
#define  MainSynth_FILTER     (1.0f / ( 2 * M_PI * 10 )) // 20hz�����˳����˲�����   ���һ�����ı���������˳�Ƶ��
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
	static uint32_t pre_time;//��һ�ε�ʱ���
	uint32_t cur_time;			 //��ǰ��ʱ���
	cur_time=TimUsGet();
	CycleTime=cur_time-pre_time;
	dt=CycleTime * 0.000001;//dt��λΪs
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

/*������*/
	if (Sensors(SENSOR_MAG)) 	 //�������MAG
	{
		if (abs(RC_Command[YAW]) < 70 && f.MAG_MODE) //���YAW���м䲻����70���ҿ����˶���ģʽ����
		{
			int16_t dif = heading - magHold;		//�õ���ֵ
			if (dif <= -180)						//����
				dif += 360;
			if (dif >= +180)
				dif -= 360;
			if (f.SMALL_ANGLES_25)
				RC_Command[YAW] -= dif * cfg.P8[PIDMAG] / 30;    // 18 deg
		}
		else
			magHold = heading;							//   ʶ��ΪYAW�䶯�������µ�headingֵ���·���
	}

	/*���߹���*/
// 	if (Sensors(SENSOR_BARO)) 
// 	{
// 		if (f.BARO_MODE)  //���붨�ߴ���||f.SONAR_MODE
// 		{
// 			if (abs(RC_Command[THROTTLE] - InitialThrottleHold) > 20)  
// 			{
// 				f.BARO_MODE = 0;   // so that a new althold reference is defined
// //				f.SONAR_MODE = 0;
// 			}

// // #ifdef SONAR
// // 				if(f.SONAR_MODE)	   //�ڳ���������Ч��   ���ϳ�������PID��
// // 					RC_Command[THROTTLE] = InitialThrottleHold + SonarPID;   // BaroPID +����PID������ֵ
// // 				else
// // #endif
// 			RC_Command[THROTTLE] = InitialThrottleHold + BaroPID;

// 				if(EasyLanding_MODE)		  //һ������
// 				{ 
// 					if(EstAlt_Baro <200 &&f.BARO_MODE)		   //�쵽���ˣ����٣�����Dֵ 
// 						cfg.D8[PIDALT] = D_ALT;
// 				#ifdef SONAR
// 					else if(sonar.Distance<150&&f.SONAR_MODE)
// 						cfg.D8[PIDALT] = D_ALT;
// 				#endif
// 					else
// 						cfg.D8[PIDALT] = 50;
					
//						if(EstAlt_Baro<100 && accZ<-10)
//						{
//							f.ARMED = 0;		//����
//							cfg.D8[PIDALT] = D_ALT;
//							EasyLanding_MODE = 0;
//						}
// 					if(f.SONAR_MODE)	  //���������Ч��������н���
// 					{
// 					#ifdef SONAR
// 						SonarAltHold_Baro-=constrain( SonarAltHold_Baro * 0.1f ,0.0 ,100.0) * dt;
// 					#endif
// 					}
// 					else
// 					{
// 						AltHold_Baro -= 	constrain( ( AltHold_Baro - (-200) ) * 0.1f ,0 ,100) * dt;   //ÿ�뽵��ľ���Ϊ��ǰ�߶�*0.1 �� ������������ٶ�Ϊ0.5m/s
// 					}
//		}
// 		if(EasyLaunching_MODE)  	  //һ�����   ����̨���øù���ֻ��Ҫ�������־λ��һ  ������һ��	  AltHold_Baro  ����һ����� �ǳ���
// 		{
// 			cfg.D8[PIDALT] = 20; 
// 			if(abs(EstAlt_Baro - AltHold_Baro)<200 )		   //�쵽���ˣ����٣�����Dֵ  ��ɱȽ���Ҫ�򵥵Ķ�
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
		// ��������Ƿ�Ҫ���뵼��
		if (( !f.GPS_HOLD_MODE) || (!f.GPS_FIX_HOME)) 	 //  !f.GPS_HOME_MODE &&
		{
			//�������Ҫ������������뵼����صĲ�����
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
//					last_nav_rated[LON] =   nav_rated[LON] ;	   //�������������D_Term���е�ͨ�˲�
//					nav_rated[LAT] = last_nav_rated[LAT] + (dt / (GPS_filter + dt)) * (nav_rated[LAT] - last_nav_rated[LAT]);
//					last_nav_rated[LAT] =   nav_rated[LAT] ;	   //�������������D_Term���е�ͨ�˲�

				GPS_angle[ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
				GPS_angle[PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
			}
			else
			{
				GPS_angle[ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
				GPS_angle[PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
			}

			if(nav_mode == NAV_MODE_POSHOLD)			  //����ͣģʽ������GPS��̬����ģʽ
			{
				if(abs(RC_Command[ROLL])<20 && abs(RC_Command[PITCH])<20)	  //�������������ͣ�£��˿���ң�����ƶ��ɻ�������λ��Ҳ���Ÿ���
				{
					GPS_Speed_Mode = 0;
					if(!GPS_coord_Saved && abs(actual_speed[LAT])<100 && abs(actual_speed[LON])<100)		   //��û������ͣλ��
					{
						GPS_hold[LAT] = GPS_coord[LAT];	 //���浱ǰ��������
						GPS_hold[LON] = GPS_coord[LON];
						GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
						GPS_coord_Saved = 1;
					}
				}
				else	 //�п��ƽ���  ����������  RC_Command���ٶȷ���   ���� 200 ����200cm/s���ٶ�
				{
					GPS_coord_Saved	= 0;   //�´λ������ͣλ��
					GPS_Speed_Mode = 1;
					GPS_target_speed[LAT] = (RC_Command[PITCH] * cos_yaw_x - RC_Command[ROLL] * sin_yaw_y);
					GPS_target_speed[LON] = (RC_Command[PITCH] * sin_yaw_y + RC_Command[ROLL] * cos_yaw_x);
				}
			}
	
//				GPS_angle[ROLL] = last_GPS_angle[ROLL] + (dt / (GPS_filter + dt)) * (GPS_angle[ROLL] - last_GPS_angle[ROLL]);
//				last_GPS_angle[ROLL] =   GPS_angle[ROLL] ;	   //�������������D_Term���е�ͨ�˲�
//				GPS_angle[PITCH] = last_GPS_angle[PITCH] + (dt / (GPS_filter + dt)) * (GPS_angle[PITCH] - last_GPS_angle[PITCH]);
//				last_GPS_angle[PITCH] =   GPS_angle[PITCH] ;	   //�������������D_Term���е�ͨ�˲�
		}
	}
	
	// **** ROLL=0 & PITCH=1 & YAW=2 PID ****    
	for (axis = 0; axis < 3; axis++) 
	{
		if (axis < 2) //����ģʽ
		{
			//������ȫ�µ�˫��Ƕ��PID�����㷨
			if(f.ACC_MODE&&!OneKey_RollStep)//OneKey_RollStep��Ϊ0ʱ˵������һ����������ر�����
			{
				if(Sensors(SENSOR_GPS) && f.GPS_HOLD_MODE && nav_mode==NAV_MODE_POSHOLD)
					errorAngle = constrain( GPS_angle[axis], -300, +300) - angle[axis] + cfg.angleTrim[axis];
				else
					errorAngle = constrain( RC_Command[axis] + GPS_angle[axis], -300, +300) - angle[axis] + cfg.angleTrim[axis];
				//�˹���̬����
				if(f.HANDLE_AVAILABLE)errorAngle+=(HandAttitude_CtrlAngle[axis]*5);//�����ֱ�ʱ�������ֱ�������
				
				errorAngle -= OpticalFlow_Angle[axis];
				errorAngle -= VisionLanding_Angle[axis];
				//errorAngle -= (Avoid_Angle[2*(1-axis)]-Avoid_Angle[2*(1-axis)+1]);
				if(axis==0) errorAngle += Avoid_Angle[2];//errorAngle -= (Avoid_Angle[3]-Avoid_Angle[2]);
				if(axis==1) errorAngle -= Avoid_Angle[0];//(Avoid_Angle[0]-Avoid_Angle[1]);
			}
			else
				errorAngle = 3 * constrain( RC_Command[axis] , -500, +500);

			
				//attPID = updatePID( attCmd[axis],  sensors.attitude200Hz[axis], dt, holdIntegrators, &systemConfig.PID[ROLL_ATT_PID ] );
			 //ע�� �� �������cfg.Pֵ�ǵ���������ֵ�Ŵ���ʮ��   ���� POS��P���Ŵ�100��
			 //Dֵû�б��Ŵ�  
			 //Iֵ���Ŵ���1000��    ֻ�е�����ص�3��I�������Ŵ�100��
				Level_PTerm = errorAngle * cfg.P8[PIDLEVEL] / 10 /10 ;	 //����ANGLE�ǷŴ���ʮ����ֵ ����Ҫ����10	 
//				Level_PTerm = constrain(Level_PTerm, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);		
		
				errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle * 10 * dt, -1500, +1500);  		//�൱��һ�����һ��errorAngle ��ʮ�� (25�� 2500) ����IӦȡ0.01����Ĳ���
			
				Level_ITerm = (errorAngleI[axis] * cfg.I8[PIDLEVEL])/1000;     // �ǵ�I���Ŵ���1000��������Ҫ����1000�Ի�ԭ�����õ�I������0.01����� 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

				target_rate = Level_PTerm + Level_ITerm ;	 //���Ϊ�� ���ȣ�*P

			gyro_rate =  gyroData[axis] * GYRO_RATE_SCALE;	   //ת����λΪ ��ÿ��

			RateError =  target_rate - gyro_rate ; 
			PTerm = RateError * dynP8[axis] / 10; 		  //p: 1.4 *10(���������˵�)  /100  =0.14

			DTerm  = (RateError- last_RateError[axis]) / dt;
	
			//D��Ը�Ƶ����ʮ�����У��п��ܻ��ƻ��������������ȶ�������Ҫ���е�ͨ�˲�
			DTerm = last_DTerm[axis] + (dt / (MainSynth_FILTER + dt)) * (DTerm - last_DTerm[axis]);
									
			last_DTerm[axis] =   DTerm ;	   //�������������D_Term���е�ͨ�˲�
			last_RateError[axis]  = RateError;	//��D_Term��������Ҫ�õ�ǰһ�ε�ֵ
					
			DTerm =  dynD8[axis] *  DTerm / 1000;			//d:25   ����10000 =0.0025
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
			if(cfg.mixerConfiguration==MULTITYPE_HEX6X)         //���ᵥ���ͣת�ؼ�����
				axisPID[axis] = constrain(axisPID[axis] , -200 ,200);
	
    }
  }
}
