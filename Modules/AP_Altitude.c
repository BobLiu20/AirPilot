/**
	******************************************************************************
	* @file    AP_Altitude.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   �õ���ǰ�߶ȣ������������PID������ֵ
	******************************************************************************
**/
#include "AP_Altitude.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
uint16_t CalibratingB = 100;//У׼��ѹ�ƣ��������Ϊ0��,ֻ�轫�丳ֵΪ����0����������100������
int32_t BaroGroundPressure;	   //��ѹ�Ƶ���У׼ֵ����׼
int32_t  EstAlt_Baro;             // in cm ��ǰ��ѹ�߶�ֵ
int16_t  errorAltitudeI = 0;
float vel;

float Sonar_I_Term = 0;

int16_t BaroPID = 0;		//��ѹ�Ƽ���õ���PIDֵ
int16_t SonarPID = 0;		//����������õ���PIDֵ

float  AltHold_Baro;		//��ѹ�ƶ��߸߶�
float AltHold_Sonar; //�������߸߶�

static int16_t Altitude_ThrottleHold;//�������ߺ���������
int16_t Altitude_HoldRcCommand;			 //�������ߺ��RcCommand[THROTTLE]ֵ
int16_t RawRcCommand_Val;

float AltHold_TakeOff=150.0f;//�Զ���ɶ��߸߶ȣ�����ɸ߶�
uint8_t TakeOff_FirstIn=0;//��һ�ν����Զ����

/* OS ----------------------------------------------------------------*/
static  void  Altitude_MainTask (void *p_arg);
static  OS_TCB  Altitude_MainTaskTCB;
static  CPU_STK  Altitude_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
static void Altitude_GetBaroPID(void);
static void Altitude_HoldMode(void);
static void Altitude_GetSonarPID(void);
static void Air_AutoLanding(void);
static void Air_AutoTakeoff(void);

/**
  * @brief  ��ʼ��
  *         ��������
  * @param  None
  * @retval None
  */
void Altitude_Init(void)
{
	OS_ERR	err;
	
	if (!Sensors(SENSOR_BARO))return ;//��������ѹ�ƾͲ���ʼ��
#ifdef SONAR
	Sonar_Init();//����ģ���ʼ��
#endif	
	OSTaskCreate((OS_TCB   *)&Altitude_MainTaskTCB,  
				 (CPU_CHAR     *)"Altitude_MainTask",
				 (OS_TASK_PTR   )Altitude_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_ALTITUDE ,
				 (CPU_STK      *)&Altitude_MainTaskStk[0],
				 (CPU_STK_SIZE  )Altitude_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
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
static  void  Altitude_MainTask (void *p_arg)
{
	OS_ERR	err;
	static uint8_t count = 0;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(25,OS_OPT_TIME_PERIODIC,&err);//40hz
		
		if(++count>=3)//ÿ��75MS����һ��
		{
#ifdef SONAR
			Sonar_Update();//���³�������,�������ʱ1MS
#endif
			Altitude_GetSonarPID();
			count = 0;
		}
		
		if(RC_Options[BOXLANDING])//�Զ�����
  		f.AUTOLANDING=1;
//  		else 
//  			f.AUTOLANDING=0;

		Air_AutoLanding();//�Զ�����
		Air_AutoTakeoff();
		
		
		Altitude_GetBaroPID();			//��ѹ��PID
		Altitude_HoldMode();				//����ģʽѡ��
		
		debug[2]=AltHold_Sonar;
		debug[3]=EstAlt_Sonar;
  }
}

/**
  * @brief  �Զ����
  *         
  * @param  None
  * @retval None
  */
static void Air_AutoTakeoff(void)
{
	static uint8_t first_flag=0;
	OS_ERR err;
	
	//static float temp_baro_pid_d;
	
	if(RC_Data[AUX2]<1300&&RC_Data[THROTTLE]>1300&&RC_Data[YAW]>1900)//RC_Data[AUX1]<1300&&
	{
		f.ARMED = 1;
		f.AUTOTAKEOFF=1;
	}
	
	if(!f.AUTOTAKEOFF)
	{
		first_flag=0;
		return ;
	}
	
	if(!RC_Options[BOXBARO])//û������ģʽ
	{
		//cfg.activate[BOXBARO]=0xffff;//ǿ�ƿ�������
		f.AUTOTAKEOFF = 0;
	}
	
	if(first_flag==0)
	{
		first_flag = 1;
		TakeOff_FirstIn = 1;//��ǵ�һ�ν����Զ����
		f.GPS_HOLD_MODE = 0;//�Զ������һ��GPS
		//temp_baro_pid_d = cfg.D8[PIDALT];
		//cfg.D8[PIDALT] = 40;
		
		Altitude_HoldRcCommand = 1200;
		OSTimeDly(2000,OS_OPT_TIME_DLY,&err);
// 		for(;Altitude_HoldRcCommand < 1300;)//��һ�����Զ���ɽ��������������������żӴ�
// 		{
// 			Altitude_HoldRcCommand++;
// 			OSTimeDly(10,OS_OPT_TIME_DLY,&err);
// 		}
		
		f.BARO_MODE = 0;
		f.SONAR_MODE = 0;
// 		AltHold_Baro = ;
	}
	
	if(f.SONAR_MODE && AltHold_Sonar>AltHold_TakeOff)
	{
		f.AUTOTAKEOFF = 0;
	}
	else
		AltHold_Sonar+=1.0f;
	
	if(f.BARO_MODE && !f.SONAR_MODE && AltHold_Baro>AltHold_TakeOff)
	{
		//cfg.D8[PIDALT] = temp_baro_pid_d;
		f.AUTOTAKEOFF = 0;
	}
	else
		AltHold_Baro+=3.0f;

	
// 	if(f.SONAR_MODE)
// 	{
// 		f.BARO_MODE
// 	}
}


/**
  * @brief  �Զ�����
  *         75ms����һ��,��f.AUTOLANDINGΪ1ʱ�ŵ��ô˺���
  * @param  None
  * @retval None
  */
static void Air_AutoLanding(void)
{
// 	static float alt_last=0;
// 	static uint8_t t_count=0;
	//static float temp_baro_pid_d;
	OS_ERR err;
	
	if(!f.AUTOLANDING)return ;
	
// 	if(!RC_Options[BOXBARO])//û������ģʽ
// 	{
// 		cfg.activate[BOXBARO]=0xffff;//ǿ�ƿ�������
// 	}
	
// 	if(cfg.D8[PIDALT] != 40)
// 	{
// 		//temp_baro_pid_d = cfg.D8[PIDALT];
// 		cfg.D8[PIDALT] = 40;
// 	}
	
	if(f.SONAR_MODE)
	{
// 		if(AltHold_Sonar > -50)   AltHold_Sonar -= 1;
// 		else if(EstAlt_Sonar<10)
// 		{
// 			f.AUTOLANDING = 0;
// 			for(;Altitude_HoldRcCommand > 1100;)//�������������������Ž���
// 			{
// 				Altitude_HoldRcCommand-=1;
// 				OSTimeDly(10,OS_OPT_TIME_DLY,&err);
// 			}
// 			f.ARMED = 0;//����
// 		}
// 		alt_last = 0;//���
// 		t_count = 0;
	}
	else if(f.BARO_MODE)
	{
		if(AltHold_Baro > 800.0f)AltHold_Baro -= 3;
		else if(AltHold_Baro > 500.0f&&EstAlt_Baro < 850.0f)AltHold_Baro -= 2;
		else if(EstAlt_Baro<550.0f)AltHold_Baro -= 1;
// 		if(abs(EstAlt_Baro-alt_last)<20)
// 			t_count++;
// 		else
// 		{
// 			alt_last=EstAlt_Baro;
// 			t_count=0;
// 		}
// 		if(t_count>100)//�߶�ֵ��7.5s���ޱ仯����Ϊ��½��
// 		{
// 			f.AUTOLANDING = 0;
// 			for(;Altitude_HoldRcCommand > 1100;)//�������������������Ž���
// 			{
// 				Altitude_HoldRcCommand--;
// 				OSTimeDly(20,OS_OPT_TIME_DLY,&err);
// 			}
// 			f.ARMED = 0;//����
// 			
// 			//cfg.D8[PIDALT] = temp_baro_pid_d;
// 		}
	}
}



/**
  * @brief  ����ģʽ
  *         ��ѹ�ƶ��ߺͳ����������Զ��л����Գ������������ȣ����������Чʱ�Զ��л�����ѹ��
  * @param  None
  * @retval None
  */
static void Altitude_HoldMode(void)
{
	static int16_t rc_data_old = 0;
	static int8_t rc_ctrl=0;
	

	//�ֱ�ģʽ�½��е����⴦��
// 	if(f.HANDLE_CTRL_MODE)
// 	{
// 		if(RC_Data[THROTTLE]<1500)//���ֱ������£������Ż����ӵ�ʱ�򲻽��ж��ߴ���
// 		{
// 			errorAltitudeI = 0;
// 			f.BARO_MODE = 0;
// 			f.SONAR_MODE = 0;
// 			AltHold_Baro = EstAlt_Baro;    //��û���ߵ�ʱ���������ֵ�����Ա��ⶨ�߿���ʱPID�д������
// 			return ;
// 		}
// 	}

	//�ڶ��߹��������ű䶯����������ݵĶ��߹��ܹرգ����л�������ֱ��
	if(abs(RC_RawData[THROTTLE] - rc_data_old)>20)
	{
		f.BARO_MODE = 0;   // so that a new althold reference is defined
		f.SONAR_MODE = 0;
		rc_ctrl=1;
	}
	if(rc_ctrl>0)
	{
		rc_data_old=RC_RawData[THROTTLE];
		rc_ctrl++;
		if(rc_ctrl>10)rc_ctrl=0;//�����ﶨ������ݹرն��ߵ�ʱ��
		AltHold_Baro = EstAlt_Baro;    //��û���ߵ�ʱ���������ֵ�����Ա��ⶨ�߿���ʱPID�д������
		return ;
	}

	if(RC_Options[BOXBARO])//��������ģʽ
	{
		if(!f.BARO_MODE)//û�п�����ѹ����ʱ�����뿪�������ܴ򿪺�ֻ���һ��
		{
			f.BARO_MODE = 1;		//��������ģʽ
			
// 			if(f.HANDLE_CTRL_MODE)//�ֱ�ģʽ��
// 			{
// 				Altitude_ThrottleHold = 1500;//�̶�Ϊ1500
// 				AltHold_Baro = 100.0f;//�ֱ�����ɼ��������趨Ϊ1M
// 			}
// 			else //ң������
// 			{
				if(f.AUTOTAKEOFF&&(!f.SONAR_AVAILABLE&&TakeOff_FirstIn))//��һ�����޳���
				{
					AltHold_Baro = EstAlt_Baro+50.0f;
					TakeOff_FirstIn=0;
				}
				else
					AltHold_Baro = EstAlt_Baro;		//����ǰ�߶��趨Ϊ���ָ߶�
				Altitude_ThrottleHold = RawRcCommand_Val;//RC_Command[THROTTLE];////��ǰ�����趨Ϊ��������
// 			}
			rc_data_old = RC_RawData[THROTTLE];
			errorAltitudeI = 0;
			BaroPID = 0;
		}
		//��GPSģʽ�£�ֻ�е����Զ���ɺ��Զ�����ʱ��ʹ�ó�����
		if(f.SONAR_AVAILABLE)//&&(!RC_Options[BOXGPSHOLD] || (RC_Options[BOXGPSHOLD]&&(f.AUTOLANDING || (f.AUTOTAKEOFF &&TakeOff_FirstIn)))))//���ڳ�����ģ���ҳ�����������Ч
		{
			if(!f.SONAR_MODE)//����
			{
				f.SONAR_MODE = 1;
				if(f.AUTOTAKEOFF&&TakeOff_FirstIn)
				{
					TakeOff_FirstIn = 0;
					AltHold_Sonar = EstAlt_Sonar;//+30.0f;
				}
				else
					AltHold_Sonar = EstAlt_Sonar;		//����ǰ�߶��趨Ϊ���ָ߶�
				SonarPID = 0;
				Sonar_I_Term = 0;
			}
		}
		else
		{
			if(f.SONAR_MODE)
			{
				f.SONAR_MODE = 0;//��������Ч��ر�
				AltHold_Baro = EstAlt_Baro;		//����ǰ�߶��趨Ϊ���ָ߶�
				errorAltitudeI = 0;	
				BaroPID = 0;
			}
		}
	}
	else
	{
		f.BARO_MODE = 0;
		f.SONAR_MODE = 0;
		AltHold_Baro = EstAlt_Baro;    //��û���ߵ�ʱ���������ֵ�����Ա��ⶨ�߿���ʱPID�д������
	}

	if(f.BARO_MODE)//����ģʽ
	{
		if(f.SONAR_MODE)
			Altitude_HoldRcCommand = Altitude_ThrottleHold + SonarPID;
		else
			Altitude_HoldRcCommand = Altitude_ThrottleHold + BaroPID;
		Altitude_HoldRcCommand = constrain(Altitude_HoldRcCommand, cfg.mincheck, 2000);
	}
}



#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }

static float InvSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}
static int32_t isq(int32_t x){return x * x;}
/**
  * @brief  ʹ����ѹ�Ƽ���߶�
  *         25ms����һ�Σ�40HZ
  * @param  None
  * @retval None
  */



static void Altitude_GetBaroPID(void)
{
	int16_t error;
//	float invG;
// 	int16_t accZ;
//	static float vel = 0.0f;
	static float accVelScale = 9.80665f / 10000.0f / 256 ;
	static float baroOffset = 0.0f;
	int32_t  BaroAlt;
	static int32_t lastBaroAlt;
  float baroVel;
	float vel_tmp;
	float boostGain;
	uint32_t dt,n_time;
	static uint32_t p_time=0;
	n_time=TimUsGet();
	dt=n_time-p_time;//���ν����ʱ������us
	p_time=n_time;
	
	if(CalibratingB > 0) //У׼����
	{
	  BaroGroundPressure = BaroPressureSum/(float)(BARO_TAB_SIZE - 1) + 0.5f; // +0.5 for correct rounding
	  CalibratingB--;
  }
	// log(0) is bad!	log(0)������
	if(BaroGroundPressure != 0) 
	{
		// pressure relative to ground pressure with temperature compensation (fast!)
		// see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
		BaroAlt = log( BaroGroundPressure / (BaroPressureSum/(float)(BARO_TAB_SIZE - 1)) ) * (BaroTemperature+27315) * 29.271267f; // in cemtimeter 
		//BaroAlt = (1.0f - pow(((baroPressureSum/(float)(BARO_TAB_SIZE - 1))) / 101325.0f, 0.190295f)) * 4433000.0f;
	}
	else 
	{
		BaroAlt = 0;
	}

	EstAlt_Baro = (EstAlt_Baro * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 ��s)
//  EstAlt_Baro = (EstAlt_Baro * 0.95 + BaroAlt * 0.05);
	if(f.SONAR_AVAILABLE)
	{
		baroOffset = baroOffset * 0.995 + ( EstAlt_Sonar -  EstAlt_Baro) * 0.005 ;
	}
	baroOffset = constrain(baroOffset, -150 ,150);
	EstAlt_Baro +=  baroOffset;
	//P
	error = constrain(AltHold_Baro - EstAlt_Baro, -1000, 1000);
	applyDeadband(error, 10); //remove small P parametr to reduce noise near zero position
	BaroPID = constrain((cfg.P8[PIDALT] * error / 10), -300, +300);
// 	boostGain = constrain(0.5 * error + 0.75  , 1.0 ,3.0);
// 	BaroPID *= boostGain;

//	Xbee_Debug[2]=error;
	//I
	errorAltitudeI += error * cfg.I8[PIDALT]/50;
	errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
	BaroPID += (errorAltitudeI / 200); //I in range +/-150

// 	// projection of ACC vector to global Z, with 1G subtructed
// 	// Math: accZ = A * G / |G| - 1G
// 	invG = InvSqrt(isq(EstG.V.X) + isq(EstG.V.Y) + isq(EstG.V.Z));
// 	accZ = (accLPFVel[ROLL] * EstG.V.X + accLPFVel[PITCH] * EstG.V.Y + accLPFVel[YAW] * EstG.V.Z) * invG - acc_1G; 
	//int16_t accZ = (accLPFVel[ROLL] * EstG.V.X + accLPFVel[PITCH] * EstG.V.Y + accLPFVel[YAW] * EstG.V.Z) * invG - 1/invG; 
//	applyDeadband(accZ, acc_1G/50);
	

	
	if (f.BARO_MODE )// && !f.SONAR_MODE
	{
		// Integrator - velocity, cm/sec
//		vel+= accZ * accVelScale * dt;		//�����ٶȼ��������ٶ�

		baroVel = (EstAlt_Baro - lastBaroAlt) * 1000000.0f / dt;			 //����ѹ���������ٶ�
		lastBaroAlt = EstAlt_Baro;

		baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
		applyDeadband(baroVel, 10); // to reduce noise near zero  

		// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
		// By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
//		if( !f.SONAR_MODE )
			vel = vel * 0.985f + baroVel * 0.015f;		  //�����˲���
		//vel = constrain(vel, -300, 300); // constrain velocity +/- 300cm/s   

		//Xbee_Debug[0]=vel;
		//D
		vel_tmp = vel;
		applyDeadband(vel_tmp, 5);
		//   vario = vel_tmp;
		BaroPID -= constrain(cfg.D8[PIDALT] * vel_tmp / 20, -300, 300);

//		Xbee_Debug[1]=BaroPID;
	}
	else
	{
		vel = 0;
		lastBaroAlt = EstAlt_Baro;
	}
}

#ifdef SONAR
static void Altitude_GetSonarPID(void)
{
	s16	SonarAltError=0;
	s16 P_Term,D_Term=0;
	static s16 Last_DTerm = 0;
//	static float vel = 0.0f;
//	static float accVelScale = 9.80665f / 10000.0f / 256 ;
	uint32_t n_time;
	float dt;
	static uint32_t p_time=0;
	static float PreSonarDistance = 0;//��һ�εĵĳ����߶�
	float SonarVel,DesireSonarVel;
	float ErrorVel;



	static PID_DATA Sonar_rate_piddata;
	static PID_PARAM Sonar_rate_pidparam;
	
	float P_Sonar=1.0f;	
	float I_Sonar=0.0f;	
	float D_Sonar=1.0f;
// 	float P_Rate_Sonar=1.5f;
// 	float I_Rate_Sonar=1.0f;	
// 	float D_Rate_Sonar=1.0f;
//	static float LastaccZ;	
	Sonar_rate_pidparam.kP = 1.5f;  
	Sonar_rate_pidparam.kI = 0.8f;
	Sonar_rate_pidparam.kD = 0.0f;    
	Sonar_rate_pidparam.Imax = 100.0f;
	Sonar_rate_pidparam.apply_lpf = 1;
	


	
	n_time=TimUsGet();
	dt=(n_time-p_time) * 0.000001f;//���ν����ʱ������s
	p_time=n_time;

	if(!f.SONAR_MODE)
	{
			PID_reset(&Sonar_rate_piddata);

//			return ;
	}


	SonarAltError = constrain(AltHold_Sonar - EstAlt_Sonar , -100 ,100);
	applyDeadband(SonarAltError , 5); // to reduce noise near zero  

	
	SonarVel = (EstAlt_Sonar - PreSonarDistance) / dt;  //  cm/s  ��������	
	PreSonarDistance = EstAlt_Sonar;
	
//	vel+= accZ * accVelScale * dt;		//�����ٶȼ��������ٶ�


	// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
	// By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
//	vel = vel * 0.70f + SonarVel * 0.30f;		  //�����˲���
//	applyDeadband(SonarVel, 5); // to reduce noise near zero  
//	vel = SonarVel;		  //�����˲���
	DesireSonarVel = P_Sonar * SonarAltError ;//- D_Sonar * SonarVel

	if(f.AUTOLANDING&&f.SONAR_MODE)
	{
		static uint8_t unarm_cnt=0;
		DesireSonarVel =  -10.0f;
		if(EstAlt_Sonar<10.0f)
		{
			if(unarm_cnt++>2)//ֻ������3���жϵ�С��15��������
			{
				f.ARMED = 0;//����
				f.AUTOLANDING=0;
			}
		}
		else
			unarm_cnt = 0;
	}
		ErrorVel = DesireSonarVel - SonarVel;
// 	P_Term = P_Rate_Sonar * ErrorVel;

// //	applyDeadband(accZ, acc_1G/50);   
// //	accZ = constrain(accZ , -100,100);
// //	accZ = LastaccZ * 0.80 + accZ * 0.20 ;
// //	LastaccZ = accZ;
// 	
// //	D_Term = D_Rate_Sonar * ((float)accZ * accScale);   // ��λת���� cm/s2

// 	LastSonarVel = SonarVel;


// // 		D_Term = Last_DTerm * 0.8 + D_Term * 0.2;
// // 		Last_DTerm = D_Term;
// 	
// 	Sonar_I_Term += I_Rate_Sonar * ErrorVel  * dt; 
// 	Sonar_I_Term = constrain(Sonar_I_Term , -100, 100);
// 	
// 	SonarPID = P_Term  +  Sonar_I_Term 	;			 //  - D_Term;     - D_Rate_Sonar*D_Term

	SonarPID = apply_pid(&Sonar_rate_piddata, ErrorVel, dt, &Sonar_rate_pidparam) ;	
//	+=DeltaSonarPID;

	SonarPID = constrain(SonarPID,-400,400);


}
#endif

