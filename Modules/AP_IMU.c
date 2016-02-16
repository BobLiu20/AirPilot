/**
	******************************************************************************
	* @file    AP_IMU.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   计算姿态Attitude，得到机身倾角和方位
	******************************************************************************
**/
#include "AP_IMU.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
int16_t gyroData[3] = { 0, 0, 0 },accSmooth[3];
int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
int16_t heading,magHold;//当前灰机的角度(-180~180),定向角度
int16_t accZ;
int16_t acc_25deg = 256 * 0.423f;
static float AdvAcc[2];
static float AdvAccLPF[2];

static int16_t accZoffset = 0; // = acc_1G*6; //58 bytes saved and convergence is fast enough to omit init

/* OS -----------------------------------------------*/
static  void  IMU_MainTask (void *p_arg);
static  OS_TCB  IMU_MainTaskTCB;
static  CPU_STK  IMU_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
static void IMU_GetEstimatedAttitude(void);
static void IMU_Compute(void);

/**
  * @brief  初始化
  *         创建任务
  * @param  None
  * @retval None
  */
void IMU_Init(void)
{
	OS_ERR	err;
	OSTaskCreate((OS_TCB   *)&IMU_MainTaskTCB,  
				 (CPU_CHAR     *)"IMU_MainTask",
				 (OS_TASK_PTR   )IMU_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_IMU ,
				 (CPU_STK      *)&IMU_MainTaskStk[0],
				 (CPU_STK_SIZE  )IMU_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
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
static  void  IMU_MainTask (void *p_arg)
{
	OS_ERR	err;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(1,OS_OPT_TIME_DLY,&err);//ms
		IMU_Compute();
  }
}

/**
  * @brief  计算计算
  *         
  * @param  
  * @param  
  * @retval 
  */
#define GYRO_INTERLEAVE
static void IMU_Compute(void)
{
	OS_ERR err;
	uint32_t axis;
	static int16_t gyroADCprevious[3] = { 0, 0, 0 };
	int16_t gyroADCp[3];
	int16_t gyroADCinter[3];
//	static uint32_t timeInterleave = 0;
	static int16_t gyroYawSmooth = 0;

	if (Sensors(SENSOR_ACC)) 	  //如果存在加速度
	{
		Acc_GetADC();			  //获取加速度值
		IMU_GetEstimatedAttitude();	  //得到姿态
  }
	
	Gyro_GetADC();				  //得到陀螺仪的值

	for (axis = 0; axis < 3; axis++) 
	{
#ifdef GYRO_INTERLEAVE
		gyroADCp[axis] = gyroADC[axis];
#else
		gyroData[axis] = gyroADC[axis];
#endif
		if (!Sensors(SENSOR_ACC))  //如果没有加速度设备就清除其值
			accADC[axis] = 0;
	}

	//annexCode();	  //空闲时间调用的代码,和上位机调试通讯的也在里面调用

#ifdef GYRO_INTERLEAVE
	OSTimeDly(1,OS_OPT_TIME_DLY,&err);//两次读取陀螺仪时间不能少于650us，转换时间
  Gyro_GetADC();	  //再次获取陀螺仪值
  for (axis = 0; axis < 3; axis++) 
	{
		gyroADCinter[axis] = gyroADC[axis] + gyroADCp[axis];
		// empirical, we take a weighted value of the current and the previous values
		gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
		gyroADCprevious[axis] = gyroADCinter[axis] / 2;
		if (!Sensors(SENSOR_ACC))
			accADC[axis] = 0;
  }
#endif

	if (0) 	//平滑处理			   feature(FEATURE_GYRO_SMOOTHING)
	{
		static uint8_t Smoothing[3] = { 0, 0, 0 };
		static int16_t gyroSmooth[3] = { 0, 0, 0 };
    if (Smoothing[0] == 0) 
		{
      // initialize
			Smoothing[ROLL] = (cfg.gyro_smoothing_factor >> 16) & 0xff;
			Smoothing[PITCH] = (cfg.gyro_smoothing_factor >> 8) & 0xff;
			Smoothing[YAW] = (cfg.gyro_smoothing_factor) & 0xff;
    }
    for (axis = 0; axis < 3; axis++) 
		{
      //   gyroData[axis] = (int16_t)(((int32_t)((int32_t)gyroSmooth[axis] * (Smoothing[axis] - 1)) + gyroData[axis] + 1 ) / Smoothing[axis]);
			gyroData[axis] = (int16_t) (gyroSmooth[axis] * 0.6 + gyroData[axis]*0.4 ) ;
      gyroSmooth[axis] = gyroData[axis];
    }
  } 
	else if (cfg.mixerConfiguration == MULTITYPE_TRI) 
	{
		gyroData[YAW] = (gyroYawSmooth * 2 + gyroData[YAW]) / 3;
    gyroYawSmooth = gyroData[YAW];
  }
}

/*
*********************************************************************************************************
*                                         姿态Attitude
*********************************************************************************************************
*/
#define GYR_CMPFM_FACTOR 1000.0f
#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)cfg.gyro_cmpf_factor + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#define GYRO_SCALE ((1998.0f * M_PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     // 32767 / 16.4lsb/dps for MPU3000
// #define GYRO_SCALE ((2380 * M_PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     //should be 2279.44 but 2380 gives better result (ITG-3200)
// +-2000/sec deg scale
//#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
// +- 200/sec deg scale
// 1.5 is emperical, not sure what it means
// should be in rad/sec

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
static void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;
    v->Z -= delta[ROLL] * v_tmp.X + delta[PITCH] * v_tmp.Y;
    v->X += delta[ROLL] * v_tmp.Z - delta[YAW] * v_tmp.Y;
    v->Y += delta[PITCH] * v_tmp.Z + delta[YAW] * v_tmp.X;
}

static int16_t _atan2f(float y, float x)
{
    // no need for aidsy inaccurate shortcuts on a proper platform
    return (int16_t)(atan2f(y, x) * (180.0f / M_PI * 10.0f));
}


static int32_t isq(int32_t x){return x * x;}

#define  ACC_LPF_FOR_VELOCITY  4
float accLPFVel[3];
t_fp_vector EstG;
#define ACC_Z_DEADBAND (acc_1G>>5) 

#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }
	

static void IMU_GetEstimatedAttitude(void)
{
	uint32_t axis;
	int32_t accMag = 0;
//static t_fp_vector EstG;
	static t_fp_vector EstM;
#if defined(MG_LPF_FACTOR)
	static int16_t mgSmooth[3];
#endif
	static float accLPF[3];
	static uint32_t previousT;
	uint32_t currentT = TimUsGet();
	float scale, deltaGyroAngle[3];
	float invG;
	uint32_t dt;
	static float accVelScale = 9.80665f / 10000.0f / 256 ;
	dt=currentT-previousT;//两次进入的时间间隔，us
	scale = (currentT - previousT) * GYRO_SCALE;
	previousT = currentT;

	// Initialization
	for (axis = 0; axis < 3; axis++) 
	{
		deltaGyroAngle[axis] = gyroADC[axis] * scale;
		if (cfg.acc_lpf_factor > 0) 
		{
			accLPF[axis]    = accLPF[axis]    * (1.0f - (1.0f / cfg.acc_lpf_factor)) + accADC[axis] * (1.0f / cfg.acc_lpf_factor);			   //cfg.acc_lpf_factor = 100;
			accLPFVel[axis] = accLPFVel[axis] * (1.0f - (1.0f / ACC_LPF_FOR_VELOCITY)) + accADC[axis] * (1.0f/ACC_LPF_FOR_VELOCITY);
			accSmooth[axis] = accLPF[axis];
		} 
		else 
		{
			accSmooth[axis] = accADC[axis];
		}
		accMag += (int32_t)accSmooth[axis] * accSmooth[axis];

		if (Sensors(SENSOR_MAG)) 
		{
#if defined(MG_LPF_FACTOR)
			mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
			#define MAG_VALUE mgSmooth[axis]
#else
			#define MAG_VALUE magADC[axis]
#endif
		}
	}
  accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);

  rotateV(&EstG.V, deltaGyroAngle);
	if (Sensors(SENSOR_MAG))
			rotateV(&EstM.V, deltaGyroAngle);

	if (abs(accSmooth[ROLL]) < acc_25deg && abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0)
			f.SMALL_ANGLES_25 = 1;
	else
			f.SMALL_ANGLES_25 = 0;

  // Apply complimentary filter (Gyro drift correction) 
	//互补滤波器（cmpf，complimentary pass filter）
  // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
	if ((90 < accMag && accMag < 110) || f.SMALL_ANGLES_25)
	{
		for (axis = 0; axis < 3; axis++)
			EstG.A[axis] = (EstG.A[axis] * (float)cfg.gyro_cmpf_factor + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;

// 		AdvAcc[ROLL] = EstG.A[ROLL] - accSmooth[ROLL]; //- acc_1G * ((float)angle[ROLL]/573);
// 		AdvAcc[PITCH] = EstG.A[PITCH] - accSmooth[PITCH];

// 		AdvAccLPF[ROLL]=AdvAccLPF[ROLL] * (1.0f - (1.0f / 50)) + AdvAcc[ROLL]  * (1.0f / 50);
// 		AdvAccLPF[PITCH]=AdvAccLPF[PITCH] * (1.0f - (1.0f / 50)) + AdvAcc[PITCH]  * (1.0f / 50);
  }
	else
	{
		AdvAcc[ROLL] = 0;
		AdvAcc[PITCH] = 0;
		AdvAccLPF[ROLL]=0;
		AdvAccLPF[PITCH]=0;
	}

	if (Sensors(SENSOR_MAG)) 
	{
		for (axis = 0; axis < 3; axis++)
			EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
	}

	// Attitude of the estimated vector
	angle[ROLL] = _atan2f(EstG.V.X, EstG.V.Z);
	angle[PITCH] = _atan2f(EstG.V.Y, EstG.V.Z);
	
		// projection of ACC vector to global Z, with 1G subtructed
	// Math: accZ = A * G / |G| - 1G
  	    invG = sqrt(isq(EstG.V.X) + isq(EstG.V.Y) + isq(EstG.V.Z));
  	    accZ = (accLPFVel[ROLL] * EstG.V.X + accLPFVel[PITCH] * EstG.V.Y + accLPFVel[YAW] * EstG.V.Z) / invG - acc_1G; 

			if (!f.ARMED) {
				accZoffset -= accZoffset>>3;
				accZoffset += accZ;
			}  
			accZ -= accZoffset>>3;
			applyDeadband(accZ, ACC_Z_DEADBAND);
			vel+= accZ * accVelScale * dt;		//靠加速度计算来的速度

#ifdef MAG
	if (Sensors(SENSOR_MAG))
	{
		// Attitude of the cross product vector GxM
		heading = _atan2f(EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X, EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z);
		heading = heading + MagneticDeclination;
		heading = heading / 10;

		if (heading > 180)
			heading = heading - 360;
		else if (heading < -180)
			heading = heading + 360;
	}
#endif
}
