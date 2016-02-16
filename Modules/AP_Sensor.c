/**
	******************************************************************************
	* @file    AP_Sensor.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   各种传感器在这里检测、初始化、读取等
	*					 在使用硬件I2C时，I2C工作在300k的速度时，读取一次需要290us
	*																				400k的速度时，读取一次需要230us
	*					 注：i2c最大速度为400k
	******************************************************************************
**/
#include "AP_Sensor.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
sensor_t acc;                       // acc access functions
sensor_t gyro;                      // gyro access functions
sensor_t mag;												// mag access functions
baro_t baro;                        // barometer access functions
float MagneticDeclination = 0.0f; //磁偏角校准值
uint16_t acc_1G=256;

uint16_t CalibratingA = 0;       //想进行加速度校准时，请将CalibratingA幅值为400
uint16_t CalibratingG = 0;			 //校准则赋值1000

int16_t gyroADC[3], accADC[3], magADC[3];	 //保存陀螺仪和加速度校准后的原始值
static int16_t gyroZero[3] = { 0, 0, 0 };//校准值

/* function prototypes -----------------------------------------------*/
static void Mag_GetADC(void);
static void Mag_Init(void);
static void Baro_Update(void);

/* OS -----------------------------------------------*/
OS_MUTEX Mutex_I2C2Using;//i2c2占用的互斥

static  void  Sensor_MainTask (void *p_arg);
static  OS_TCB  Sensor_MainTaskTCB;
static  CPU_STK  Sensor_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];



/**
  * @brief  传感器的检测和初始化
  *         最低要求要检测到陀螺仪，否则进入错误模式
  * @param  none
  * @retval none
  */
void Sensor_DetectAndInit(void)
{
	OS_ERR err;
	int16_t deg, min;
	
	OSMutexCreate(&Mutex_I2C2Using,//I2C2互斥
							"I2CUsing",
							&err);
	
	SensorsSet(SENSOR_ACC | SENSOR_BARO | SENSOR_MAG );//标记存在这些传感器		
	
	if(!MPU6050_Detect(&acc,&gyro,cfg.mpu6050_scale))//检测是否存在MPU6050
		System_FailureMode(0);		//进入错误模式，即只要不存在MPU6050，就不让飞行
	gyro.init();//绝对要存在陀螺仪，否则来不到这里了
	CalibratingG=1000;//校准陀螺仪
  acc.init(); //ACC初始化

// #ifdef BMP085
// 	if(!bmp085Detect(&baro))	//检测是否存在气压计
// 	{
// 		sensorsClear(SENSOR_BARO);//如果没有找到，标记不使用气压	
// 	}
// #endif
#ifdef MS5611
	if(!MS5611_Detect(&baro))	//检测是否存在气压计
	{
		SensorsClear(SENSOR_BARO);//如果没有找到，标记不使用气压	
	}
#endif

	if (!HMC5883_Detect(&mag))		//判断是否存在磁阻5883
		SensorsClear(SENSOR_MAG);//不存在则清除其标记
	//校准磁偏角
	deg = cfg.mag_declination / 100;
	min = cfg.mag_declination % 100;
	MagneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
	Mag_Init();

	OSTaskCreate((OS_TCB   *)&Sensor_MainTaskTCB,  
					 (CPU_CHAR     *)"Sensor_MainTask",
					 (OS_TASK_PTR   )Sensor_MainTask, 
					 (void         *)0,
					 (OS_PRIO       )PRIO_SENSOR ,
					 (CPU_STK      *)&Sensor_MainTaskStk[0],
					 (CPU_STK_SIZE  )Sensor_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
					 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
					 (OS_MSG_QTY    )0,
					 (OS_TICK       )0,
					 (void         *)0,
					 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
					 (OS_ERR       *)&err);
}

/**
  * @brief  传感器数据读取任务
  *         磁阻、气压计
  * @param  
  * @retval 
  */
static  void  Sensor_MainTask (void *p_arg)
{
	OS_ERR	err;
	CPU_TS ts;
	static u8 mag_count=0;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(10,OS_OPT_TIME_PERIODIC,&err);//周期延时
		OSMutexPend(&Mutex_I2C2Using,					 //占用
								0,
								OS_OPT_PEND_BLOCKING,
								&ts,
								&err);
		if(mag_count++>10)
		{
			mag_count=0;
			Mag_GetADC();		//磁阻每100Ms读一次
		}
		else
		{
			Baro_Update();  //气压计每10ms调用一次
		}
		OSMutexPost(&Mutex_I2C2Using,					 //释放
								OS_OPT_POST_NONE,
								&err);
  }
}

/*
*********************************************************************************************************
*                                       陀螺仪GYRO
*********************************************************************************************************
*/
/**
  * @brief  修正函数，校准陀螺仪
  *         
  * @param  None
  * @retval none
  */
static void GYRO_Common(void)
{
	static int16_t previousGyroADC[3] = { 0, 0, 0 };
	static int32_t g[3];
	int axis;

	if (CalibratingG > 0) {
		for (axis = 0; axis < 3; axis++) 
		{
			// Reset g[axis] at start of calibration
			if (CalibratingG == 1000)
				g[axis] = 0;
			// Sum up 1000 readings
			g[axis] += gyroADC[axis];
			// g[axis] += (1000 - CalibratingG) >> 1;
			// Clear global variables for next reading
			gyroADC[axis] = 0;
			gyroZero[axis] = 0;
			if (CalibratingG == 1) {
				gyroZero[axis] = g[axis] / 1000;
			}
		}
		CalibratingG--;
	}
	for (axis = 0; axis < 3; axis++) 
	{
		gyroADC[axis] -= gyroZero[axis];
		//anti gyro glitch, limit the variation between two consecutive readings
		gyroADC[axis] = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
		previousGyroADC[axis] = gyroADC[axis];
	}
}

/**
  * @brief  获取陀螺仪修正值后的原始值
  *         
  * @param  none
  * @retval none
  */
void Gyro_GetADC(void)
{
	OS_ERR err;
	CPU_TS ts;
	OSMutexPend(&Mutex_I2C2Using,					 //占用
						0,
						OS_OPT_PEND_BLOCKING,
						&ts,
						&err);
	// range: +/- 8192; +/- 2000 deg/sec
	gyro.read(gyroADC);
	OSMutexPost(&Mutex_I2C2Using,					 //释放
						OS_OPT_POST_NONE,
						&err);
	gyro.align(gyroADC);

	GYRO_Common();
}

/*
*********************************************************************************************************
*                                   加速度ACC
*********************************************************************************************************
*/
/**
  * @brief  加速度计校准函数，用于计算0点值和利用0点值修正
  *         校准值将保存到FLASH中，以便下次使用
  * @param  none
  * @retval none
  */
static void Acc_Common(void)
{
	static int32_t a[3];//用来保存400个数据的和
	int axis;

	if (CalibratingA > 0)
	{
		for (axis = 0; axis < 3; axis++)
		{	//0,1,2分别代表X,Y,Z轴
			// Reset a[axis] at start of calibration
			if (CalibratingA == 400)	//当CalibratingA被置为400时，进行加速度校准
				a[axis] = 0;
			// 总计400个读数
			a[axis] += accADC[axis];
			// Clear global variables for next reading
			accADC[axis] = 0;
			cfg.accZero[axis] = 0;
		}
		// 使用平均数校准，Z轴要减去重力加速度1g，然后保存所有数据
		if (CalibratingA == 1)
		{	//当为1的时候进行处理了
			cfg.accZero[ROLL] = a[ROLL] / 400;	//求400次的平均
			cfg.accZero[PITCH] = a[PITCH] / 400;
			cfg.accZero[YAW] = a[YAW] / 400 - acc_1G;       //256=1G
			cfg.angleTrim[ROLL] = 0;							  //可能是微调
			cfg.angleTrim[PITCH] = 0;
			WRITE_TO_FLASH;//将参数写入FLASH
		}
		CalibratingA--;
	}
	accADC[ROLL] -= cfg.accZero[ROLL];		//将三个值进行校准,也就是减去0点值
	accADC[PITCH] -= cfg.accZero[PITCH];
	accADC[YAW] -= cfg.accZero[YAW];
}

/**
  * @brief  获取加速度的修正后的值
  *         
  * @param  none
  * @retval None
  */
void Acc_GetADC(void)
{
	OS_ERR err;
	CPU_TS ts;
	OSMutexPend(&Mutex_I2C2Using,					 //占用
						0,
						OS_OPT_PEND_BLOCKING,
						&ts,
						&err);
	acc.read(accADC);	//读取到的数据保存在数组accADC中
	OSMutexPost(&Mutex_I2C2Using,					 //释放
						OS_OPT_POST_NONE,
						&err);

	acc.align(accADC);

	Acc_Common();		//校准获取0点值，利用0点值修正
}

/*
*********************************************************************************************************
*                                 电子罗盘MAG
*********************************************************************************************************
*/
static float magCal[3] = { 1.0, 1.0, 1.0 };     // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

/**
  * @brief  读取原始数据
  *         
  * @param  none
  * @retval none
  */
static void Mag_GetRawADC(void)
{
	static int16_t rawADC[3];
	mag.read(rawADC);
	/*下面要根据实际情况调整*/
	magADC[ROLL] = -rawADC[2]; // X
	magADC[PITCH] = rawADC[0]; // Y
	magADC[YAW] = -rawADC[1]; // Z
}

/**
  * @brief  磁阻初始化，并获取校准值
  *         
  * @param  none
  * @retval none
  */
static void Mag_Init(void)
{
	OS_ERR  err;
	u8 calibration_gain = 0x60;	//HMC5883
	u32 numAttempts = 0,good_count = 0;
	bool success = false;
	u16 expected_x = 766;	//HMC5883默认值
	u16 expected_yz = 713;
	float gain_multiple = 660.0f / 1090.0f;// adjustment for runtime vs calibration gain
	float cal[3];
	
	mag.init();//初始化5883传感器	

	magCal[0]=0;
	magCal[1]=0;
	magCal[2]=0;

	while(success == false && numAttempts <20 && good_count <5)
	{
		numAttempts++;//记录尝试进行初始化的次数
		HMC5883_Cal(calibration_gain);//进入校准模式
		OSTimeDly(100,OS_OPT_TIME_DLY,&err);
		Mag_GetRawADC();//读取原始数据
		OSTimeDly(10,OS_OPT_TIME_DLY,&err);

		cal[0] = fabsf(expected_x / (float)magADC[ROLL]);
		cal[1] = fabsf(expected_yz / (float)magADC[PITCH]);
		cal[2] = fabsf(expected_yz / (float)magADC[ROLL]);

		if (cal[0] > 0.7f && cal[0] < 1.3f && cal[1] > 0.7f && cal[1] < 1.3f && cal[2] > 0.7f && cal[2] < 1.3f) 
		{
			good_count++;
			magCal[0] += cal[0];
			magCal[1] += cal[1];
			magCal[2] += cal[2];
    }
	}
	if(good_count >= 5)	   //自动校准成功
	{
		magCal[0] = magCal[0] * gain_multiple / (float)good_count;
		magCal[1] = magCal[1] * gain_multiple / (float)good_count;
		magCal[2] = magCal[2] * gain_multiple / (float)good_count;
		success = true;
	}
	else				   //无法进行自动校准，则使用猜测经验值
	{
		magCal[0] = 1.0f;
		magCal[1] = 1.0f;
		magCal[2] = 1.0f;	
	}
	HMC5883_FinishCal();//结束校准
	magInit = 1;
}

/**
  * @brief  磁阻数据获取
  *         每次调用间隔不能少于100Ms
  * @param  None
  * @retval none
  */
static void Mag_GetADC(void)
{
	static uint32_t tCal = 0;
	static int16_t magZeroTempMin[3];
	static int16_t magZeroTempMax[3];
	uint32_t axis;
	
	// Read mag sensor
	Mag_GetRawADC();

	magADC[ROLL]  = magADC[ROLL]  * magCal[ROLL];
	magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
	magADC[YAW]   = magADC[YAW]   * magCal[YAW];

	if (f.CALIBRATE_MAG) //进入磁阻校准
	{
		LEDFLASH_SET(LEDFLASH_MAGCAL);//设置指示灯闪烁
		tCal = 1;
		for (axis = 0; axis < 3; axis++) 
		{
			cfg.magZero[axis] = 0;
			magZeroTempMin[axis] = magADC[axis];
			magZeroTempMax[axis] = magADC[axis];
		}
		f.CALIBRATE_MAG = 0;
	}

	if (magInit) 
	{              // we apply offset only once mag calibration is done
			magADC[ROLL] -= cfg.magZero[ROLL];
			magADC[PITCH] -= cfg.magZero[PITCH];
			magADC[YAW] -= cfg.magZero[YAW];
	}

	if (tCal != 0) 
	{
		if (tCal++ < 300) // 30s: 你有30秒时间，将IMU全角度旋转，让每个方向达到最大
		{
			for (axis = 0; axis < 3; axis++) 
			{
				if (magADC[axis] < magZeroTempMin[axis])
						magZeroTempMin[axis] = magADC[axis];
				if (magADC[axis] > magZeroTempMax[axis])
						magZeroTempMax[axis] = magADC[axis];
			}
		}
		else
		{
			tCal = 0;
			for (axis = 0; axis < 3; axis++)
				cfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2; // Calculate offsets
			LEDFLASH_CLR(LEDFLASH_MAGCAL);//清除闪烁
			WRITE_TO_FLASH;//将参数写入FLASH
		}
	}
}

/*
*********************************************************************************************************
*                                   气压计BARO
*********************************************************************************************************
*/
#ifdef BARO
/**
  * @brief  读取气压计温度和压力
  *         气压计需要转换时间，转换时间接近10Ms,所以调用此函数时间间隔不少于10ms
  * @param  
  * @retval 
  */
static void Baro_Update(void)
{
	static uint8_t state = 0;
	int32_t pressure;
	switch (state) {
			case 0:
					baro.start_ut();	   //开始温度转换,10ms
					state++;
					break;
			case 1:
					baro.get_ut();		   //读取温度
					state++;
					break;
			case 2:
					baro.start_up();	   //开始压力转换10ms
					state++;
					break;
			case 3:
					baro.get_up();		   //得到压力值
					pressure = baro.calculate();   //得到通过温度等校准后的气压值，单位为pa
					Baro_Common(pressure);		   //新加,进行滑动平均滤波
					//BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f; // centimeter	   移动到IMU里的得到高度那里去了
					state = 0;
					break;
	}
}
#endif /* BARO */
