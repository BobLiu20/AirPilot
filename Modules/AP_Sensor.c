/**
	******************************************************************************
	* @file    AP_Sensor.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   ���ִ������������⡢��ʼ������ȡ��
	*					 ��ʹ��Ӳ��I2Cʱ��I2C������300k���ٶ�ʱ����ȡһ����Ҫ290us
	*																				400k���ٶ�ʱ����ȡһ����Ҫ230us
	*					 ע��i2c����ٶ�Ϊ400k
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
float MagneticDeclination = 0.0f; //��ƫ��У׼ֵ
uint16_t acc_1G=256;

uint16_t CalibratingA = 0;       //����м��ٶ�У׼ʱ���뽫CalibratingA��ֵΪ400
uint16_t CalibratingG = 0;			 //У׼��ֵ1000

int16_t gyroADC[3], accADC[3], magADC[3];	 //���������Ǻͼ��ٶ�У׼���ԭʼֵ
static int16_t gyroZero[3] = { 0, 0, 0 };//У׼ֵ

/* function prototypes -----------------------------------------------*/
static void Mag_GetADC(void);
static void Mag_Init(void);
static void Baro_Update(void);

/* OS -----------------------------------------------*/
OS_MUTEX Mutex_I2C2Using;//i2c2ռ�õĻ���

static  void  Sensor_MainTask (void *p_arg);
static  OS_TCB  Sensor_MainTaskTCB;
static  CPU_STK  Sensor_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];



/**
  * @brief  �������ļ��ͳ�ʼ��
  *         ���Ҫ��Ҫ��⵽�����ǣ�����������ģʽ
  * @param  none
  * @retval none
  */
void Sensor_DetectAndInit(void)
{
	OS_ERR err;
	int16_t deg, min;
	
	OSMutexCreate(&Mutex_I2C2Using,//I2C2����
							"I2CUsing",
							&err);
	
	SensorsSet(SENSOR_ACC | SENSOR_BARO | SENSOR_MAG );//��Ǵ�����Щ������		
	
	if(!MPU6050_Detect(&acc,&gyro,cfg.mpu6050_scale))//����Ƿ����MPU6050
		System_FailureMode(0);		//�������ģʽ����ֻҪ������MPU6050���Ͳ��÷���
	gyro.init();//����Ҫ���������ǣ�����������������
	CalibratingG=1000;//У׼������
  acc.init(); //ACC��ʼ��

// #ifdef BMP085
// 	if(!bmp085Detect(&baro))	//����Ƿ������ѹ��
// 	{
// 		sensorsClear(SENSOR_BARO);//���û���ҵ�����ǲ�ʹ����ѹ	
// 	}
// #endif
#ifdef MS5611
	if(!MS5611_Detect(&baro))	//����Ƿ������ѹ��
	{
		SensorsClear(SENSOR_BARO);//���û���ҵ�����ǲ�ʹ����ѹ	
	}
#endif

	if (!HMC5883_Detect(&mag))		//�ж��Ƿ���ڴ���5883
		SensorsClear(SENSOR_MAG);//���������������
	//У׼��ƫ��
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
  * @brief  ���������ݶ�ȡ����
  *         ���衢��ѹ��
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
		OSTimeDly(10,OS_OPT_TIME_PERIODIC,&err);//������ʱ
		OSMutexPend(&Mutex_I2C2Using,					 //ռ��
								0,
								OS_OPT_PEND_BLOCKING,
								&ts,
								&err);
		if(mag_count++>10)
		{
			mag_count=0;
			Mag_GetADC();		//����ÿ100Ms��һ��
		}
		else
		{
			Baro_Update();  //��ѹ��ÿ10ms����һ��
		}
		OSMutexPost(&Mutex_I2C2Using,					 //�ͷ�
								OS_OPT_POST_NONE,
								&err);
  }
}

/*
*********************************************************************************************************
*                                       ������GYRO
*********************************************************************************************************
*/
/**
  * @brief  ����������У׼������
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
  * @brief  ��ȡ����������ֵ���ԭʼֵ
  *         
  * @param  none
  * @retval none
  */
void Gyro_GetADC(void)
{
	OS_ERR err;
	CPU_TS ts;
	OSMutexPend(&Mutex_I2C2Using,					 //ռ��
						0,
						OS_OPT_PEND_BLOCKING,
						&ts,
						&err);
	// range: +/- 8192; +/- 2000 deg/sec
	gyro.read(gyroADC);
	OSMutexPost(&Mutex_I2C2Using,					 //�ͷ�
						OS_OPT_POST_NONE,
						&err);
	gyro.align(gyroADC);

	GYRO_Common();
}

/*
*********************************************************************************************************
*                                   ���ٶ�ACC
*********************************************************************************************************
*/
/**
  * @brief  ���ٶȼ�У׼���������ڼ���0��ֵ������0��ֵ����
  *         У׼ֵ�����浽FLASH�У��Ա��´�ʹ��
  * @param  none
  * @retval none
  */
static void Acc_Common(void)
{
	static int32_t a[3];//��������400�����ݵĺ�
	int axis;

	if (CalibratingA > 0)
	{
		for (axis = 0; axis < 3; axis++)
		{	//0,1,2�ֱ����X,Y,Z��
			// Reset a[axis] at start of calibration
			if (CalibratingA == 400)	//��CalibratingA����Ϊ400ʱ�����м��ٶ�У׼
				a[axis] = 0;
			// �ܼ�400������
			a[axis] += accADC[axis];
			// Clear global variables for next reading
			accADC[axis] = 0;
			cfg.accZero[axis] = 0;
		}
		// ʹ��ƽ����У׼��Z��Ҫ��ȥ�������ٶ�1g��Ȼ�󱣴���������
		if (CalibratingA == 1)
		{	//��Ϊ1��ʱ����д�����
			cfg.accZero[ROLL] = a[ROLL] / 400;	//��400�ε�ƽ��
			cfg.accZero[PITCH] = a[PITCH] / 400;
			cfg.accZero[YAW] = a[YAW] / 400 - acc_1G;       //256=1G
			cfg.angleTrim[ROLL] = 0;							  //������΢��
			cfg.angleTrim[PITCH] = 0;
			WRITE_TO_FLASH;//������д��FLASH
		}
		CalibratingA--;
	}
	accADC[ROLL] -= cfg.accZero[ROLL];		//������ֵ����У׼,Ҳ���Ǽ�ȥ0��ֵ
	accADC[PITCH] -= cfg.accZero[PITCH];
	accADC[YAW] -= cfg.accZero[YAW];
}

/**
  * @brief  ��ȡ���ٶȵ��������ֵ
  *         
  * @param  none
  * @retval None
  */
void Acc_GetADC(void)
{
	OS_ERR err;
	CPU_TS ts;
	OSMutexPend(&Mutex_I2C2Using,					 //ռ��
						0,
						OS_OPT_PEND_BLOCKING,
						&ts,
						&err);
	acc.read(accADC);	//��ȡ�������ݱ���������accADC��
	OSMutexPost(&Mutex_I2C2Using,					 //�ͷ�
						OS_OPT_POST_NONE,
						&err);

	acc.align(accADC);

	Acc_Common();		//У׼��ȡ0��ֵ������0��ֵ����
}

/*
*********************************************************************************************************
*                                 ��������MAG
*********************************************************************************************************
*/
static float magCal[3] = { 1.0, 1.0, 1.0 };     // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

/**
  * @brief  ��ȡԭʼ����
  *         
  * @param  none
  * @retval none
  */
static void Mag_GetRawADC(void)
{
	static int16_t rawADC[3];
	mag.read(rawADC);
	/*����Ҫ����ʵ���������*/
	magADC[ROLL] = -rawADC[2]; // X
	magADC[PITCH] = rawADC[0]; // Y
	magADC[YAW] = -rawADC[1]; // Z
}

/**
  * @brief  �����ʼ��������ȡУ׼ֵ
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
	u16 expected_x = 766;	//HMC5883Ĭ��ֵ
	u16 expected_yz = 713;
	float gain_multiple = 660.0f / 1090.0f;// adjustment for runtime vs calibration gain
	float cal[3];
	
	mag.init();//��ʼ��5883������	

	magCal[0]=0;
	magCal[1]=0;
	magCal[2]=0;

	while(success == false && numAttempts <20 && good_count <5)
	{
		numAttempts++;//��¼���Խ��г�ʼ���Ĵ���
		HMC5883_Cal(calibration_gain);//����У׼ģʽ
		OSTimeDly(100,OS_OPT_TIME_DLY,&err);
		Mag_GetRawADC();//��ȡԭʼ����
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
	if(good_count >= 5)	   //�Զ�У׼�ɹ�
	{
		magCal[0] = magCal[0] * gain_multiple / (float)good_count;
		magCal[1] = magCal[1] * gain_multiple / (float)good_count;
		magCal[2] = magCal[2] * gain_multiple / (float)good_count;
		success = true;
	}
	else				   //�޷������Զ�У׼����ʹ�ò²⾭��ֵ
	{
		magCal[0] = 1.0f;
		magCal[1] = 1.0f;
		magCal[2] = 1.0f;	
	}
	HMC5883_FinishCal();//����У׼
	magInit = 1;
}

/**
  * @brief  �������ݻ�ȡ
  *         ÿ�ε��ü����������100Ms
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

	if (f.CALIBRATE_MAG) //�������У׼
	{
		LEDFLASH_SET(LEDFLASH_MAGCAL);//����ָʾ����˸
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
		if (tCal++ < 300) // 30s: ����30��ʱ�䣬��IMUȫ�Ƕ���ת����ÿ������ﵽ���
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
			LEDFLASH_CLR(LEDFLASH_MAGCAL);//�����˸
			WRITE_TO_FLASH;//������д��FLASH
		}
	}
}

/*
*********************************************************************************************************
*                                   ��ѹ��BARO
*********************************************************************************************************
*/
#ifdef BARO
/**
  * @brief  ��ȡ��ѹ���¶Ⱥ�ѹ��
  *         ��ѹ����Ҫת��ʱ�䣬ת��ʱ��ӽ�10Ms,���Ե��ô˺���ʱ����������10ms
  * @param  
  * @retval 
  */
static void Baro_Update(void)
{
	static uint8_t state = 0;
	int32_t pressure;
	switch (state) {
			case 0:
					baro.start_ut();	   //��ʼ�¶�ת��,10ms
					state++;
					break;
			case 1:
					baro.get_ut();		   //��ȡ�¶�
					state++;
					break;
			case 2:
					baro.start_up();	   //��ʼѹ��ת��10ms
					state++;
					break;
			case 3:
					baro.get_up();		   //�õ�ѹ��ֵ
					pressure = baro.calculate();   //�õ�ͨ���¶ȵ�У׼�����ѹֵ����λΪpa
					Baro_Common(pressure);		   //�¼�,���л���ƽ���˲�
					//BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f; // centimeter	   �ƶ���IMU��ĵõ��߶�����ȥ��
					state = 0;
					break;
	}
}
#endif /* BARO */
