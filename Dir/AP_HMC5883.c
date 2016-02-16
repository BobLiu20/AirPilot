/**
	******************************************************************************
	* @file    AP_HMC5883.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   磁阻的驱动程序
	******************************************************************************
**/
#include "AP_HMC5883.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
// HMC5883L, default address 0x1E
#define MAG_ADDRESS (0x1E<<1)
#define MAG_DATA_REGISTER 0x03
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

/* variables ---------------------------------------------------------*/
/* function prototypes -----------------------------------------------*/
static void HMC5883_Init(void);
static void HMC5883_Read(int16_t *magData);

/**
  * @brief  陀螺仪检测函数
  *         
  * @param  none
  * @retval none
  */
bool HMC5883_Detect(sensor_t * mag)
{
	bool ack = false;
	uint8_t sig = 0;

	ack = I2C2_TiReadReg_Buf(MAG_ADDRESS, 0x0A, 1, &sig);
	if (!ack || sig != 'H')
			return false;
	mag->init = HMC5883_Init;			//初始化函数
	mag->read = HMC5883_Read;			//...so on

  return true;
}

/**
  * @brief  初始化
  *         
  * @param  None
  * @retval none
  */
static  void HMC5883_Init(void)
{
	OS_ERR err;
  OSTimeDly(100,OS_OPT_TIME_DLY,&err);
	I2C2_WriteReg(MAG_ADDRESS, ConfigRegA, SampleAveraging_8 << 5 | DataOutputRate_75HZ << 2 | NormalOperation);
	OSTimeDly(50,OS_OPT_TIME_DLY,&err);
}

void HMC5883_Cal(uint8_t calibration_gain)
{
	OS_ERR err;
	// force positiveBias (compass should return 715 for all channels)
	I2C2_WriteReg(MAG_ADDRESS, ConfigRegA, SampleAveraging_8 << 5 | DataOutputRate_75HZ << 2 | PositiveBiasConfig);
	OSTimeDly(50,OS_OPT_TIME_DLY,&err);
	// set gains for calibration
	I2C2_WriteReg(MAG_ADDRESS, ConfigRegB, calibration_gain);
	I2C2_WriteReg(MAG_ADDRESS, ModeRegister, SingleConversion);
}

void HMC5883_FinishCal(void)
{
    // leave test mode
    I2C2_WriteReg(MAG_ADDRESS, ConfigRegA, SampleAveraging_8 << 5 | DataOutputRate_75HZ << 2 | NormalOperation);
    I2C2_WriteReg(MAG_ADDRESS, ConfigRegB, magGain);
    I2C2_WriteReg(MAG_ADDRESS, ModeRegister, ContinuousConversion);
}

static void HMC5883_Read(int16_t *magData)
{
	uint8_t buf[6];
//	uint8_t err;
	if(I2C2_TiReadReg_Buf(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf))
	{
		magData[0] = buf[0] << 8 | buf[1];
		magData[1] = buf[2] << 8 | buf[3];
		magData[2] = buf[4] << 8 | buf[5];
	}
}

