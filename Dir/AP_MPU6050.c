/**
	******************************************************************************
	* @file    AP_MPU6050.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   
	******************************************************************************
**/
#include "AP_MPU6050.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
// MPU6050, Standard address 0x68
#ifdef IMU_NEWMODULE
#define MPU6050_ADDRESS         (0x68<<1)	  //根据硬件接线确定地址是68还是69	 老的IMU模块是0X69,新的是0X68
#else //IMU_OLDMODULE
#define MPU6050_ADDRESS         (0x69<<1)
#endif

// Experimental DMP support
// #define MPU6050_DMP

#define DMP_MEM_START_ADDR 0x6E
#define DMP_MEM_R_W 0x6F

#define INV_MAX_NUM_ACCEL_SAMPLES      (8)
#define DMP_REF_QUATERNION             (0)
#define DMP_REF_GYROS                  (DMP_REF_QUATERNION + 4) // 4
#define DMP_REF_CONTROL                (DMP_REF_GYROS + 3)      // 7
#define DMP_REF_RAW                    (DMP_REF_CONTROL + 4)    // 11
#define DMP_REF_RAW_EXTERNAL           (DMP_REF_RAW + 8)        // 19
#define DMP_REF_ACCEL                  (DMP_REF_RAW_EXTERNAL + 6)       // 25
#define DMP_REF_QUANT_ACCEL            (DMP_REF_ACCEL + 3)      // 28
#define DMP_REF_QUATERNION_6AXIS       (DMP_REF_QUANT_ACCEL + INV_MAX_NUM_ACCEL_SAMPLES)        // 36
#define DMP_REF_EIS                    (DMP_REF_QUATERNION_6AXIS + 4)   // 40
#define DMP_REF_DMP_PACKET             (DMP_REF_EIS + 3)        // 43
#define DMP_REF_GARBAGE                (DMP_REF_DMP_PACKET + 1) // 44
#define DMP_REF_LAST                   (DMP_REF_GARBAGE + 1)    // 45

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU6050_SMPLRT_DIV      0       //8000Hz
// #define MPU6050_DLPF_CFG        0   // 256Hz
#define MPU6050_DLPF_CFG   3        // 42Hz

#define MPU6000ES_REV_C4        0x14
#define MPU6000ES_REV_C5        0x15
#define MPU6000ES_REV_D6        0x16
#define MPU6000ES_REV_D7        0x17
#define MPU6000ES_REV_D8        0x18
#define MPU6000_REV_C4          0x54
#define MPU6000_REV_C5          0x55
#define MPU6000_REV_D6          0x56
#define MPU6000_REV_D7          0x57
#define MPU6000_REV_D8          0x58
#define MPU6000_REV_D9          0x59

/* variables ---------------------------------------------------------*/
uint8_t mpuProductID = 0;

/* function prototypes -----------------------------------------------*/
static void MPU6050_AccInit(void);
static void MPU6050_AccRead(int16_t * accData);
static void MPU6050_AccAlign(int16_t * accData);
static void MPU6050_GyroInit(void);
static void MPU6050_GyroRead(int16_t * gyroData);
static void MPU6050_GyroAlign(int16_t * gyroData);

/**
  * @brief  MPU6050检测，检测其是否存在
  *         
  * @param  *acc:加速度传感器结构体
  * @param  *gyro:陀螺仪结构体
	* @param	scale:获取版本
  * @retval false:不存在，true:存在
  */
bool MPU6050_Detect(sensor_t * acc, sensor_t * gyro, uint8_t scale)
{
	bool ack;
	uint8_t sig;
	OS_ERR  err;

	OSTimeDly(35,OS_OPT_TIME_DLY,&err);// datasheet page 13 says 30ms. other stuff could have been running meanwhile. but we'll be safe
       
	ack = I2C2_TiReadReg_Buf(MPU6050_ADDRESS, MPU_RA_WHO_AM_I, 1, &sig);
	if (!ack)
			return false;

	if (sig != 0x68)
			return false;

	// get chip revision + fake it if needed
	if (scale)
			mpuProductID = MPU6000_REV_C5; // no, seriously? why don't you make the chip ID list public.
	else
			I2C2_TiReadReg_Buf(MPU6050_ADDRESS, MPU_RA_PRODUCT_ID, 1, &mpuProductID);

	acc->init = MPU6050_AccInit;			//初始化函数
	acc->read = MPU6050_AccRead;			//...so on
	acc->align = MPU6050_AccAlign;
	gyro->init = MPU6050_GyroInit;
	gyro->read = MPU6050_GyroRead;
	gyro->align = MPU6050_GyroAlign;

	return true;
}

/**
  * @brief  加速度计初始化
  * 
  * @param  None
  * @retval none
  */
static void MPU6050_AccInit(void)
{
    acc_1G = 256;//1023;重力加速度
}

/**
  * @brief  加速度计读取
  *         
  * @param  *accData：储存数据的数组
  * @retval none
  */
static void MPU6050_AccRead(int16_t * accData)
{
	uint8_t buf[6];

	if(I2C2_TiReadReg_Buf(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H, 6, buf))
	{
		accData[0] = (buf[0] << 8) | buf[1];
		accData[1] = (buf[2] << 8) | buf[3];
		accData[2] = (buf[4] << 8) | buf[5];
	}
}

/**
  * @brief  加速度计原始数据处理
  *         
  * @param  数据数组
  * @retval None
  */
static void MPU6050_AccAlign(int16_t * accData)
{
    int16_t temp[2];
    temp[0] = accData[0];
    temp[1] = accData[1];

    // official direction is RPY
    accData[0] = temp[1] / 32;
    accData[1] = -temp[0] / 32;
    accData[2] = accData[2] / 32;
}

/**
  * @brief  陀螺仪初始化
  *         
  * @param  none
  * @retval none
  */
static void MPU6050_GyroInit(void)
{
	OS_ERR  err;
	I2C2_WriteReg(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
	OSTimeDly(5,OS_OPT_TIME_DLY,&err);
	I2C2_WriteReg(MPU6050_ADDRESS, MPU_RA_SMPLRT_DIV, 0x00);      //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	I2C2_WriteReg(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	I2C2_WriteReg(MPU6050_ADDRESS, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
	I2C2_WriteReg(MPU6050_ADDRESS, MPU_RA_CONFIG, MPU6050_DLPF_CFG);  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	I2C2_WriteReg(MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, 0x18);      //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

	// ACC Init stuff. Moved into gyro init because the reset above would screw up accel config. Oops.
	// Product ID detection code from eosBandi (or rather, DIYClones). This doesn't cover product ID for MPU6050 as far as I can tell
	if ((mpuProductID == MPU6000ES_REV_C4) || (mpuProductID == MPU6000ES_REV_C5) || (mpuProductID == MPU6000_REV_C4) || (mpuProductID == MPU6000_REV_C5)) {
			// Accel scale 8g (4096 LSB/g)
			// Rev C has different scaling than rev D
			I2C2_WriteReg(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, 1 << 3);
	} else {
			I2C2_WriteReg(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, 2 << 3);
	}
}

/**
  * @brief  陀螺仪读取
  *         
  * @param  数据数组
  * @retval None
  */
static void MPU6050_GyroRead(int16_t * gyroData)
{
	uint8_t buf[6];
	if(I2C2_TiReadReg_Buf(MPU6050_ADDRESS, MPU_RA_GYRO_XOUT_H, 6, buf))
	{
		gyroData[0] = (buf[0] << 8) | buf[1];
		gyroData[1] = (buf[2] << 8) | buf[3];
		gyroData[2] = (buf[4] << 8) | buf[5];
	}
}

/**
  * @brief  陀螺仪数据处理
  *         
  * @param  数据数组
  * @retval None
  */
static void MPU6050_GyroAlign(int16_t * gyroData)
{
    // official direction is RPY
    gyroData[0] = gyroData[0] / 4;
    gyroData[1] = gyroData[1] / 4;
    gyroData[2] = -gyroData[2] / 4;
}
