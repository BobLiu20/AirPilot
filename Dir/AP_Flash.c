/**
	******************************************************************************
	* @file    AP_Flash.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7.23
	* @brief   STM32F407VGT6µÄFLASHÎª1M
	*					·ÖÎªSector 1ÖÁSector 11
	*					Ê¹ÓÃSector 11×÷Îª²ÎÊý´¢´æÇø
	*					Sector 11µÄµØÖ·Îª0x080E 0000 - 0x080F FFFF£¬´óÐ¡Îª128K
	*
	******************************************************************************
**/
#include "AP_Flash.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
#define FLASH_WRITE_ADDR  0x080E0000       //Ê¹ÓÃflashµÄ×îºóÒ»¿é´¢´æÕâÐ©²ÎÊý
#define FLASH_WRITE_SECTOR FLASH_Sector_11

#define CHECKNEWCONF 26		//ÓÃÀ´±ê¼Ç¼ì²âÊÇ·ñ´æÔÚ²ÎÊýÊý¾Ý

/* variables ---------------------------------------------------------*/
int16_t LookupPitchRollRC[6];   // lookup table for expo & RC rate PITCH+ROLL
int16_t LookupThrottleRC[11];   // lookup table for expo & mid THROTTLE

config_t cfg;							//ÅäÖÃ²ÎÊýÊý¾Ý
const char RcChannelLetters[] = "AERT1234";

static uint32_t EnabledSensors = 0;//ÓÃÀ´±ê¼Ç´«¸ÐÆ÷ÊÇ·ñ´æÔÚµÄ±ê¼ÇÎ»

/* function prototypes -----------------------------------------------*/

/**
  * @brief  ½âÎöÒ£¿ØÍ¨µÀ
  *         
  * @param  
  * @retval none
  */
static void ParseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++) {
        s = strchr(RcChannelLetters, *c);
        if (s)
            cfg.rcmap[s - RcChannelLetters] = c - input;
    }
}

/**
  * @brief  ¶ÁÈ¡FLASHÖÐµÄ²ÎÊýÊý¾Ý
  *         
  * @param  none
  * @retval none
  */
void Flash_Read(void)
{
    uint8_t i;

    // Read flash
    memcpy(&cfg, (char *) FLASH_WRITE_ADDR, sizeof(config_t));

    for (i = 0; i < 6; i++)
        LookupPitchRollRC[i] = (2500 + cfg.rcExpo8 * (i * i - 25)) * i * (int32_t) cfg.rcRate8 / 2500;

    for (i = 0; i < 11; i++) {
        int16_t tmp = 10 * i - cfg.thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - cfg.thrMid8;
        if (tmp < 0)
            y = cfg.thrMid8;
        LookupThrottleRC[i] = 10 * cfg.thrMid8 + tmp * (100 - cfg.thrExpo8 + (int32_t) cfg.thrExpo8 * (tmp * tmp) / (y * y)) / 10;      // [0;1000]
        LookupThrottleRC[i] = cfg.minthrottle + (int32_t) (cfg.maxthrottle - cfg.minthrottle) * LookupThrottleRC[i] / 1000;     // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
    }

    cfg.tri_yaw_middle = constrain(cfg.tri_yaw_middle, cfg.tri_yaw_min, cfg.tri_yaw_max);       //REAR
}

/**
  * @brief  ½«²ÎÊýÐ´ÈëFLASHÖÐ
  *         
  * @param  b:ÊÇ·ñÓÐLEDÌáÊ¾
  * @retval none
  */
void Flash_WriteParams(uint8_t b)
{
	FLASH_Status status;
	uint32_t i;

	FLASH_Unlock();

	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGPERR | FLASH_FLAG_WRPERR);

// 	//if (FLASH_ErasePage(FLASH_WRITE_ADDR) == FLASH_COMPLETE)
	if(FLASH_EraseSector(FLASH_WRITE_SECTOR, VoltageRange_3) == FLASH_COMPLETE)
	{
		for (i = 0; i < sizeof(config_t); i += 4) 
		{
			status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *) ((char *) &cfg + i));
			if (status != FLASH_COMPLETE)
			{
				System_FailureMode(1);
				break;          // TODO: fail
			}
		}
	}
	else
		System_FailureMode(1);

	FLASH_Lock();

	Flash_Read();
}

/**
  * @brief  ÓÃÓÚ¼ì²âÊÇ·ñµÚÒ»´ÎÊ¹ÓÃÐÂÐ¾Æ¬£¬Ò²¾ÍÊÇ²é¿´FLASHÀïÊÇ·ñ±£´æÓÐ²ÎÊý£¬Èç¹ûÃ»ÓÐÔòÐ´ÈëÄ¬ÈÏ²Î
  *         
  * @param  reset:Èç¹ûÎªfalseÎªÕý³£¼ì²â£¬ÎªtrueÔòÇ¿ÖÆÖØÐÂ²ÁÐ´
  * @retval none
  */
void Flash_CheckFirstTime(bool reset)
{
    uint8_t test_val, i;

    test_val = *(uint8_t *) FLASH_WRITE_ADDR;

    if (!reset && test_val == CHECKNEWCONF)
        return;

    // Default settings
    cfg.version = CHECKNEWCONF;
    cfg.mixerConfiguration = MULTITYPE_QUADX;
    FeatureClearAll();
    FeatureSet(FEATURE_MOTOR_STOP|FEATURE_FAILSAFE|FEATURE_GPS);

    cfg.looptime = 0;
    cfg.P8[ROLL] = 40;
    cfg.I8[ROLL] = 30;
    cfg.D8[ROLL] = 23;
    cfg.P8[PITCH] = 40;
    cfg.I8[PITCH] = 30;
    cfg.D8[PITCH] = 23;
    cfg.P8[YAW] = 85;
    cfg.I8[YAW] = 45;
    cfg.D8[YAW] = 0;
    cfg.P8[PIDALT] = 16;
    cfg.I8[PIDALT] = 15;
    cfg.D8[PIDALT] = 7;
    cfg.P8[PIDPOS] = 11; // POSHOLD_P * 100;
    cfg.I8[PIDPOS] = 0; // POSHOLD_I * 100;
    cfg.D8[PIDPOS] = 0;
    cfg.P8[PIDPOSR] = 20; // POSHOLD_RATE_P * 10;
    cfg.I8[PIDPOSR] = 8; // POSHOLD_RATE_I * 100;
    cfg.D8[PIDPOSR] = 45; // POSHOLD_RATE_D * 1000;
    cfg.P8[PIDNAVR] = 14; // NAV_P * 10;
    cfg.I8[PIDNAVR] = 20; // NAV_I * 100;
    cfg.D8[PIDNAVR] = 80; // NAV_D * 1000;
    cfg.P8[PIDLEVEL] = 70;
    cfg.I8[PIDLEVEL] = 10;
    cfg.D8[PIDLEVEL] = 20;
    cfg.P8[PIDMAG] = 40;
    cfg.P8[PIDVEL] = 0;
    cfg.I8[PIDVEL] = 0;
    cfg.D8[PIDVEL] = 0;
    cfg.rcRate8 = 90;
    cfg.rcExpo8 = 65;
    cfg.rollPitchRate = 0;
    cfg.yawRate = 0;
    cfg.dynThrPID = 0;
    cfg.thrMid8 = 50;
    cfg.thrExpo8 = 0;
    for (i = 0; i < CHECKBOXITEMS; i++)
        cfg.activate[i] = 0;
    cfg.angleTrim[0] = 0;
    cfg.angleTrim[1] = 0;
    cfg.accZero[0] = 0;
    cfg.accZero[1] = 0;
    cfg.accZero[2] = 0;
    cfg.mag_declination = -233;    // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero. ¹ãÖÝ´ÅÆ«½ÇÎª2¶È33·Ö
    cfg.acc_hardware = ACC_MPU6050;     //ÎÒÃÇÄ¬ÈÏÊÇÓÃMPU6050
    cfg.acc_lpf_factor = 100;
    cfg.gyro_cmpf_factor = 1500; // default MWC
    cfg.gyro_lpf = 42;
    cfg.mpu6050_scale = 1; // fuck invensense
    cfg.gyro_smoothing_factor = 0x00141403;     // default factors of 20, 20, 3 for R/P/Y
    cfg.vbatscale = 110;
    cfg.vbatmaxcellvoltage = 43;
    cfg.vbatmincellvoltage = 33;

    // Radio
    ParseRcChannels("AETR1234");
    cfg.deadband = 0;
    cfg.yawdeadband = 0;
    cfg.alt_hold_throttle_neutral = 20;
    cfg.spektrum_hires = 0;
    cfg.midrc = 1500;
    cfg.mincheck = 1100;
    cfg.maxcheck = 1900;
    cfg.retarded_arm = 0;       // disable arm/disarm on roll left/right

    // Failsafe Variables
    cfg.failsafe_delay = 200;    // 0.1sec
    cfg.failsafe_off_delay = 50;       // 20sec
    cfg.failsafe_throttle = 1200;       // decent default which should always be below hover throttle for people.

    // Motor/ESC/Servo
    cfg.minthrottle = 1150;
    cfg.maxthrottle = 2000;//1850
    cfg.mincommand = 1000;
    cfg.Vision_ErrorZero[0] = 0;
    cfg.Vision_ErrorZero[1] = 0;

    // servos
    cfg.yaw_direction = 1;
    cfg.tri_yaw_middle = 1500;
    cfg.tri_yaw_min = 1020;
    cfg.tri_yaw_max = 2000;

    // gimbal
    cfg.gimbal_pitch_gain = 10;
    cfg.gimbal_roll_gain = 10;
    cfg.gimbal_flags = GIMBAL_NORMAL;
    cfg.gimbal_pitch_min = 1020;
    cfg.gimbal_pitch_max = 2000;
    cfg.gimbal_pitch_mid = 1500;
    cfg.gimbal_roll_min = 1020;
    cfg.gimbal_roll_max = 2000;
    cfg.gimbal_roll_mid = 1500;

    // gps/nav stuff
    cfg.gps_baudrate = 115200;
    cfg.gps_wp_radius = 200;		 //µ¥Î»  cm
    cfg.gps_lpf = 20;
    cfg.nav_slew_rate = 30;
    cfg.nav_controls_heading = 1;
    cfg.nav_speed_min = 100;
    cfg.nav_speed_max = 300;		  //3.0m/s

    // serial(uart1) baudrate
    cfg.serial_baudrate = 115200;

    Flash_WriteParams(0);
}

/**
  * @brief  ´«¸ÐÆ÷´æÔÚ±ê¼Ç£¬EnabledSensorsÀï±£´æ×ÅÊÇ·ñ´æÔÚÓÐÕâÖÖ´«¸ÐÆ÷£
  *         Ã¿Ò»Î»¶ÔÓ¦²»Í¬µÄ´«¸ÐÆ÷
  * @param  ¼ì²âÄÄÒ»Î»
  * @retval ·µ»ØÊÇ·ñ´æÔÚ
  */
bool Sensors(uint32_t mask)
{
    return EnabledSensors & mask;
}

void SensorsSet(uint32_t mask)
{
    EnabledSensors |= mask;
}

void SensorsClear(uint32_t mask)
{
    EnabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return EnabledSensors;
}

bool Feature(uint32_t mask)
{
    return cfg.enabledFeatures & mask;
}

void FeatureSet(uint32_t mask)
{
    cfg.enabledFeatures |= mask;
}

void FeatureClear(uint32_t mask)
{
    cfg.enabledFeatures &= ~(mask);
}

void FeatureClearAll()
{
    cfg.enabledFeatures = 0;
}

uint32_t FeatureMask(void)
{
    return cfg.enabledFeatures;
}

