#include "bsp.h"

/**
  * @brief  板级初始化
  *         
  * @param  none
  * @retval none
  */
void  BSP_Init (void)
{
	LED_Init();
	Flash_Read();									//读取参数
	Flash_CheckFirstTime(false);	//检查FLASH里是否存在有效数据，如果没有则写入默认数据
//	cfg.mixerConfiguration = MIXERCONFIG;统一在调参软件里设置机型
	Mixer_Init();									//动力初始化
	Serial_Init(cfg.serial_baudrate);//上位机调参初始化
	I2C2_Init();											//I2C2
	Sensor_DetectAndInit();
	IMU_Init();
	Altitude_Init();
	Actuator_Init();
	RC_Init();
	Daemon_Init();
	GPS_Init(cfg.gps_baudrate);
	Telemetry_Init();
#ifdef OPTICALFLOW
	OpticalFlow_Init();
#endif
	EasyCtrl_Init();
	VisionLanding_Init();
	Avoid_Init();
}

void CPU_TS_TmrInit(void)	   	 //uCOS内部需要调用，保留
{
}

CPU_TS_TMR CPU_TS_TmrRd(void)			 //uCOS内部需要调用，保留
{
	return 0;
}
