#include "bsp.h"

/**
  * @brief  �弶��ʼ��
  *         
  * @param  none
  * @retval none
  */
void  BSP_Init (void)
{
	LED_Init();
	Flash_Read();									//��ȡ����
	Flash_CheckFirstTime(false);	//���FLASH���Ƿ������Ч���ݣ����û����д��Ĭ������
//	cfg.mixerConfiguration = MIXERCONFIG;ͳһ�ڵ�����������û���
	Mixer_Init();									//������ʼ��
	Serial_Init(cfg.serial_baudrate);//��λ�����γ�ʼ��
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

void CPU_TS_TmrInit(void)	   	 //uCOS�ڲ���Ҫ���ã�����
{
}

CPU_TS_TMR CPU_TS_TmrRd(void)			 //uCOS�ڲ���Ҫ���ã�����
{
	return 0;
}
