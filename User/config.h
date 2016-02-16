#ifndef __CONFIG_H
#define __CONFIG_H
/*
*********************************************************************************************************
*                          �����Ƿ�ʹ����Щ��������ģ��,��ʹ�õ���ע��
*********************************************************************************************************
*/
#define GYRO
#define ACC
#define MAG
#define BARO
#define SONAR			   //��������
#define XBEEPRO			   //��������
#define OPTICALFLOW		 //����

/*���÷����������ͣ�����ͬʱֻ��ѡ��һ��*/
//#define MIXERCONFIG  MULTITYPE_BI	  	  //���������
#define MIXERCONFIG  MULTITYPE_QUADX	  //���������
//#define MIXERCONFIG  MULTITYPE_HEX6X 	  //���������

/*������ѹ���ͺţ�����ͬʱֻ��ѡ��һ��*/
//#define BMP085
#define MS5611

/*����IMUģ�飬Ŀǰʹ�ù����֣��ͽ��ϵĺ��µİɣ�ͬʱֻ��ѡ������һ��*/
#define IMU_NEWMODULE	//��ģ�飬Ӧ������ɫ��		 			   �����������Ϊ�¾�ģ���MPU6050�ĵ�ַ��һ������·����
//#define IMU_OLDMODULE	//��ģ�飬Ӧ���Ǻ�ɫ�Ǹ�




/*
*********************************************************************************************************
*                                         �������ȼ���ֵԽС��Խ�߼���
*********************************************************************************************************
*/
#define PRIO_APPTASKSTART      2
#define PRIO_SERIAL            20			//�������ͨѶ			//20
#define PRIO_SENSOR						 12			//���ִ��������ݶ�ȡ
#define PRIO_IMU							 8			//IMU��̬����
#define PRIO_ALTITUDE					 9			//�߶�
#define PRIO_ACTUATOR					 5		  //��������
#define PRIO_RC								 6		  //ң����
#define PRIO_MIXER             4		  //�������
#define PRIO_DAEMON						 21			//�ػ�����
#define PRIO_GPS							 14			//GPS
#define PRIO_TELEMETRY				 16			//��������
#define PRIO_OPTICALFLOW			 15			//����
#define PRIO_VISIONLANDING		 17			//�Ӿ��Զ�����
#define PRIO_EASYCTRL					 7			//���׿��ƣ��������ֱ�
#define PRIO_AVOID						 10			//��������

/*
*********************************************************************************************************
*                                   �ж����ȼ�(2λ��ռ2λ�ӣ�ȡֵ0-3��ԽС���ȼ�Խ��)
*********************************************************************************************************
*/
#define PREEMPTION_DMA2STREAM7	3			//�������ͨѶ��DMA�����ж�				//��
#define SUB_DMA2STREAM7					2																					//��
#define PREEMPTION_TIM2TIM3			0			//ң�ؽ��ջ�����PWM
#define SUB_TIM2TIM3						1
#define PREEMPTION_USART2				1			//GPS���ݴ����ж�
#define SUB_USART2							1
#define PREEMPTION_UART4				1			//�����������ݴ����ж�
#define SUB_UART4								2
#define PREEMPTION_I2C2EV				1			//i2c2�¼��ж�
#define SUB_I2C2EV							3			
#define PREEMPTION_EXTI9_5			2			//��������ж�
#define SUB_EXTI9_5							1			
#define PREEMPTION_EXTI15_10		2			//���ϳ�������ж�
#define SUB_EXTI15_10						2	

/*
*********************************************************************************************************
*                                        about OS
*********************************************************************************************************
*/
#define APP_CFG_TASK_START_STK_SIZE 256		//	����ջ��С



#endif
