#ifndef __CONFIG_H
#define __CONFIG_H
/*
*********************************************************************************************************
*                          配置是否使用这些传感器或模块,不使用的请注释
*********************************************************************************************************
*/
#define GYRO
#define ACC
#define MAG
#define BARO
#define SONAR			   //超声定高
#define XBEEPRO			   //无线数传
#define OPTICALFLOW		 //光流

/*配置飞行器的类型，下面同时只能选择一个*/
//#define MIXERCONFIG  MULTITYPE_BI	  	  //两轴飞行器
#define MIXERCONFIG  MULTITYPE_QUADX	  //四轴飞行器
//#define MIXERCONFIG  MULTITYPE_HEX6X 	  //六轴飞行器

/*配置气压计型号，下面同时只能选择一个*/
//#define BMP085
#define MS5611

/*配置IMU模块，目前使用过两种，就叫老的和新的吧，同时只能选择其中一个*/
#define IMU_NEWMODULE	//新模块，应该是蓝色吧		 			   出现这个是因为新旧模块的MPU6050的地址不一样，电路问题
//#define IMU_OLDMODULE	//旧模块，应该是红色那个




/*
*********************************************************************************************************
*                                         任务优先级（值越小，越高级）
*********************************************************************************************************
*/
#define PRIO_APPTASKSTART      2
#define PRIO_SERIAL            20			//调参软件通讯			//20
#define PRIO_SENSOR						 12			//部分传感器数据读取
#define PRIO_IMU							 8			//IMU姿态计算
#define PRIO_ALTITUDE					 9			//高度
#define PRIO_ACTUATOR					 5		  //核心中枢
#define PRIO_RC								 6		  //遥控器
#define PRIO_MIXER             4		  //动力输出
#define PRIO_DAEMON						 21			//守护进程
#define PRIO_GPS							 14			//GPS
#define PRIO_TELEMETRY				 16			//无线数传
#define PRIO_OPTICALFLOW			 15			//光流
#define PRIO_VISIONLANDING		 17			//视觉自动降落
#define PRIO_EASYCTRL					 7			//简易控制，包含有手柄
#define PRIO_AVOID						 10			//超声避障

/*
*********************************************************************************************************
*                                   中断优先级(2位抢占2位从，取值0-3，越小优先级越大)
*********************************************************************************************************
*/
#define PREEMPTION_DMA2STREAM7	3			//调参软件通讯，DMA发送中断				//主
#define SUB_DMA2STREAM7					2																					//从
#define PREEMPTION_TIM2TIM3			0			//遥控接收机捕获PWM
#define SUB_TIM2TIM3						1
#define PREEMPTION_USART2				1			//GPS数据传输中断
#define SUB_USART2							1
#define PREEMPTION_UART4				1			//无线数传数据传输中断
#define SUB_UART4								2
#define PREEMPTION_I2C2EV				1			//i2c2事件中断
#define SUB_I2C2EV							3			
#define PREEMPTION_EXTI9_5			2			//超声测距中断
#define SUB_EXTI9_5							1			
#define PREEMPTION_EXTI15_10		2			//避障超声测距中断
#define SUB_EXTI15_10						2	

/*
*********************************************************************************************************
*                                        about OS
*********************************************************************************************************
*/
#define APP_CFG_TASK_START_STK_SIZE 256		//	任务栈大小



#endif
