#ifndef __DAEMON_H
#define __DAEMON_H
#include "includes.h"

/*
*********************************************************************************************************
*                                         LED1指示
*********************************************************************************************************
*/
extern uint16_t LED_FlashBit;//记录着需要闪烁的位
#define LEDFLASH_ARMD   		(1<<0)		//解锁状态
#define LEDFLASH_SENSORFAIL (1<<1)		//传感器检测错误，连续闪烁
#define LEDFLASH_MAGCAL     (1<<2)		//磁阻校准时闪烁30秒
#define LEDFLASH_SENSORCAL	(1<<3)		//陀螺仪或者加速度校准
#define LEDFLASH_AIRSLOPE   (1<<4)		//解锁前，机身过于倾斜

#define LEDFLASH_SET(x) LED_FlashBit|=x	//设置
#define LEDFLASH_CLR(x) LED_FlashBit&=~x //清除

/*
*********************************************************************************************************
*                                         允许解锁保护
*********************************************************************************************************
*/
extern uint8_t NotOkToArmBit;//当某一位为1时，则不可解锁,会清除f.OK_TO_ARM
#define NOTARMBIT_LOSTRC		(1<<0)		//解锁前，丢失遥控信号，不允许解锁
#define NOTARMBIT_ANGLE100	(1<<1)		//倾斜超过100度，通常将其认为是坠机
#define NOTARMBIT_ANGLE25		(1<<2)		//解锁前倾斜超过25度
#define NOTARMBIT_CALING		(1<<3)		//正在进行陀螺仪或加速度或磁阻的校准，不允许解锁
#define NOTARMBIT_I2CERR		(1<<4)		//I2C通讯出错数过大，不允许解锁

#define NOTOKTOARM_SET(x) NotOkToArmBit|=x	//设置
#define NOTOKTOARM_CLR(x) NotOkToArmBit&=~x //清除对应禁止解锁位


extern uint8_t NeedToWriteFlash;//当此值为1时，需要写入FLASH操作
#define WRITE_TO_FLASH NeedToWriteFlash=1	//标记将参数写入FLASH

extern uint16_t Rc_LostCnt;//遥控信号丢失计数
extern uint16_t Handle_LostCnt;//手柄信号丢失计数



extern void Daemon_Init(void);

#endif
