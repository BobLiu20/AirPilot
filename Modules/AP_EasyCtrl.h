#ifndef __AP_EASYCTRL_H
#define __AP_EASYCTRL_H
#include "includes.h"

extern int16_t RC_Data[8];//模拟运行的RC_Data,一般应该使用此值
extern int16_t HandAttitude_CtrlAngle[2];	//控制的角度，-35----+35

extern uint8_t OneKey_RollStep;//指示翻滚的步，当进入翻滚模式时，将关闭自稳，进入3D模式

extern void HandAttitude_Ctrl(uint8_t *hand_data);
extern void EasyCtrl_Init(void);

#endif
