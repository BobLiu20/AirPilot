#ifndef __AP_EASYCTRL_H
#define __AP_EASYCTRL_H
#include "includes.h"

extern int16_t RC_Data[8];//ģ�����е�RC_Data,һ��Ӧ��ʹ�ô�ֵ
extern int16_t HandAttitude_CtrlAngle[2];	//���ƵĽǶȣ�-35----+35

extern uint8_t OneKey_RollStep;//ָʾ�����Ĳ��������뷭��ģʽʱ�����ر����ȣ�����3Dģʽ

extern void HandAttitude_Ctrl(uint8_t *hand_data);
extern void EasyCtrl_Init(void);

#endif
