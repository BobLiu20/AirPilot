#ifndef __AP_ALTITUDE_H
#define __AP_ALTITUDE_H
#include "includes.h"
extern int32_t  EstAlt_Baro;             // in cm ��ǰ��ѹ�߶�ֵ
extern float  AltHold_Baro;							 //���߸߶�
extern float AltHold_Sonar; //�������߸߶�
extern int16_t  errorAltitudeI;
extern int16_t  BaroPID;
extern uint16_t CalibratingB;
extern int16_t Altitude_HoldRcCommand;			 //�������ߺ��RcCommand[THROTTLE]ֵ
extern int16_t RawRcCommand_Val;
extern float vel;

extern float AltHold_TakeOff;//�Զ���ɶ��߸߶ȣ�����ɸ߶�

void Altitude_Init(void);
#endif
