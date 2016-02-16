#ifndef __AP_ALTITUDE_H
#define __AP_ALTITUDE_H
#include "includes.h"
extern int32_t  EstAlt_Baro;             // in cm 当前气压高度值
extern float  AltHold_Baro;							 //定高高度
extern float AltHold_Sonar; //超声定高高度
extern int16_t  errorAltitudeI;
extern int16_t  BaroPID;
extern uint16_t CalibratingB;
extern int16_t Altitude_HoldRcCommand;			 //开启定高后的RcCommand[THROTTLE]值
extern int16_t RawRcCommand_Val;
extern float vel;

extern float AltHold_TakeOff;//自动起飞定高高度，即起飞高度

void Altitude_Init(void);
#endif
