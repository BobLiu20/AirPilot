#ifndef __AP_MIXER_H
#define __AP_MIXER_H
#include "includes.h"

extern OS_FLAG_GRP Mixer_TaskRunFlagGrp;	//���������¼���־��
extern int16_t Motor_Val[6];	 //���
extern int16_t Servo_Val[2];

void Mixer_Init(void);
void Mixer_WriteAllMotors(int16_t mc);

#endif
