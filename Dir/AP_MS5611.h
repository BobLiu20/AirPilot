#ifndef __MS5611_H
#define __MS5611_H
#include "includes.h"

#ifdef MS5611

#define BARO_TAB_SIZE 21	//����ƽ���˲��ĳ���
extern uint32_t BaroPressureSum;	//��������BARO_TAB_SIZE�ε��ܺ�
extern int32_t BaroTemperature;				//У׼����¶�

bool MS5611_Detect(baro_t *baro);
void Baro_Common(s32);
#endif
#endif
