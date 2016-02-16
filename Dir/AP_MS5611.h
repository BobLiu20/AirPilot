#ifndef __MS5611_H
#define __MS5611_H
#include "includes.h"

#ifdef MS5611

#define BARO_TAB_SIZE 21	//滑动平均滤波的长度
extern uint32_t BaroPressureSum;	//用来保存BARO_TAB_SIZE次的总和
extern int32_t BaroTemperature;				//校准后的温度

bool MS5611_Detect(baro_t *baro);
void Baro_Common(s32);
#endif
#endif
