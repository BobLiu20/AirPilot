#ifndef __BATTERY_H
#define __BATTERY_H
#include "includes.h"

extern u16 Battery_Val;				//这里保存着单节电池的真实电压值，真实电压值=Battery_Val/100;  也就是这个100倍
extern u8 Battery_LowPower;			//电量不足指示，如果为1表示充足，为0表示不足了

void Battery_Init(void);
void Battery_Read(void);


#endif
