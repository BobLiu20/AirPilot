#ifndef __BATTERY_H
#define __BATTERY_H
#include "includes.h"

extern u16 Battery_Val;				//���ﱣ���ŵ��ڵ�ص���ʵ��ѹֵ����ʵ��ѹֵ=Battery_Val/100;  Ҳ�������100��
extern u8 Battery_LowPower;			//��������ָʾ�����Ϊ1��ʾ���㣬Ϊ0��ʾ������

void Battery_Init(void);
void Battery_Read(void);


#endif
