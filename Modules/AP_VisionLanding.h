#ifndef __AP_VISIONLANDING_H
#define __AP_VISIONLANDING_H
#include "includes.h"

extern int16_t VisionLanding_Angle[2];
extern int16_t VisionLanding_Altitude;//cm����ģ���л�ȡ�ĳ����߶�

void VisionLanding_ReceiveRawData(uint8_t *buf);

void VisionLanding_Init(void);

#endif
