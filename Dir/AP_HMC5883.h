#ifndef __HMC5883_H
#define __HMC5883_H
#include "includes.h"
bool HMC5883_Detect(sensor_t * mag);
void HMC5883_Cal(uint8_t calibration_gain);
void HMC5883_FinishCal(void);

#endif
