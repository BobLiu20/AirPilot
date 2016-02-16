#ifndef __AP_SENSOR_H
#define __AP_SENSOR_H
#include "includes.h"

extern uint16_t CalibratingA ;       //想进行加速度校准时，请将calibratingA幅值为400
extern uint16_t CalibratingG ;
extern uint16_t acc_1G;
extern int16_t gyroADC[3], accADC[3], magADC[3];
extern float MagneticDeclination; //磁偏角校准值

void Sensor_DetectAndInit(void);
void Gyro_GetADC(void);
void Acc_GetADC(void);
#endif
