#ifndef __MPU6050_H
#define __MPU6050_H
#include "includes.h"

bool MPU6050_Detect(sensor_t * acc, sensor_t * gyro, uint8_t scale);

#endif
