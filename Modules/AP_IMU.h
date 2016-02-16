#ifndef __AP_IMU_H
#define __AP_IMU_H
#include "includes.h"

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

extern float accLPFVel[3];
extern t_fp_vector EstG;

extern int16_t gyroData[3],accSmooth[3];
extern int16_t angle[2];     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
extern int16_t heading,magHold;//当前灰机的角度(-180~180)
extern int16_t accZ;

void IMU_Init(void);

#endif
