#ifndef __AP_PID_H
#define __AP_PID_H
#include "includes.h"

typedef struct {
	float _integrator;          ///< integrator value
	float _last_input;        ///< last input for derivative
	float _last_derivative;     ///< last derivative for low-pass filter
	float _output;
	float _derivative;
} PID_DATA;

typedef struct {
	float kP;
	float kI;
	float kD;
	float Imax;
	int8_t apply_lpf;
} PID_PARAM;

float apply_pid(PID_DATA * pid_data, float error, float dt, PID_PARAM * pid);
void PID_reset(PID_DATA * pid_data);
float PID_get_integrator(PID_DATA * pid_data);
void PID_set_integrator(PID_DATA * pid_data, float i);

#endif
