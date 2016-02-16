/**
	******************************************************************************
	* @file    AP_PID.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   PID¿â
	******************************************************************************
**/
#include "AP_PID.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
/* function prototypes -----------------------------------------------*/
static float PID_get_p(PID_DATA * pid_data, float error, PID_PARAM * pid);
static float PID_get_i(PID_DATA * pid_data, float error, float dt, PID_PARAM * pid);
static float PID_get_d(PID_DATA * pid_data, float error, float dt, PID_PARAM * pid);

/**
  * @brief  
  *         
  * @param  None
  * @retval None
  */
float apply_pid(PID_DATA * pid_data, float error, float dt, PID_PARAM * pid)
{

	return (PID_get_p(pid_data, error, pid) +PID_get_i(pid_data, error, dt, pid) + PID_get_d(pid_data, error, dt, pid));

}

/**
  * @brief  Reset the PID integrator
  *         
  * @param  None
  * @retval None
  */
void PID_reset(PID_DATA * pid_data)
{
    pid_data->_integrator = 0;
    pid_data->_last_input = 0;
    pid_data->_last_derivative = 0;
}

/**
  * @brief  
  *         
  * @param  None
  * @retval None
  */
float PID_get_integrator(PID_DATA * pid_data)
{
    return pid_data->_integrator;
}


void PID_set_integrator(PID_DATA * pid_data, float i)
{
    pid_data->_integrator = i;
}


/***********************
/// Iterate the PID, return the new control value
///
/// Positive error produces positive output.
///
/// @param error        The measured error value
/// @param dt           The time delta in milliseconds (note
///                                     that update interval cannot be more
///                                     than 65.535 seconds due to limited range
///                                     of the data type).
/// @param scaler       An arbitrary scale factor
///
/// @returns            The updated control output.
***********************/
static float PID_get_p(PID_DATA * pid_data, float error, PID_PARAM * pid)
{
    return error * pid->kP;
}

static float PID_get_i(PID_DATA * pid_data, float error, float dt, PID_PARAM * pid)
{
    pid_data->_integrator += ( error * pid->kI) * dt;
    if (pid_data->_integrator < -pid->Imax) {
        pid_data->_integrator = -pid->Imax;
    } else if (pid_data->_integrator > pid->Imax) {
        pid_data->_integrator = pid->Imax;
    }
    return pid_data->_integrator;
}


#define PID_cutoff_frequency 20
#define PID_FILTER       (1.0f / (2.0f * M_PI * PID_cutoff_frequency))
static float PID_get_d(PID_DATA * pid_data, float error, float dt, PID_PARAM * pid)
{
    pid_data->_derivative = (error - pid_data->_last_input) / dt;
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
	if(pid -> apply_lpf)
	{
		pid_data->_derivative = pid_data->_last_derivative + (dt / (PID_FILTER + dt)) * (pid_data->_derivative - pid_data->_last_derivative);
	}
    // update state
    pid_data->_last_input = error;
    pid_data->_last_derivative = pid_data->_derivative;
    // add in derivative component
    return pid->kD * pid_data->_derivative;
}



