#ifndef __AP_PWM_H
#define __AP_PWM_H
#include "includes.h"

void PWM_RcCaptureInit(void);
int16_t PWM_RcValRead(uint8_t channel);
void PWM_EscPwmInit(void);
void PWM_WriteMotor(uint8_t index, uint16_t value);
void PWM_WriteServo(uint8_t index, uint16_t value);

#endif
