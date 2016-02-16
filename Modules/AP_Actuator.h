#ifndef __AP_ACTUATOR_H
#define __AP_ACTUATOR_H
#include "includes.h"

extern uint8_t RC_Options[CHECKBOXITEMS];
extern int16_t RC_Command[4];
extern int16_t HeadFreeModeHold;		//无头模式
extern uint16_t CycleTime;
extern int16_t axisPID[3];

extern void Actuator_Init(void);
#endif
