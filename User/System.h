#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "includes.h"
#include <stm32f4xx.h>

uint32_t TimUsGet(void);
uint32_t TimMsGet(void);
void DelayUs(uint32_t us);
void System_FailureMode(uint8_t mode);

#endif
