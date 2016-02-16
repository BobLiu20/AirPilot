#ifndef __XBEEPRO_H
#define __XBEEPRO_H
#include "includes.h"

#ifdef XBEEPRO

void XbeePro_Init(void);
void XbeePro_Process(void);

extern unsigned char XbeePro_ImportanceInfo;
extern int16_t Xbee_Debug[3];//≤‚ ‘œ‘ æ”√
#endif
#endif
