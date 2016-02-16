#ifndef __AP_SONAR_H
#define __AP_SONAR_H
#include "includes.h"
#ifdef SONAR

extern float EstAlt_Sonar;//³¬Éù²â¾à¸ß¶È,CM

void Sonar_Init(void);
void Sonar_Update(void);

#endif
#endif
