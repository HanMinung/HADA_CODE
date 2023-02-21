#pragma once


#ifndef _REALTIMECHECK
#define _REALTIMECHECK
#include "Include.h"

double init_t = 0.0;
double prev_t = 0.0;
int    simul_cnt = 0;
double simu_t = 0.0;
double curr_t = 0.0;
double idle_t = 0.0;
double delt_t = 0.0;


double	GetWindowTime();

#endif // !_REALTIMECHECK