#pragma once

#ifndef REALTIME_CHECK
#define REALTIME_CHECK


#include "TimeCheck.h"

double	GetWindowTime(void)
{
	LARGE_INTEGER	liEndCounter, liFrequency;

	QueryPerformanceCounter(&liEndCounter);
	QueryPerformanceFrequency(&liFrequency);

	return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart));
};

void InitTImeCheck()
{
	init_t = GetWindowTime();
	prev_t = init_t;

	simul_cnt = 0;
	simu_t = 0.0;

	count = 0;
}

void Idle_Time()
{
	count++;
	curr_t = GetWindowTime();
	idle_t = (curr_t - init_t) - simu_t;

	while (1)
	{
		curr_t = GetWindowTime();
		delt_t = curr_t - init_t - simu_t;

		if (delt_t >= SAMPLING_TIME) break;
	}

	simu_t = SAMPLING_TIME * ((double)simul_cnt + 1.0);

	simul_cnt++;
}

#endif // !REALTIME_CHECK
