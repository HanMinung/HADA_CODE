#include "sol.h"

double GetWindowTime(void)
{
	LARGE_INTEGER liCount, liFreq;
	QueryPerformanceCounter(&liCount); // �ð� �Լ� �и� ������ ������ ������ �����ϴ�
	QueryPerformanceFrequency(&liFreq); // ������/[sec]
	return (( liCount.QuadPart / ((double) ( liFreq.QuadPart ))) * 1000.0 );
};
