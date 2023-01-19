#pragma once
#ifndef MEMWRITE_H
#define MEMWRITE_H

#include "Include.h"

//======================================
// Data Received

double  Lidar_dis  [MAX_LIDAR_NUM];
double  Lidar_xCoor[MAX_LIDAR_NUM];
double  Lidar_yCoor[MAX_LIDAR_NUM];

//======================================
// Shared Memory

typedef struct
{
	double   xCoordinate[MAX_LIDAR_NUM];
	double   yCoordinate[MAX_LIDAR_NUM];
	double         dist[MAX_LIDAR_NUM];
	double        angle[MAX_LIDAR_NUM];
	char     stringDATA[MAX_LASER_RECEIVE];
}Lidar_smdat;



HANDLE           hMampF_CLI_Lidar;
Lidar_smdat*     CLI_smdat_Lidar;

#endif // !MEMWRITE_H
