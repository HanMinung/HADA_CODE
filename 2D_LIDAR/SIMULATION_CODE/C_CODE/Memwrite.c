#pragma once
#ifndef MEMWRITE_C
#define MEMWRITE_C

#include "Memwrite.h"

void OpenClientLidar(void)
{
		TCHAR CLIE_LIDAR[] = TEXT("Lidar_smdat_ReadData");

		hMampF_CLI_Lidar = OpenFileMapping(
			FILE_MAP_READ,            // read/write access
			FALSE,                    // do not inherit the name
			CLIE_LIDAR);               // name of mapping object RP_SMDAT

		if (hMampF_CLI_Lidar == NULL)
		{
			_tprintf(TEXT("Lidar   : Could not open file mapping object (%d).\n"),
				GetLastError());
			exit(1);
			return 1;
		}

		CLI_smdat_Lidar = (Lidar_smdat*)MapViewOfFile(hMampF_CLI_Lidar, FILE_MAP_READ, 0, 0, sizeof(Lidar_smdat));

		if (CLI_smdat_Lidar == NULL)
		{
			_tprintf(TEXT("Lidar   : Could not map view of file (%d).\n"),
				GetLastError());
			exit(1);
			CloseHandle(hMampF_CLI_Lidar);
			return 1;
		}
		//printf("sm_connect\n")
}

void Import_Lidar()
{
		for (int m = 0; m < 761; m++)
		{
			Lidar_dis[m] = CLI_smdat_Lidar->dist[m];
		}
}


#endif // !MEMWRITE_C
