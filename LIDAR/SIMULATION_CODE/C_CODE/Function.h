#pragma once
#ifndef FUNCTION
#define FUNCTION
extern int RecvArray_rubber[];
int key;
int count = 0;
int camMOD = 0;
int lidarMOD = 0;
int erpMOD = 1; // ERP mode 수정 필요

//====================
// MOD select
void MOD_select(int _camera, int _lidar)
{
	camMOD   = _camera;
	lidarMOD = _lidar ;
}

//====================
// ERPCMD

void Open_ERPconnect();
void Command_ERP42  ();
void CMD_delta_f    ();
void CommandERP     (int _spdcmd);

//====================
// Rubber

void FieldBuffer     ();
void LidarDataRead   ();
void GenerateOFF     ();
void GenerateSFF     ();
void GenerateIFF     ();
void Lidar_Clustering();
void SAVE_DATA       ();

//====================
// Memwrite
void OpenClientLidar ();
void Import_Lidar    ();
void SaveData_Flag   ();

//====================
// Time

double	GetWindowTime();
void    InitTImeCheck();
void    Idle_Time    ();

//=====================
// UDP
void Open_UDP_Socket ();
void wait_UDP(int _recvData[]);
void Recvdata_UDP_Ru ();
void Senddata_Cluster_Ru();
void Close_UDP       ();

//=====================
// Group
void SensorOpen()
{

	Open_ERPconnect();

	if (camMOD)
		Open_UDP_Socket();
	if (lidarMOD)
		OpenClientLidar();


}
void Initialization()
{
	if (lidarMOD)
		FieldBuffer();

	if (camMOD)
		wait_UDP(RecvArray_rubber);

}
void RevcData()
{
	if (camMOD) {

		Recvdata_UDP_Ru();
	}

	if (lidarMOD)
	{
		Import_Lidar();
		GenerateOFF(); // OFF Generate 켜줘야 save data 가 됨.
		//GenerateSFF();
		//GenerateIFF();
		Lidar_Clustering();
	}

	if (camMOD) {

		Senddata_Cluster_Ru();
		
		if (flag_clustering == 1)      // Cluster 2개 이하일때
		{
			if (RecvArray_rubber[0] == LEFT_CONE)
				delta_f = -MAX_STEER;
			else if (RecvArray_rubber[0] == RIGHT_CONE)
				delta_f = MAX_STEER;
			else
				delta_f = delta_f;
		}
		else if (flag_clustering == 2) // Cluster 3개이하일때
		{
			if (RecvArray_rubber[0] == LEFT_CONE)
				delta_f = -MAX_STEER;
			else if (RecvArray_rubber[0] == RIGHT_CONE)
				delta_f = MAX_STEER;
			else
				delta_f = delta_f;
		}
	}
		
}
#endif // !FUNCTION
