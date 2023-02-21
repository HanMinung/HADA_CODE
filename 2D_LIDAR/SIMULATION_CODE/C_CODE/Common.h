#pragma once
#ifndef COMMON
#define COMMON

FILE* dataFile;

//=================================
// Time check
#define      FINAL_TIME             200.0    //[sec]
#define      SAMPLING_TIME          0.05    //[sec]
#define      BUF_SIZE				(int)(FINAL_TIME/SAMPLING_TIME)

//==================================
// Saving Data

#define      LOOP(Count,End)                      for( Count=0; Count<End; Count++ )
#define      VEC_PRT_OFF(X,UNT )				     {LOOP(j,GET_DIM(X[i])) fprintf( offFile, "%14.5f\t", X[i][j]*UNT  );}
#define      STR_PRT_OFF(X     )                  {fprintf( offFile, "%s",   X                );}
#define      GET_DIM(X)							 (sizeof(X) / sizeof(X[0]))
#define      VEC_WRT(BUF,ARAY,COUNTO)		     {LOOP(i, GET_DIM(ARAY)) BUF[COUNTO][i] = ARAY[i];}
	        
#define      MAX_LASER_RECEIVE	 (unsigned int)(   16*1024)
	        
#define      SteerMODE      1
#define      SpeedMODE      1

#define      minPoint       (int)     6
#define      boundary       (double) 0.08
#define      RangeMAX               3000           //500 per 1[m]
#define      MAX_NEIGHBOR_NUM     520000
#define      MAX_LIDAR_NUM	         761	         //500360
#define      CLUSTER_NUM             721
#define      MAX_ANGLE_POSITIVE       28 * DEG2RAD
#define      MAX_ANGLE_NEGATIVE      -28 * DEG2RAD

//================================
// LOCAL
#define      K_LOCAL              0.12           // {scalefactor(500) * meter} / {scalefactor(500)*max_range}

//=================================
// UNIT
#define      PI                 3.1415926535
#define      DEG2RAD            PI/180.0
#define      RAD2DEG            180.0/PI

//==================================
// MOD
enum Camera_Mod { CAMERA_OFF, CAMERA_ON };
enum Lidar_Mod  { LIDAR_OFF ,  LIDAR_ON };

#define      FMOD(STAN,SAMPLE_T,VARI)		 if(fmod(STAN,SAMPLE_T)==0){printf("%.0f\n", VARI);};
#define      LEFT_CONE   1
#define      RIGHT_CONE  2
//===================================
// ERP CMD
#define      MAX_STEER             28.0
#define      MAX_SPEED            (double) (90.0)
					               
#define      MAXVEL1                (double)(90.0)
#define      MAXVEL2                (double)(50.0)
#define      MINVEL                (double)(50.0)
//===================================
#define KEY(K) 		if (_kbhit()) {printf("tddd\n");K = _getch();if (K == 'f');break;}

#endif // !COMMON
