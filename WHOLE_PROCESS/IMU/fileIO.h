#ifndef _FILEIO
#define _FILEIO

#define DIR_SETTING         "../../../setting.txt"

// example Data
#define NROW_DATAIN         30000
typedef enum _NCOL_DATA
{
	NCOL_TIME = 0,
	NCOL_ROLL,
	NCOL_PITCH,
	NCOL_YAW,
	NCOL_LAT  = 7,
	NCOL_LONG,
	NCOL_VN   = 11,
	NCOL_VE,
	NCOL_VD,
	NCOL_AN,
	NCOL_AE,
	NCOL_AD,
	NCOL_H,
	NCOL_T,
	NCOL_PRE,
	NCOL_DATAIN
} _NCOL_DATA;


//=============================//
//	Declarations of Variables  //
//=============================//

double dataIn[NROW_DATAIN][NCOL_DATAIN];


//=============================//
//	Declarations of Functions  //
//=============================//


void readSettingData(void);

void dataRead(void);

void readExsceneData(void);

void importFileData(void);

void dataPrinting(void);


#endif


