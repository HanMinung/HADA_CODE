#include "Include.h"
#include "Function.h"

extern int BREAK_FLAG;

void main()
{
	MOD_select     (CAMERA_ON, LIDAR_ON);

	SensorOpen     ();

	Initialization ();
	
	InitTImeCheck  ();

	do
	{
		RevcData  ()             ;

		CommandERP(MAX_SPEED)    ;
		
		FMOD(simu_t, 1, simu_t)
		KEY (key)
		if(BREAK_FLAG == 1)
			break;
		Idle_Time ()             ;

	} while (simu_t < FINAL_TIME);

	SAVE_DATA();
	SaveData_Flag();
}