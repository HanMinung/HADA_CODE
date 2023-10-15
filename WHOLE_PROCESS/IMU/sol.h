#ifndef _SOL
#define _SOL

//--------------------------------------------------------------------------------
// Public Xsens device API C++ example MTi receive data.
//--------------------------------------------------------------------------------
#include "xspublic/xscontroller/xscontrol_def.h"
#include "xspublic/xscontroller/xsdevice_def.h"
#include "xspublic/xscontroller/xsscanner.h"
#include "xspublic/xstypes/xsoutputconfigurationarray.h"
#include "xspublic/xstypes/xsdatapacket.h"
#include "xspublic/xstypes/xstime.h"
#include "xspublic/xscommon/xsens_mutex.h"
#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include <windows.h>
#include <assert.h>

//=====================================================================
//=====================================================================
//For Shared Memory
#include "consoleDesign.h"

#include "targetver.h"
#include "SerialHeader.h"
#include <stdio.h>
#include <conio.h>
#include <tchar.h>

// =====================
//          GPS
// =====================
//#include "SerialHeader.h"



double GetWindowTime(void);

#endif