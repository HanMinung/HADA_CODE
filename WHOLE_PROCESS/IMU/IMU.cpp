#include "sol.h"

using namespace std;

typedef struct
{
	//unsigned int index_imu;
	//double roll;
	//double pitch;
	//double yaw;
	//double accX;
	//double accY;
	//double accZ;
	int    SIM_count;
	double SIM_Time;
	double euler_x;
	double euler_y;
	double euler_z;
	double acc_x;
	double acc_y;
	double acc_z;
	double vel_x;
	double vel_y;
	double vel_z;
	double rot_x;
	double rot_y;
	double rot_z;
} IMU_DATA;

typedef struct Flag
{
	int flag;
} ReadData_flag;

TCHAR IMU_SM[]      = TEXT("Xsens_smdat_ReadData");//imu data");
TCHAR IMU_SM_FLAG[] = TEXT("Xsens_ReadData_Flag");

//=====================================================================
//=====================================================================

#define RUNTIME                       (double)             (10000.0)      //[sec]
#define FREQ                          (double)             (100.0)      //[Hz]
#define N_data                        (unsigned int)       (RUNTIME*FREQ)
#define CoordSys                                            (XDI_CoordSysNed)

double euler_roll  = 0.0;
double euler_pitch = 0.0;
double euler_yaw   = 0.0;

// =========================================
// =========================================
double iniTime = 0.0;
double curTime = 0.0;
double delTime = 0.0;
double simTime = 0.0;
double task1   = 0.0;
double task2   = 0.0;
double task3   = 0.0;
double Ts      = 1 / FREQ;
int    simcnt  = 0;
//=====================================================================
//==========================Array name=================================

double euler[N_data][3]      = { 0.0, }; //euler angle(roll pitch yaw)
double acc[N_data][3]        = { 0.0, };  // acceleration (X Y Z)
double latlon[N_data][2]     = { 0.0, };
double raw_latlon[N_data][2] = { 0.0, };
double Buf_sim[N_data]       = { 0.0, };

uint16_t simCnt_imu = 0;
// XsStatusFlag.h
enum XsStatusFlag
{
	XSF_SelfTestOk       = 0x01      //!< Is set when the self test result was ok
	,
	XSF_OrientationValid = 0x02      //!< Is set when the computed orientation is valid. The orientation may be invalid during startup or when the sensor data is clipping during violent (for the device) motion
	,
	XSF_GpsValid         = 0x04      //!< Is set when the device has a GPS receiver and the receiver says that there is a GPS position fix.
	
	,
	XSF_NoRotationMask            = 0x18      //!< If all of these flags are set, the No Rotation algorithm is running
	,
	XSF_NoRotationAborted         = 0x10      //!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm was aborted
	,
	XSF_NoRotationSamplesRejected = 0x08      //!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running but has rejected samples
	,
	XSF_NoRotationRunningNormally = 0x18      //!< If all these flags are set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running normally
	
	,
	XSF_RepresentativeMotion = 0x20      //!< Indicates if the In-Run Compass Calibration is doing the representative motion analysis
	
	,
	XSF_ExternalClockSynced = 0x40      //!< Indicates whether the internal clock is synced with an external clock (Either GNNS or custom provided clock sync)
	
	,
	XSF_ClipAccX         = 0x00000100,
	XSF_ClipAccY         = 0x00000200,
	XSF_ClipAccZ         = 0x00000400,
	XSF_ClipGyrX         = 0x00000800,
	XSF_ClipGyrY         = 0x00001000,
	XSF_ClipGyrZ         = 0x00002000,
	XSF_ClipMagX         = 0x00004000,
	XSF_ClipMagY         = 0x00008000,
	XSF_ClipMagZ         = 0x00010000,
	XSF_Retransmitted    = 0x00040000   //!< When set Indicates the sample was received as a retransmission
	,
	XSF_ClippingDetected = 0x00080000   //!< When set Indicates clipping has occurred
	,
	XSF_Interpolated     = 0x00100000   //!< When set Indicates the sample is an interpolation between other samples
	,
	XSF_SyncIn           = 0x00200000   //!< When set indicates a sync-in event has been triggered
	,
	XSF_SyncOut          = 0x00400000   //!< When set Indicates a sync-out event has been generated
	
	,
	XSF_FilterMode        = 0x03800000   //!< Mask for the 3 bit filter mode field
	,
	XSF_HaveGnssTimePulse = 0x04000000   //!< Indicates that the 1PPS GNSS time pulse is present
	
	,
	XSF_RtkStatus = 0x18000000   //!< Mask for 2 bit RTK status field 00: No RTK; 01: RTK floating; 10: RTK fixed
};

//======================================================================
//===========================function===================================

Journaller* gJournal = 0;
FILE      * pFile;

using namespace std;

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
			: m_maxNumberOfPacketsInBuffer(maxBufferSize), m_numberOfPacketsInBuffer(0)
	{
	}
	
	virtual ~CallbackHandler() throw()
	{
	}
	
	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}
	
	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock  locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != 0); // if false (= 0) program is aborted, packet = 0�϶�
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void) getNextPacket();
		
		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer); // if num > max program is aborted
	}

private:
	mutable xsens::Mutex m_mutex;
	
	size_t             m_maxNumberOfPacketsInBuffer;
	size_t             m_numberOfPacketsInBuffer;
	list<XsDataPacket> m_packetBuffer;
};


int main()
{
	readSettingData();
	initConsole();
	
	HANDLE hMemoryMap = NULL;
	LPBYTE pMemoryMap = NULL; // LPBYTE�� unsigned char�� ��������
	
	HANDLE hMemoryMap_Flag = NULL;
	LPBYTE pMemoryMap_Flag = NULL;
	
	/////////////////////////////////////////////////////////////////////////////
	//For Xsens Data
	hMemoryMap = ::CreateFileMapping(
			(HANDLE) 0xffffffff, // ���� ���� �ڵ�, �ʱ⿡ 0xffffffff�� �����Ѵ�.
			NULL,            // ���� �Ӽ�
			PAGE_READWRITE,     // �а�/���� �Ӽ�
			0,               // 64��Ʈ ��巹���� ����Ѵ�. ���� 32��Ʈ - �޸��� ũ��
			sizeof(IMU_DATA),   // ���� 32��Ʈ - ���⼱LPBYTE Ÿ��.
			IMU_SM);    // ���� ���ϸ��� �̸� - Uique �ؾ��Ѵ�.
	
	if ( !hMemoryMap )
	{
		::MessageBox(NULL, L"���� �޸𸮸� ������ �� �����ϴ�.", L"Error", MB_OK);
		return FALSE;
	}
	
	pMemoryMap = (BYTE*) ::MapViewOfFile(
			hMemoryMap,            // ���ϸ��� �ڵ�
			FILE_MAP_ALL_ACCESS,    // �׼��� ��� - ����� ����
			0,                  // �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ
			0,                  // �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ
			0);                  // ����� �޸� ����� ũ�� - 0�̸� ������ ��ü �޸�
	
	if ( !pMemoryMap )
	{
		CloseHandle(hMemoryMap);
		::MessageBox(NULL, L"���� �޸𸮸� ���� �����ϴ�.", L"Error", MB_OK);
		return FALSE;
	}
	
	IMU_DATA* CLI_smdat_Xsens = (IMU_DATA*) pMemoryMap;
	
	/////////////////////////////////////////////////////////////////////////////
	//For Xsens Flag
	hMemoryMap_Flag = ::CreateFileMapping(
			(HANDLE) 0xffffffff, // ���� ���� �ڵ�, �ʱ⿡ 0xffffffff�� �����Ѵ�.
			NULL,            // ���� �Ӽ�
			PAGE_READWRITE,     // �а�/���� �Ӽ�
			0,               // 64��Ʈ ��巹���� ����Ѵ�. ���� 32��Ʈ - �޸��� ũ��
			sizeof(ReadData_flag),   // ���� 32��Ʈ - ���⼱LPBYTE Ÿ��.
			IMU_SM_FLAG);    // ���� ���ϸ��� �̸� - Uique �ؾ��Ѵ�.
	
	if ( !hMemoryMap_Flag )
	{
		::MessageBox(NULL, L"���� �޸𸮸� ������ �� �����ϴ�.", L"Error", MB_OK);
		return FALSE;
	}
	
	pMemoryMap_Flag = (BYTE*) ::MapViewOfFile(
			hMemoryMap_Flag,            // ���ϸ��� �ڵ�
			FILE_MAP_ALL_ACCESS,    // �׼��� ��� - ����� ����
			0,                  // �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ
			0,                  // �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ
			0);                  // ����� �޸� ����� ũ�� - 0�̸� ������ ��ü �޸�
	
	if ( !pMemoryMap_Flag )
	{
		CloseHandle(hMemoryMap_Flag);
		::MessageBox(NULL, L"���� �޸𸮸� ���� �����ϴ�.", L"Error", MB_OK);
		return FALSE;
	}
	
	ReadData_flag* imu_smdat_Flag = (ReadData_flag*) pMemoryMap_Flag;
	
	//=============================================================================
	
	
	
	int count = 0;
	int filter_mode;
	cout << "Creating XsControl object..." << endl;
	
	XsControl* control = XsControl::construct();
	assert(control != 0);
	
	// Lambda function for error handling
	auto handleError = [ = ](string errorString)
	{
		control->destruct();
		
		if ( pMemoryMap ) UnmapViewOfFile(pMemoryMap);
		
		if ( hMemoryMap ) CloseHandle(hMemoryMap);
		
		
		cout << errorString << endl;
		cout << "Press Any Key to continue." << endl;
		_getch();
		//cin.get();
		return -1;
	};
	
	cout << "Scanning for devices..." << endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();
	
	// Find an MTi device
	XsPortInfo      mtPort;
	for (auto const &portInfo: portInfoArray)
	{
		if ( portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			
			break;
		}
	}
	
	if ( mtPort.empty())
		return handleError("No MTi device found. Aborting.");
	
	cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: "
	     << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;
	
	cout << "Opening port..." << endl;
	if ( !control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");
	
	// Get the device object
	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != 0);
	cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString()
	     << " opened." << endl;
	
	// Create and attach callback handler to device
	CallbackHandler callback;
	device->addCallbackHandler(&callback);
	
	// Put the device into configuration mode before configuring the device
	cout << "Putting device into configuration mode..." << endl;
	if ( !device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");
	
	cout << "Configuring the device..." << endl;
	
	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	
	device->readEmtsAndDeviceConfiguration();
	
	
	//filter_mode = 04;
	//if (!device->setOnboardFilterProfile(filter_mode)) {
	//    return handleError("Could not set filter mode Aborting");
	//}
	
	XsOutputConfigurationArray configArray;
	
	if ( device->deviceId().isAhrs())     // if mti-630 : isAhrs() , if mti-710 : isGnss()
	{
		
		configArray.push_back(XsOutputConfiguration(XDI_EulerAngles, FREQ)); // heading roll pitch yaw
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, FREQ));
		configArray.push_back(XsOutputConfiguration(XDI_LatLon, FREQ));
		//configArray.push_back(XsOutputConfiguration(XDI_GnssPvtData, 4));
		
	}
	else
	{
		return handleError("Unknown device while configuring. Aborting.");
	}
	
	if ( !device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");
	
	cout << "Putting device into measurement mode..." << endl;
	if ( !device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");
	
	//==============================================================================
	
	
	cout << "\nMain loop. Recording data " << endl;
	cout << string(79, '-') << endl;
	
	//int64_t startTime = XsTime::timeStampNow();
	iniTime = GetWindowTime() * 0.001;
	
	// while(flag)
	do
	{
		//if (XsTime::timeStampNow() - startTime > RUNTIME * 1000) { flag = 0; break; }
		
		if ( callback.packetAvailable())
		{
			// Retrieve a packet
			XsDataPacket packet = callback.getNextPacket();
			
			
			if ( packet.containsOrientation())
			{
				
				XsEuler Euler = packet.orientationEuler(CoordSys);
				euler[ simcnt ][ 0 ] = Euler.roll();
				euler[ simcnt ][ 1 ] = Euler.pitch();
				euler[ simcnt ][ 2 ] = Euler.yaw();
				
			}
			
			if ( packet.containsCalibratedAcceleration())
			{
				XsVector Acc = packet.calibratedAcceleration();
				acc[ simcnt ][ 0 ] = Acc[ 0 ];
				acc[ simcnt ][ 1 ] = Acc[ 1 ];
				acc[ simcnt ][ 2 ] = Acc[ 2 ];
				
				
			}
			if ( packet.containsLatitudeLongitude())
			{
				XsVector LatLon = packet.latitudeLongitude();
				latlon[ simcnt ][ 0 ] = (double) LatLon[ 0 ];
				latlon[ simcnt ][ 1 ] = (double) LatLon[ 1 ];
			}
			
			/* if (packet.containsRawGnssPvtData())
			 {
				 XsRawGnssPvtData RawGnss = packet.rawGnssPvtData();
				 raw_latlon[simcnt][0] = (double)RawGnss.m_lat;
				 raw_latlon[simcnt][0] = (double)RawGnss.m_lon;
			 }*/
			
			
			// Store in shared memory
			/*imu_smdat->index_imu = simcnt;
			imu_smdat->roll = euler[simcnt][0];
			imu_smdat->pitch = euler[simcnt][1];
			imu_smdat->yaw = euler[simcnt][2];
			imu_smdat->accX = acc[simcnt][0];
			imu_smdat->accY = acc[simcnt][1];
			imu_smdat->accZ = acc[simcnt][2];*/
			//strcpy(imu_smdat->buffer, "IMU is f**king good");
			
			
			/* printf("Roll: %lf, Pitch: %lf, Yaw: %lf, AccX: %lf, AccY: %lf, AccZ: %l0f\n, Lat: %lf, Lon: %lf, RawLat:%lf, RawLon:%lf\n,", euler[simcnt][0], euler[simcnt][1], euler[simcnt][2], acc[simcnt][0], acc[simcnt][1], acc[simcnt][2], latlon[simcnt][0],latlon[simcnt][1], raw_latlon[simcnt][0],raw_latlon[simcnt][1]);
			*/ //printf("UTC time: %lf\n index: %d, roll: %lf, pitch: %lf, yaw: %lf\n", utc * 1000000, count, euler[count][0], euler[count][1], euler[count][2]);
			
		}
		CLI_smdat_Xsens->SIM_count = simCnt_imu;
		CLI_smdat_Xsens->euler_x   = euler[ simcnt ][ 0 ];
		CLI_smdat_Xsens->euler_y   = euler[ simcnt ][ 1 ];
		CLI_smdat_Xsens->euler_z   = euler[ simcnt ][ 2 ];
		CLI_smdat_Xsens->acc_x     = acc[ simcnt ][ 0 ];
		CLI_smdat_Xsens->acc_y     = acc[ simcnt ][ 1 ];
		CLI_smdat_Xsens->acc_z     = acc[ simcnt ][ 2 ];
		
		if ( fmod(simcnt, 10) == 0 )
		{
			printf("yaw: %f\n", euler[ simcnt ][ 2 ]);
		}
		
		
		Buf_sim[ simcnt ] = simTime;
		
		while (1)
		{
			curTime = GetWindowTime() * 0.001;   // [s]
			delTime = curTime - iniTime - simTime;
			
			if ( delTime >= Ts )
			{
				break;
			}
		}
		
		simTime = ((double) simcnt + 1.0 ) * Ts;
		simcnt  = simcnt + 1;
		
	} while (simTime < RUNTIME);
	
	
	pFile = fopen("210705_euler_acc1.csv", "wt");
	if ( pFile == NULL )
	{
		//printf("shit..\n");
		return 0;
	}
	for (int r = 0; r < N_data; r++)
	{
		fprintf(pFile, "%.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f\n", (double) Buf_sim[ r ], (double) acc[ r ][ 0 ],
		        (double) acc[ r ][ 1 ], (double) acc[ r ][ 2 ], (double) euler[ r ][ 0 ], (double) euler[ r ][ 1 ],
		        euler[ r ][ 2 ]);// , latlon[r][0], latlon[r][1]);
	}
	
	cout << "\n" << string(79, '-') << "\n";
	cout << "File Saving Complete" << endl;
	
	
	cout << "\n" << string(79, '-') << "\n";
	cout << endl;
	
	cout << "Closing port..." << endl;
	control->closePort(mtPort.portName().toStdString());
	
	cout << "Freeing XsControl object..." << endl;
	control->destruct();
	
	
	if ( pMemoryMap )
		UnmapViewOfFile(pMemoryMap);
	
	if ( hMemoryMap )
		CloseHandle(hMemoryMap);
	return 0;
}

