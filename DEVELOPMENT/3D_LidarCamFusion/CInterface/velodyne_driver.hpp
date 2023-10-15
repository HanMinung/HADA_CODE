// Velodyne Lidar에서 데이터 수집
#ifndef __VELODYNE_DRIVER__
#define __VELODYNE_DRIVER__

// 기본 헤더
#include <iostream>
#include <functional>
#include <ctime>
#include <cassert>
#define _USE_MATH_DEFINES
#include <math.h>

// Socket 연결 헤더
#ifdef _WIN64
#include <WinSock2.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/shm.h>
#include <sys/ipc.h>
// Linux socket 관련 헤더
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
// Linux processer 실행 헤더
#include <unistd.h>
#endif // _WIN64
#include <fcntl.h>

// 멀티 쓰레드 접근 헤더
#include <thread>
// #include <mutex>

// Queue 통신
#include <queue>
#include <tuple>


// 복사한 항목
#include    <stdio.h>
#include    <stdlib.h>
#include    <math.h>
#include    <string.h>
#include    <memory.h>
#include    <conio.h>
#include    <time.h>
#include    <Windows.h>
#include    <tchar.h>


// 상수 선언
static const int BLOCK_PER_FIRING = 2;				// 블록당 Firing 개수
static const int LASER_DATA_BYTES = 3;				// 레이저 데이터가 차지하는 크기

static const int BUFFER_SIZE = 1206;				// 데이터 버퍼 크기
static const u_short PORT = 2368;					// 라이다 데이터를 받아올 포트

static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

static const INT8 CHANELS = 16;						// 라이다 채널
static const INT8 BLOCK_SIZE = 12;					// 데이터 셋 행 크기
static const INT8 POINT_SIZE = 96;					// 데이터 셋 열 크기

static const double FRAME_CUT = 0.1;				// 데이터 분할 시간 지점 [s][µs]

static const UINT16 ROTATION_MAX = 36000;			// 각도 최대값
static const double DISTANCE_RESOLUTION = 0.002;	// 변위에 대한 해상도(1 / 500)
static const double ROTATION_RESOLUTION = 0.01;		// 각도에 대한 해상도 (1 / 100)
// 레이저 ID에 따른 부앙각 정보
static const int LASER_ANGLE[] = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };

namespace velodyne
{
	// X, Y, Z값을 가진 포인트 클라우드
	typedef struct
	{
		double x;
		double y;
		double z;
	} point_cloud;
	
	// 센서에서 입력받은 데이터 처리
	class driver
	{
	public:

		HANDLE hMampF_LASER;
		point_cloud** laser_smdat;

		// 생성자 정의
		// 소캣 서버 생성 및 데이터 수신, 처리
		driver();

		// 소멸자 정의
		// 드라이버 종료
		~driver();

		//point_cloud 
		point_cloud point_clouds[16000] = {};
		
		void SharedMem_CREATE(void);

	private:
		// Queue에 들어갈 데이터 튜플 형태
		typedef std::tuple<double, int, int, double> data_tuple;

		// 물체까지 거리
		typedef union
		{
			uint16_t distance;		// 물체까지 변위
			uint8_t bytes[2];		// 바이트별 숫자 크기 (16진법)
		} point;

		// 데이터 블럭: 방위각, 플래그, 데이터 포인트
		typedef struct
		{
			uint16_t flag;					// 데이터 플래그
			uint16_t azimuth;				// 방위각
			uint8_t points[POINT_SIZE];		// 거리 데이터 및 반사율
		} data_block;

		// 데이터 패킷: 데이터 블록, 송신 시간 포함
		typedef struct
		{
			data_block blocks[BLOCK_SIZE];	// 포인트 블럭
			uint32_t timestamp;				// 송신한 시간
			uint16_t factory_field;			// 모델명, 실질적으로 사용 X
		} data_packet;

		// 데이터 송신 종료를 위한 변수
		bool exit = false;

		// 받은 데이터를 임의로 저장할 Queue <전송된 시간, 방위각, 부앙각, 거리>
		std::queue<data_tuple> q;

		// 쓰레드 데이터 제어를 위한 Mutex 선언
		// std::mutex m;

		// 소켓 변수
#ifdef _WIN64
		SOCKET socket_;
#elif __linux__
		int socket_;
#endif // _WIN64

		// 포인트에서 x, y, z 좌표 산출
		point_cloud calculate_xyz(int azimuth, double distance, int angle);

		// 데이터 수신 및 처리
		void data_processing();

	};	// class driver
}	// namespace sensor_driver

#endif // !__VELODYNE_DRIVER__