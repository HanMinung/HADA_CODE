// Velodyne Lidar에서 데이터 수집
#ifndef VELODYNE_DRIVER_HPP
#define VELODYNE_DRIVER_HPP

// 기본 헤더
#include <iostream>
#include <cstdio>
#include <functional>
#include <ctime>
#include <cassert>
#include <cmath>
#include <vector>
#include <array>
#include <numbers>

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

// 멀티 쓰레드 제어 헤더
#include <thread>

constexpr int BLOCK_PER_FIRING = 2;                // 블록당 Firing 개수
constexpr int LASER_DATA_BYTES = 3;                // 레이저 데이터가 차지하는 크기

constexpr int     BUFFER_SIZE = 1206;                // 데이터 버퍼 크기
constexpr u_short PORT        = 2368;                    // 라이다 데이터를 받아올 포트

constexpr uint16_t UPPER_BANK = 0xeeff;

constexpr int8_t CHANELS    = 16;                    // 라이다 채널
constexpr int8_t BLOCK_SIZE = 12;                    // 데이터 셋 행 크기
constexpr int8_t POINT_SIZE = 96;                    // 데이터 셋 열 크기

constexpr double FRAME_CUT = 0.1;                // 데이터 분할 시간 지점 [s][µs]

constexpr uint16_t ROTATION_MAX               = 36000;            // 각도 최대값
constexpr float    DISTANCE_RESOLUTION        = 0.002;    // 변위에 대한 해상도(1 / 500)
constexpr float    ROTATION_RESOLUTION        = 0.01;    // 각도에 대한 해상도 (1 / 100)
// 레이저 ID에 따른 부앙각 정보
constexpr int      LASER_ANGLE[16]            = {
		-15, 1, -13, 3, -11, 5, -9, 7,
		-7, 9, -5, 11, -3, 13, -1, 15
};
constexpr int      SORTED_LASER_ANGEL_IDX[16] = {
		15, 13, 11, 9, 7, 5, 3, 1,
		14, 12, 10, 8, 6, 4, 2, 0
};

namespace num = std::numbers;

namespace Velodyne
{
	/**
	 * @struct point_cloud
	 * @brief x, y, z 데이터 한쌍을 담는 구조체
	 *
	 * @var x: x 좌표
	 * @var y: y 좌표
	 * @var z: z 좌표
	 */
	struct point_cloud
	{
		point_cloud() = default;
		
		point_cloud(float x, float y, float z)
				: x(x), y(y), z(z)
		{
		}
		
		float x;
		float y;
		float z;
	};
	
	/**
	 * velodyne 센서에서 데이터를 받아오기 위해 필요한 클래스
	 *
	 * @warning 전체 코드에서 딱 하나의 인스턴스만 존재해야 함
	 */
	class driver
	{
	public:
		/** 소켓 서버 생성 및 연결 */
		driver();
		
		/** 드라이버 종료 */
		~driver();
		
		/** 데이터 수신, 처리 함수 실행 */
		void run();
		
		/** 저장된 클라우드 데이터 호출 */
		std::array<point_cloud, 20000> get_data() const;
		
		std::vector<double> get_channel_data(int channel = 7) const;
	
	protected:
		/** 3D 라이다에 감지된 포인트들 */
		std::array<point_cloud, 20000> point_clouds{};
		
		/** 물체까지 거리 */
		union point
		{
			uint16_t distance;      // 물체까지 변위
			uint8_t  bytes[2];       // 바이트별 숫자 크기 (16진법)
		};
		
		/** 데이터 블럭: 방위각, 플래그, 데이터 포인트 */
		struct data_block
		{
			uint16_t flag;                  // 데이터 플래그
			uint16_t azimuth;               // 방위각
			uint8_t  points[POINT_SIZE];     // 거리 데이터 및 반사율
		};
		
		/** 데이터 패킷: 데이터 블록, 송신 시간 포함 */
		struct data_packet
		{
			data_block blocks[BLOCK_SIZE];  // 포인트 블럭
			uint32_t   timestamp;             // 송신한 시간
			uint16_t   factory_field;         // 모델명, 실질적으로 사용 X
		};
		
		bool running    = true;        // 드라이버 가동 여부
		bool exit_state = false;    // 드라이버 종료 여부
		
		// 소켓 변수
#ifdef _WIN64
		SOCKET socket_ = 0;
#elif __linux__
		int socket_;
#endif // _WIN64
		
		/**
		* 구면좌표계로 표현된 라이다 데이터를 직교 좌표계로 변환
		* @param azimuth: 방위각[degree * 100]
		* @param distance: 물체까지의 거리[m]
		* @param angle: 고도각[degree * 10]
		* @return 직교좌표계(x, y, z) 데이터
		*/
		static point_cloud calculate_xyz(int azimuth, float distance, int angle);
		
		/**
		* @brief 3D 라이다로부터 데이터 수신 및 가공
		*
		* 수신한 데이터를 구면 좌표계에서 직교 좌표계로 변환한다.
		*/
		void data_processing();
	};    // class driver
	
}    // namespace sensor_driver

#endif // !VELODYNE_DRIVER_HPP