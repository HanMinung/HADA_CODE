#include "../CInterface/velodyne_driver.hpp"

using std::cout;
using std::cerr;
using std::endl;


namespace velodyne
{
	// 생성자 정의
	// 소켓 서버 생성 및 연결
	driver::driver()
		{
#ifdef _WIN64
			/***************** 윈도우일 경우 라이브러리 초기화 ****************/

			WSADATA wsa_data;
			if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
				cerr << "Failed to initialize Winsock." << endl;
				return;
			}
			cout << "Succese : Initalisze Winsock package" << endl;
#endif	//_WIN64


			/***************** UDP 소켓 생성 ****************/

			socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#ifdef _WIN64
			if (socket_ == INVALID_SOCKET)
			{
				cerr << "Failed to create socket." << endl;
				closesocket(socket_);
				WSACleanup();

				return;
			}
#elif __linux__
			if (socket_ == -1)
			{
				std::cerr << "Failed to create socket." << std::endl;
				close(socket_);

				return;
			}
#endif // _WIN64
			cout << "Succese : Create socket" << endl;


			/***************** 소켓 로컬 주소 구성 ****************/

			sockaddr_in local_address;
			local_address.sin_family = AF_INET;				// IPv4로 패밀리 설정
			local_address.sin_addr.s_addr = INADDR_ANY;		// 로컬 주소 자동 탐색
			local_address.sin_port = htons(PORT);			// 센서 포트 번호

			if (bind(socket_, (sockaddr*)&local_address, sizeof(local_address)) == SOCKET_ERROR)
			{
				cerr << "Failed to bind socket." << endl;
				closesocket(socket_);
				WSACleanup();

				return;
			}

			cout << "Succese : Bind socket" << endl;

			driver::SharedMem_CREATE();

			std::thread receive(&driver::data_processing, this);

			receive.detach();		// 데이터 송신 및 처리 시작
			
			return;
		}

	// 소멸자 정의
	// 소켓 폐쇄
	driver::~driver()
	{
		// 소켓 종료
#ifdef _WIN64
		WSACleanup();
		closesocket(socket_);
#elif __linux__
		close(socket_);
#endif // _WIN64

		return;
	}

	// 포인트에서 x, y, z 좌표 산출
	point_cloud driver::calculate_xyz(int azimuth, double distance, int angle)
	{
		point_cloud point;

		double omega = angle * M_PI / 180.;
		double alpha = (double)azimuth * ROTATION_RESOLUTION * M_PI / 180.;

		point.x = DISTANCE_RESOLUTION * distance * cos(omega) * sin(alpha);
		point.y = DISTANCE_RESOLUTION * distance * cos(omega) * cos(alpha);
		point.z = DISTANCE_RESOLUTION * distance * sin(omega);

		return point;
	}

	
	void driver::SharedMem_CREATE(void) {

		TCHAR SERV_NAME[] = TEXT("Lidar_smdat_velodyne");

		hMampF_LASER = NULL;

		hMampF_LASER = CreateFileMapping(
			INVALID_HANDLE_VALUE,
			NULL,
			PAGE_READWRITE,
			0,
			sizeof(driver::point_clouds),
			SERV_NAME);

		if (hMampF_LASER == NULL)
		{
			_tprintf(TEXT("Could not open file mapping object (%d).\n"),
				GetLastError());
			return;
		}

		laser_smdat = (velodyne::point_cloud**)MapViewOfFile(
			hMampF_LASER, 
			FILE_MAP_ALL_ACCESS, 
			0, 
			0, 
			sizeof(driver::point_clouds));

		printf("\n SM created_\n");

	}

	

	// 데이터 수신 및 처리
	void driver::data_processing()
	{
		/***************** 변수 선언 ****************/
		int sequence;				// 시퀸스 반복 횟수
		int azimuth;				// 방위각
		int azimuth_diff;			// 블록당 방위각 변화
		int last_azimuth_diff;		// 마지막으로 측정된 방위각 변화
		int point_cloud_idx = 0;	// Point Cloud 저장 주소

		double distance;			// 물체까지의 변위
		double time;				// 데이터가 추출된 시간
		double start_time = 0;		// 패키징 시작 시간

		char buffer[BUFFER_SIZE];	//데이터 버퍼

		data_packet* packet;		// 데이터 패킷
		data_block* block;			// 데이터 블록

		point point;				// 지점의 데이터 공용체

		uint32_t timestamp;			// 패킷을 전송 받은 시간

		point_cloud temp;			// 데이터 임시값


		/***************** 드라이버 코드 ****************/

		sockaddr_in remote_address;							// 연결된 주소 정보
		int remote_address_size = sizeof(remote_address);	//주소 크기

		while (true)
		{
			/***************** 데이터 수신 ****************/
			int receive_data_size = recvfrom(socket_, buffer, BUFFER_SIZE, 0, (sockaddr*)&remote_address, &remote_address_size);


			/***************** 데이터 처리(버퍼 --> 패킷) ****************/

			if (receive_data_size == 0) continue;	// 데이터가 정상적으로 들어올 때까지 수신 반복

			// 받아온 데이터를 패킷으로 이동
			packet = (data_packet*)buffer;
			assert(packet->factory_field == 0x2237);	// 예외 : 라이다의 값이 지정한 모드와 다른 경우

			timestamp = packet->timestamp;				// 데이터 수신 시간 저장

			azimuth_diff = 0;		// 방위각 변화율 초기화
			last_azimuth_diff = 0;	// 마지막으로 측정된 바위각 변화율 초기화


			/***************** 데이터 처리(패킷 --> 블록) ****************/

			for (int block_idx = 0; block_idx < BLOCK_SIZE; block_idx++)
			{
				// 패킷에서 블럭 추출
				block = &packet->blocks[block_idx];
				assert(block->flag == UPPER_BANK);		// 예외 : 블럭이 상한선을 넘은 경우
					
				azimuth = (int)block->azimuth;			// 해당 블럭의 데이터가 가지고 있는 방위각 저장

				// 다음 블럭의 방위각을 이용하여 방위각 사이 각도 계산
				if (block_idx + 1 < BLOCK_SIZE)
				{
					azimuth_diff = (ROTATION_MAX + packet->blocks[block_idx + 1].azimuth - azimuth) % ROTATION_MAX;		// 방위각 간격 계산
					last_azimuth_diff = azimuth_diff;	// 최신 데이터 업데이트
				}
				else
					azimuth_diff = last_azimuth_diff;	// 새로 산출이 불가능한 경우 가장 최신 데이터 사용

				// 블럭의 데이터를 Firing마다 분석
				for (int firing = 0, k = 0; firing < BLOCK_PER_FIRING; firing++)
				{
					sequence = BLOCK_PER_FIRING * block_idx + firing;	// Firing 횟수 산출


					/***************** 데이터 처리(블록 --> 채널) ****************/

					for (int chanel = 0; chanel < CHANELS; chanel++, k += LASER_DATA_BYTES)
					{
						point.bytes[0] = block->points[k];		// 위치 데이터의 첫번째 숫자 저장 (HEX)
						point.bytes[1] = block->points[k + 1];	// 위치 데이터의 두번째 숫자 저장 (HEX)
						distance = point.distance;				// 위에서 입력받은 데이터를 한번에 저장

						if (distance == 0) continue;	// 거리가 0인 경우(= 사각지대 or 범위 밖) 다음 채널로 통과

						time = (timestamp + (sequence * 55.296 + chanel * 2.304)) / 1000000.0;	// 통신 지연 시간 산출

						// 한바퀴 돌때마다 패키징 다시 시작
						if (time - start_time >= FRAME_CUT)
						{
							memcpy(driver::laser_smdat, driver::point_clouds, sizeof(driver::point_clouds));

							//cout << point_idx << endl;
							point_cloud_idx = 0;	// 패키징 주소 초기화
							start_time = time;		// 패키징 시작 시간 초기화
						}


						/***************** 데이터 처리(데이터(구면) --> 데이터(직교)) ****************/

						temp = calculate_xyz(azimuth, distance, LASER_ANGLE[chanel]);	// 직교좌표계 연산

						// 레이더 사용에 필요없는 부분 생략
						if (temp.y >= 0)
						{
							point_clouds[point_cloud_idx] = temp;
							point_cloud_idx ++;

						}
					}
				}

				// 다음 방위각 연산
				azimuth += azimuth_diff / BLOCK_PER_FIRING;
				azimuth %= ROTATION_MAX;
			}


		}

		return;
	}

}	// namespace sensor_driver
