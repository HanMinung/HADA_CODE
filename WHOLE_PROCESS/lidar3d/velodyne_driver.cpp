#include "../include/sensor/lidar3d/velodyne_driver.hpp"

namespace Velodyne
{
	driver::driver()
	{
#ifdef _WIN64
		/*----------------- 윈도우일 경우 라이브러리 초기화 -----------------*/
		
		WSADATA wsa_data;   // Windows 소켓 데이터
		if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0)
		{
			printf("ERROR: Failed to initialize Winsock.");
			
			return;
		}
		printf("Succese : Initalisze Winsock package\n");
#endif    //_WIN64
		
		
		/*----------------- UDP 소켓 생성 -----------------*/
		
		socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#ifdef _WIN64
		if (socket_ == INVALID_SOCKET)
		{
			printf("ERROR: Failed to create socket.");
			closesocket(socket_);
			WSACleanup();
			
			return;
		}
#elif __linux__
		if (socket_ == -1)
		{
			printf("ERROR: Failed to create socket.");
			close(socket_);

			return;
		}
#endif // _WIN64
		printf("Succese : Create socket\n");
		
		
		/*----------------- 소켓 로컬 주소 구성 -----------------*/
		
		sockaddr_in local_address{};
		local_address.sin_family      = AF_INET;                // IPv4로 패밀리 설정
		local_address.sin_addr.s_addr = INADDR_ANY;        // 로컬 주소 자동 탐색
		local_address.sin_port        = htons(PORT);            // 센서 포트 번호
		
		if (bind(socket_, (sockaddr*) &local_address, sizeof( local_address )) == SOCKET_ERROR)
		{
			printf("ERROR: Failed to bind socket.");
#ifdef _WIN64
			closesocket(socket_);
			WSACleanup();
#elif __linux__
			close(socket_);
#endif // _WIN64
			
			return;
		}
		
		printf("Succese : Bind socket\n");
	}    // driver::driver
	
	driver::~driver()
	{
		// 프로세스 종료
		running = false;
		
		while (!exit_state);
		
		// 소켓 종료
#ifdef _WIN64
		WSACleanup();
		closesocket(socket_);
#elif __linux__
		close(socket_);
#endif // _WIN64
		
		printf("Exit Driver.\n");
	}    // driver::~driver
	
	void driver::run()
	{
		std::thread receive(&driver::data_processing, this);
		
		receive.detach();        // 데이터 송신 및 처리 시작
	}
	
	std::array<point_cloud, 20000> driver::get_data() const
	{
		return point_clouds;
	}
	
	std::vector<double> driver::get_channel_data(int channel) const
	{
		std::vector<double> output;
		
		int idx = SORTED_LASER_ANGEL_IDX[ channel ];
		
		for (int i = channel; point_clouds[ i ].x != NULL; i += 16)
		{
			output.push_back(point_clouds[ i ].x);
			output.push_back(point_clouds[ i ].y);
		}
		
		return output;
	}
	
	point_cloud driver::calculate_xyz(const int azimuth, const float distance, const int angle)
	{
		point_cloud point{};
		
		float omega = (float) angle * num::pi / 180.;
		float alpha = (float) azimuth * ROTATION_RESOLUTION * num::pi / 180.;
		
		point.x = DISTANCE_RESOLUTION * distance * cosf(omega) * sinf(alpha);
		point.y = DISTANCE_RESOLUTION * distance * cosf(omega) * cosf(alpha);
		point.z = DISTANCE_RESOLUTION * distance * sinf(omega);
		
		return point;
	}    // point_cloud driver::calculate_xyz
	
	void driver::data_processing()
	{
		int sequence;                // 시퀸스 반복 횟수
		int azimuth;                // 방위각
		int azimuth_diff;            // 블록당 방위각 변화
		int last_azimuth_diff;        // 마지막으로 측정된 방위각 변화
		int point_cloud_idx = 0;    // Point Cloud 저장 주소
		
		float  distance;            // 물체까지의 변위
		double time;                // 데이터가 추출된 시간
		double start_time = 0;        // 패키징 시작 시간
		
		char buffer[BUFFER_SIZE];    //데이터 버퍼
		
		data_packet* packet;        // 데이터 패킷
		data_block * block;            // 데이터 블록
		
		point point{};                // 지점의 데이터 공용체
		
		uint32_t timestamp;            // 패킷을 전송 받은 시간
		
		point_cloud temp{};            // 데이터 임시값
		
		
		/*----------------- 드라이버 코드 -----------------*/
		
		sockaddr_in remote_address{};                            // 연결된 주소 정보
#ifdef _WIN64
		int remote_address_size = sizeof( remote_address );    //주소 크기
#elif __linux__
		socklen_t remote_address_size = sizeof( remote_address );    //주소 크기
#endif // _WIN64
		
		while (running)
		{
			/*----------------- 데이터 수신 -----------------*/
			
			// 받아오는 데이터 형태 및 테이터 전체 크기
			auto receive_data_size = recvfrom(socket_,
			                                  buffer,
			                                  BUFFER_SIZE,
			                                  0,
			                                  (sockaddr*) &remote_address,
			                                  &remote_address_size);
			
			
			/*----------------- 데이터 처리(버퍼 --> 패킷) -----------------*/
			
			if (receive_data_size == 0) continue;    // 데이터가 정상적으로 들어올 때까지 수신 반복
			
			// 받아온 데이터를 패킷으로 이동
			packet = (data_packet*) buffer;
			assert(packet->factory_field == 0x2237);    // 예외 : 라이다의 값이 지정한 모드와 다른 경우
			
			timestamp = packet->timestamp;                // 데이터 수신 시간 저장
			
			last_azimuth_diff = 0;    // 마지막으로 측정된 바위각 변화율 초기화
			
			
			/*----------------- 데이터 처리(패킷 --> 블록) -----------------*/
			
			for (int block_idx = 0; block_idx < BLOCK_SIZE && running; block_idx++)
			{
				// 패킷에서 블럭 추출
				block = &packet->blocks[ block_idx ];
				assert(block->flag == UPPER_BANK);        // 예외 : 블럭이 상한선을 넘은 경우
				
				azimuth = (int) block->azimuth;            // 해당 블럭의 데이터가 가지고 있는 방위각 저장
				
				// 다음 블럭의 방위각을 이용하여 방위각 사이 각도 계산
				if (block_idx + 1 < BLOCK_SIZE)
				{
					azimuth_diff      = ( ROTATION_MAX + packet->blocks[ block_idx + 1 ].azimuth - azimuth ) %
					                    ROTATION_MAX;    // 방위각 간격 계산
					last_azimuth_diff = azimuth_diff;    // 최신 데이터 업데이트
				}
				else
					azimuth_diff = last_azimuth_diff;    // 새로 산출이 불가능한 경우 가장 최신 데이터 사용
				
				// 블럭의 데이터를 Firing마다 분석
				for (int firing = 0, k = 0; firing < BLOCK_PER_FIRING && running; firing++)
				{
					sequence = BLOCK_PER_FIRING * block_idx + firing;    // Firing 횟수 산출
					
					
					/*----------------- 데이터 처리(블록 --> 채널) -----------------*/
					
					for (int chanel = 0; chanel < CHANELS && running; chanel++, k += LASER_DATA_BYTES)
					{
						point.bytes[ 0 ] = block->points[ k ];      // 위치 데이터의 첫번째 숫자 저장 (HEX)
						point.bytes[ 1 ] = block->points[ k + 1 ];  // 위치 데이터의 두번째 숫자 저장 (HEX)
						distance = point.distance;                  // 위에서 입력받은 데이터를 한번에 저장
						
						if (distance == 0) continue;    // 거리가 0인 경우(= 사각지대 or 범위 밖) 다음 채널로 통과
						
						time = ( timestamp + ( sequence * 55.296 + chanel * 2.304 )) / 1000000.0;    // 통신 지연 시간 산출
						
						// 한바퀴 돌때마다 패키징 다시 시작
						if (time - start_time >= FRAME_CUT)
						{
							//cout << point_idx << endl;
							point_cloud_idx = 0;    // 패키징 주소 초기화
							start_time      = time;        // 패키징 시작 시간 초기화
						}
						
						
						/*----------------- 데이터 처리(데이터(구면) --> 데이터(직교)) -----------------*/
						
						temp = calculate_xyz(azimuth, distance, LASER_ANGLE[ chanel ]);    // 직교좌표계 연산
						
						// 레이더 사용에 필요없는 부분 생략
						if (temp.y >= 0)
						{
							point_clouds[ point_cloud_idx ] = temp;
							point_cloud_idx++;
						}
					}
				}
				
				// 다음 방위각 연산
				azimuth += azimuth_diff / BLOCK_PER_FIRING;
				azimuth %= ROTATION_MAX;
			}
		}
		
		exit_state = true;
	}    // void driver::data_processing
	
}    // namespace sensor_driver
