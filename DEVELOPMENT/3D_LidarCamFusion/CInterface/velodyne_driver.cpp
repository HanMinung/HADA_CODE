#include "../CInterface/velodyne_driver.hpp"

using std::cout;
using std::cerr;
using std::endl;


namespace velodyne
{
	// ������ ����
	// ���� ���� ���� �� ����
	driver::driver()
		{
#ifdef _WIN64
			/***************** �������� ��� ���̺귯�� �ʱ�ȭ ****************/

			WSADATA wsa_data;
			if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
				cerr << "Failed to initialize Winsock." << endl;
				return;
			}
			cout << "Succese : Initalisze Winsock package" << endl;
#endif	//_WIN64


			/***************** UDP ���� ���� ****************/

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


			/***************** ���� ���� �ּ� ���� ****************/

			sockaddr_in local_address;
			local_address.sin_family = AF_INET;				// IPv4�� �йи� ����
			local_address.sin_addr.s_addr = INADDR_ANY;		// ���� �ּ� �ڵ� Ž��
			local_address.sin_port = htons(PORT);			// ���� ��Ʈ ��ȣ

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

			receive.detach();		// ������ �۽� �� ó�� ����
			
			return;
		}

	// �Ҹ��� ����
	// ���� ���
	driver::~driver()
	{
		// ���� ����
#ifdef _WIN64
		WSACleanup();
		closesocket(socket_);
#elif __linux__
		close(socket_);
#endif // _WIN64

		return;
	}

	// ����Ʈ���� x, y, z ��ǥ ����
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

	

	// ������ ���� �� ó��
	void driver::data_processing()
	{
		/***************** ���� ���� ****************/
		int sequence;				// ������ �ݺ� Ƚ��
		int azimuth;				// ������
		int azimuth_diff;			// ��ϴ� ������ ��ȭ
		int last_azimuth_diff;		// ���������� ������ ������ ��ȭ
		int point_cloud_idx = 0;	// Point Cloud ���� �ּ�

		double distance;			// ��ü������ ����
		double time;				// �����Ͱ� ����� �ð�
		double start_time = 0;		// ��Ű¡ ���� �ð�

		char buffer[BUFFER_SIZE];	//������ ����

		data_packet* packet;		// ������ ��Ŷ
		data_block* block;			// ������ ���

		point point;				// ������ ������ ����ü

		uint32_t timestamp;			// ��Ŷ�� ���� ���� �ð�

		point_cloud temp;			// ������ �ӽð�


		/***************** ����̹� �ڵ� ****************/

		sockaddr_in remote_address;							// ����� �ּ� ����
		int remote_address_size = sizeof(remote_address);	//�ּ� ũ��

		while (true)
		{
			/***************** ������ ���� ****************/
			int receive_data_size = recvfrom(socket_, buffer, BUFFER_SIZE, 0, (sockaddr*)&remote_address, &remote_address_size);


			/***************** ������ ó��(���� --> ��Ŷ) ****************/

			if (receive_data_size == 0) continue;	// �����Ͱ� ���������� ���� ������ ���� �ݺ�

			// �޾ƿ� �����͸� ��Ŷ���� �̵�
			packet = (data_packet*)buffer;
			assert(packet->factory_field == 0x2237);	// ���� : ���̴��� ���� ������ ���� �ٸ� ���

			timestamp = packet->timestamp;				// ������ ���� �ð� ����

			azimuth_diff = 0;		// ������ ��ȭ�� �ʱ�ȭ
			last_azimuth_diff = 0;	// ���������� ������ ������ ��ȭ�� �ʱ�ȭ


			/***************** ������ ó��(��Ŷ --> ���) ****************/

			for (int block_idx = 0; block_idx < BLOCK_SIZE; block_idx++)
			{
				// ��Ŷ���� �� ����
				block = &packet->blocks[block_idx];
				assert(block->flag == UPPER_BANK);		// ���� : ���� ���Ѽ��� ���� ���
					
				azimuth = (int)block->azimuth;			// �ش� ���� �����Ͱ� ������ �ִ� ������ ����

				// ���� ���� �������� �̿��Ͽ� ������ ���� ���� ���
				if (block_idx + 1 < BLOCK_SIZE)
				{
					azimuth_diff = (ROTATION_MAX + packet->blocks[block_idx + 1].azimuth - azimuth) % ROTATION_MAX;		// ������ ���� ���
					last_azimuth_diff = azimuth_diff;	// �ֽ� ������ ������Ʈ
				}
				else
					azimuth_diff = last_azimuth_diff;	// ���� ������ �Ұ����� ��� ���� �ֽ� ������ ���

				// ���� �����͸� Firing���� �м�
				for (int firing = 0, k = 0; firing < BLOCK_PER_FIRING; firing++)
				{
					sequence = BLOCK_PER_FIRING * block_idx + firing;	// Firing Ƚ�� ����


					/***************** ������ ó��(��� --> ä��) ****************/

					for (int chanel = 0; chanel < CHANELS; chanel++, k += LASER_DATA_BYTES)
					{
						point.bytes[0] = block->points[k];		// ��ġ �������� ù��° ���� ���� (HEX)
						point.bytes[1] = block->points[k + 1];	// ��ġ �������� �ι�° ���� ���� (HEX)
						distance = point.distance;				// ������ �Է¹��� �����͸� �ѹ��� ����

						if (distance == 0) continue;	// �Ÿ��� 0�� ���(= �簢���� or ���� ��) ���� ä�η� ���

						time = (timestamp + (sequence * 55.296 + chanel * 2.304)) / 1000000.0;	// ��� ���� �ð� ����

						// �ѹ��� �������� ��Ű¡ �ٽ� ����
						if (time - start_time >= FRAME_CUT)
						{
							memcpy(driver::laser_smdat, driver::point_clouds, sizeof(driver::point_clouds));

							//cout << point_idx << endl;
							point_cloud_idx = 0;	// ��Ű¡ �ּ� �ʱ�ȭ
							start_time = time;		// ��Ű¡ ���� �ð� �ʱ�ȭ
						}


						/***************** ������ ó��(������(����) --> ������(����)) ****************/

						temp = calculate_xyz(azimuth, distance, LASER_ANGLE[chanel]);	// ������ǥ�� ����

						// ���̴� ��뿡 �ʿ���� �κ� ����
						if (temp.y >= 0)
						{
							point_clouds[point_cloud_idx] = temp;
							point_cloud_idx ++;

						}
					}
				}

				// ���� ������ ����
				azimuth += azimuth_diff / BLOCK_PER_FIRING;
				azimuth %= ROTATION_MAX;
			}


		}

		return;
	}

}	// namespace sensor_driver
