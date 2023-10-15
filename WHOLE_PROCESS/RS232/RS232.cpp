#include <iostream>

#include "../include/sensor/RS232/RS232.hpp"

#define S (uint8_t)0x53
#define T (uint8_t)0x54
#define X (uint8_t)0x58

#define IDX_AORM 3
#define IDX_ESTOP 4
#define IDX_GEAR 5
#define IDX_SPEED0 6
#define IDX_SPEED1 7
#define IDX_STEER0 8
#define IDX_STEER1 9
#define IDX_BREAK 10
#define IDX_ALIVE 11

#define ETX0 (uint8_t)0x0D
#define ETX1 (uint8_t)0x0A

constexpr const char* COMPORTS[RS232_PORTNR] = {
		R"(\\.\COM1)", R"(\\.\COM2)", R"(\\.\COM3)", R"(\\.\COM4)",
		R"(\\.\COM5)", R"(\\.\COM6)", R"(\\.\COM7)", R"(\\.\COM8)",
		R"(\\.\COM9)", R"(\\.\COM10)", R"(\\.\COM11)", R"(\\.\COM12)",
		R"(\\.\COM13)", R"(\\.\COM14)", R"(\\.\COM15)", R"(\\.\COM16)",
		R"(\\.\COM17)", R"(\\.\COM18)", R"(\\.\COM19)", R"(\\.\COM20)",
		R"(\\.\COM21)", R"(\\.\COM22)", R"(\\.\COM23)", R"(\\.\COM24)",
		R"(\\.\COM25)", R"(\\.\COM26)", R"(\\.\COM27)", R"(\\.\COM28)",
		R"(\\.\COM29)", R"(\\.\COM30)", R"(\\.\COM31)", R"(\\.\COM32)"
};

RS232::RS232(int comport_number) :
		comport_num(comport_number - 1)
{
	data_io_exit        = false;
	check_io_close      = false;
	data_process_exit   = false;
	check_process_close = false;
	
	current_velocity = 0.;
	
	speed  = 0;
	steer0 = 0;
	steer1 = 0;
	
	cmd_gear  = GEAR_FOWARD;
	cmd_break = 1;
	
	write_buffer[ 0 ] = S;
	write_buffer[ 1 ] = T;
	write_buffer[ 2 ] = X;
	
	write_buffer[ IDX_AORM ]  = AOR_M_AUTO;
	write_buffer[ IDX_ESTOP ] = ESTOP_OFF;
	
	write_buffer[ 12 ] = ETX0;
	write_buffer[ 13 ] = ETX1;
	
	for (int i = IDX_GEAR; i <= IDX_ALIVE; i++)
		write_buffer[ i ] = 0;
	
	if (open_comport())
	{
		std::cerr << "Can't open comport!" << std::endl;
		
		return;
	}
}

RS232::RS232(const RS232 &other)
{
	data_io_exit        = other.data_io_exit;
	check_io_close      = other.check_io_close;
	data_process_exit   = other.data_process_exit;
	check_process_close = other.check_process_close;
	
	current_velocity = other.current_velocity;
	
	speed  = other.speed;
	steer0 = other.steer0;
	steer1 = other.steer1;
	
	cmd_gear  = other.cmd_gear;
	cmd_break = other.cmd_break;
	
	for (int i = 0; i < 14; i++)
		write_buffer[ i ] = other.write_buffer[ i ];
	
	cport = other.cport;
	
}

RS232::~RS232()
{
	data_io_exit      = true;
	data_process_exit = true;
	
	std::cout << "ERP42 driver exit" << std::endl;
}

void RS232::run()
{
	std::thread t1(&RS232::data_io, this), t2(&RS232::data_process, this);
	t1.detach();
	t2.detach();
}

int RS232::open_comport()
{
	if (( comport_num >= RS232_PORTNR ) || ( comport_num < 0 ))
	{
		printf("illegal comport number\n");
		return ( 1 );
	}
	
	switch (baud_rate)
	{
		case 110:
			strcpy_s(mode_str, "baud=110");
			break;
		case 300:
			strcpy_s(mode_str, "baud=300");
			break;
		case 600:
			strcpy_s(mode_str, "baud=600");
			break;
		case 1200:
			strcpy_s(mode_str, "baud=1200");
			break;
		case 2400:
			strcpy_s(mode_str, "baud=2400");
			break;
		case 4800:
			strcpy_s(mode_str, "baud=4800");
			break;
		case 9600:
			strcpy_s(mode_str, "baud=9600");
			break;
		case 19200:
			strcpy_s(mode_str, "baud=19200");
			break;
		case 38400:
			strcpy_s(mode_str, "baud=38400");
			break;
		case 57600:
			strcpy_s(mode_str, "baud=57600");
			break;
		case 115200:
			strcpy_s(mode_str, "baud=115200");
			break;
		case 128000:
			strcpy_s(mode_str, "baud=128000");
			break;
		case 256000:
			strcpy_s(mode_str, "baud=256000");
			break;
		case 500000:
			strcpy_s(mode_str, "baud=500000");
			break;
		case 921600:
			strcpy_s(mode_str, "baud=921600");
			break;
		case 1000000:
			strcpy_s(mode_str, "baud=1000000");
			break;
		case 1500000:
			strcpy_s(mode_str, "baud=1500000");
			break;
		case 2000000:
			strcpy_s(mode_str, "baud=2000000");
			break;
		case 3000000:
			strcpy_s(mode_str, "baud=3000000");
			break;
		default:
			printf("invalid baudrate\n");
			return 1;
			break;
	}
	
	if (strlen(mode) != 3)
	{
		printf("invalid mode \"%s\"\n", mode);
		return 1;
	}
	
	switch (mode[ 0 ])
	{
		case '8':
			strcat_s(mode_str, " data=8");
			break;
		case '7':
			strcat_s(mode_str, " data=7");
			break;
		case '6':
			strcat_s(mode_str, " data=6");
			break;
		case '5':
			strcat_s(mode_str, " data=5");
			break;
		default:
			printf("invalid number of data-bits '%c'\n", mode[ 0 ]);
			return 1;
			break;
	}
	
	switch (mode[ 1 ])
	{
		case 'N':
		case 'n':
			strcat_s(mode_str, " parity=n");
			break;
		case 'E':
		case 'e':
			strcat_s(mode_str, " parity=e");
			break;
		case 'O':
		case 'o':
			strcat_s(mode_str, " parity=o");
			break;
		default:
			printf("invalid parity '%c'\n", mode[ 1 ]);
			return 1;
			break;
	}
	
	switch (mode[ 2 ])
	{
		case '1':
			strcat_s(mode_str, " stop=1");
			break;
		case '2':
			strcat_s(mode_str, " stop=2");
			break;
		default:
			printf("invalid number of stop bits '%c'\n", mode[ 2 ]);
			return 1;
			break;
	}
	
	if (flow_ctrl)
	{
		strcat_s(mode_str, " xon=off to=off odsr=off dtr=on rts=off");
	}
	else
	{
		strcat_s(mode_str, " xon=off to=off odsr=off dtr=on rts=on");
	}
	
	/*
	http://msdn.microsoft.com/en-us/library/windows/desktop/aa363145%28v=vs.85%29.aspx

	http://technet.microsoft.com/en-us/library/cc732236.aspx

	https://docs.microsoft.com/en-us/windows/desktop/api/winbase/ns-winbase-_dcb
	*/
	
	cport = CreateFileA(COMPORTS[ comport_num ],
	                    GENERIC_READ | GENERIC_WRITE,
	                    0,                          /* no share  */
	                    nullptr,                       /* no security */
	                    OPEN_EXISTING,
	                    0,                          /* no threads */
	                    nullptr);                      /* no templates */
	
	if (cport == INVALID_HANDLE_VALUE)
	{
		printf("unable to open comport\n");
		return 1;
	}
	
	DCB port_settings;
	memset(&port_settings, 0, sizeof( port_settings ));  /* clear the new struct  */
	port_settings.DCBlength = sizeof( port_settings );
	
	if (!BuildCommDCBA(mode_str, &port_settings))
	{
		printf("unable to set comport dcb settings\n");
		CloseHandle(cport);
		return 1;
	}
	
	if (flow_ctrl)
	{
		port_settings.fOutxCtsFlow = TRUE;
		port_settings.fRtsControl  = RTS_CONTROL_HANDSHAKE;
	}
	
	if (!SetCommState(cport, &port_settings))
	{
		printf("unable to set comport cfg settings\n");
		CloseHandle(cport);
		return 1;
	}
	
	COMMTIMEOUTS Cptimeouts;
	
	Cptimeouts.ReadIntervalTimeout         = MAXDWORD;
	Cptimeouts.ReadTotalTimeoutMultiplier  = 0;
	Cptimeouts.ReadTotalTimeoutConstant    = 0;
	Cptimeouts.WriteTotalTimeoutMultiplier = 0;
	Cptimeouts.WriteTotalTimeoutConstant   = 0;
	
	if (!SetCommTimeouts(cport, &Cptimeouts))
	{
		printf("unable to set comport time-out settings\n");
		CloseHandle(cport);
		
		return 1;
	}
	
	return 0;
}

void RS232::data_io()
{
	int     n;
	uint8_t read_buffer[BUFFER_SIZE];
	
	uint8_t alive = 0;
	
	bool status;
	while (!data_io_exit)
	{
		status = ReadFile(cport,
		                  read_buffer,
		                  BUFFER_SIZE,
		                  (LPDWORD) &n,
		                  nullptr);
		
		if (!status) continue;
		
		std::unique_lock<std::mutex> lock(mutex);
		
		for (int i = 0; i < n; i++)
			packet_data.push(read_buffer[ i ]);
		
		lock.unlock();
		cond_val.notify_one();
		
		write_buffer[ IDX_GEAR ] = cmd_gear;
		
		write_buffer[ IDX_SPEED1 ] = speed;
		
		write_buffer[ IDX_STEER0 ] = steer0;
		write_buffer[ IDX_STEER1 ] = steer1;
		
		write_buffer[ IDX_BREAK ] = cmd_break;
		write_buffer[ IDX_ALIVE ] = alive;
		
		status = WriteFile(cport,
		                   &write_buffer,
		                   sizeof( write_buffer ),
		                   (LPDWORD) ((void*) &n ),
		                   nullptr);
		
		if (!status)
		{
			printf("Fail\n");
			continue;
		}
		
		alive++;
		alive %= 256;
	}
	
	cond_val.notify_one();
	check_io_close = true;
	std::cout << "Close Data I/O" << std::endl;
}

void RS232::data_process()
{
	uint8_t buffer[BIG_ENDIAN_LEN];
	
	int idx_etx0 = BIG_ENDIAN_LEN - 2;
	int idx_etx1 = BIG_ENDIAN_LEN - 1;
	
	uint8_t alive = 0;
	uint8_t data;
	
	while (!data_process_exit)
	{
		std::unique_lock<std::mutex> lock(mutex);
		cond_val.wait(lock, [ this ]
		{ return !packet_data.empty(); });
		
		if (packet_data.size() < BIG_ENDIAN_LEN)
		{
			lock.unlock();
			continue;
		}
		
		data = packet_data.front();
		if (data != S)
		{
			packet_data.pop();
			lock.unlock();
			continue;
		}
		
		for (uint8_t &i: buffer)
		{
			i = packet_data.front();
			packet_data.pop();
		}
		
		lock.unlock();
		
		if (buffer[ 1 ] != T || buffer[ 2 ] != X || buffer[ idx_etx0 ] != ETX0 || buffer[ idx_etx1 ] != ETX1)
			continue;
		
		current_velocity = (double) ( buffer[ 6 ] + buffer[ 7 ] ) / 10.;
	}
	
	check_process_close = true;
	std::cout << "Close Data Processor" << std::endl;
}

double RS232::get_velocity() const
{
	return current_velocity;
}

void RS232::set_velocity(float _velocity)
{
	auto vel = (uint8_t) ( _velocity * 10. );
	
	speed = vel;
}

void RS232::set_angle(float _angle)
{
	auto angle = (int) ( _angle * 71. );
	steer0 = ( angle & 0xff00 ) >> 8;
	steer1 = ( angle & 0xff );
}

void RS232::set_gear(int _gear)
{
	this->cmd_gear = _gear;
}

void RS232::set_break(int _break)
{
	this->cmd_break = _break;
}
