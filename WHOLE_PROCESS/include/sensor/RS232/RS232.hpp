//
// Created by Jarry_Goon on 2023-10-02.
//

#ifndef HADA_RS232_HPP
#define HADA_RS232_HPP

#include <queue>
#include <mutex>
#include <condition_variable>

#include <windows.h>

#define RS232_PORTNR 32
#define BIG_ENDIAN_LEN 18
#define LITTLE_ENDIAN_LEN 14
#define BUFFER_SIZE 1024

#define AOR_M_MANUAL (uint8_t)0x00
#define AOR_M_AUTO (uint8_t)0x01

#define ESTOP_ON (uint8_t)0x01
#define ESTOP_OFF (uint8_t)0x00

#define GEAR_FOWARD (uint8_t)0x00
#define GEAR_NEUTRAL (uint8_t)0x01
#define GEAR_BACKWARD (uint8_t)0x02

class RS232
{
public:
	RS232(int comport_number);
	
	RS232(const RS232 &other);
	
	~RS232();
	
	void run();
	
	double get_velocity() const;
	
	void set_velocity(float _velocity);
	
	void set_angle(float _angle);
	
	void set_gear(int _gear);
	
	void set_break(int _break);

protected:
	void data_io();
	
	void data_process();

private:
	int        comport_num;
	int        baud_rate = 115200;
	const char mode[4]   = { '8', 'N', '1', 0 };
	bool       flow_ctrl = false;
	
	bool data_io_exit;
	bool check_io_close;
	bool data_process_exit;
	bool check_process_close;
	
	char   mode_str[128];
	HANDLE cport;
	
	uint8_t cmd_gear;
	uint8_t cmd_break;
	
	uint8_t write_buffer[LITTLE_ENDIAN_LEN];
	
	uint8_t speed;
	uint8_t steer0;
	uint8_t steer1;
	
	std::queue<uint8_t>     packet_data;
	std::mutex              mutex;
	std::condition_variable cond_val;
	
	double current_velocity;
	
	int open_comport();
};

#endif //HADA_RS232_HPP
