//
// Created by Jarry_Goon on 2023-10-02.
//

#ifndef HADA_RS232_PYTHON_HPP
#define HADA_RS232_PYTHON_HPP

#include "../../include/sensor/RS232/RS232.hpp"

class RS232_py : public RS232
{
public:
	RS232_py(int comport_number);
	
	void run_py();
};

#endif //HADA_RS232_PYTHON_HPP
