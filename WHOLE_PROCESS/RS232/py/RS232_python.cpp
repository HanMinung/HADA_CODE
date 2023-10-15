//
// Created by Jarry_Goon on 2023-10-02.
//

#include <boost/python.hpp>

#include "RS232_python.hpp"

namespace py = boost::python;

RS232_py::RS232_py(int comport_number) : RS232(comport_number)
{}

void RS232_py::run_py()
{
	Py_BEGIN_ALLOW_THREADS
		std::thread t1(&RS232_py::data_io, this), t2(&RS232_py::data_process, this);
		t1.detach();
		t2.detach();
	Py_END_ALLOW_THREADS
}

BOOST_PYTHON_MODULE (RS232_py)
{
	py::class_<RS232_py>("RS232",
	                     "Parameters"
	                     "    comport_number: RS232 Port number",
	                     py::init<int>())
			.def("run", &RS232_py::run_py)
			.def("get_velocity", &RS232_py::get_velocity)
			.def("set_velocity", &RS232_py::set_velocity)
			.def("set_angle", &RS232_py::set_angle)
			.def("set_break", &RS232_py::set_break)
			.def("set_gear", &RS232_py::set_gear);
}
