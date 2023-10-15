//
// Created by Jarry_Goon on 2023-07-18.
//

#ifndef HADA_LIDAR3D_PYTHON_HPP
#define HADA_LIDAR3D_PYTHON_HPP

#include "../../Camera/py/camera_python.hpp"
#include "../../include/sensor/lidar3d/velodyne_driver.hpp"
#include "../../include/sensor/lidar3d/projector.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

namespace py = boost::python;
namespace np = boost::python::numpy;

namespace Velodyne::python
{
	class driver : public Velodyne::driver
	{
	public:
		void run_py();
		
		py::list get_data_list();
		
		py::object get_channel_data(int channel = 7);
	};
}


namespace Lidar3D::python
{
	class projector : public Lidar3D::projector
	{
	public:
		double PCD(const py::object &driver_py,
		           int object_center_u,
		           int object_center_v,
		           bool view = false);
		
		py::object get_projection_data();
	
	private:
		py::object project_data;
	};
}


#endif //HADA_LIDAR3D_PYTHON_HPP
