#ifndef LIDAR3D_HPP
#define LIDAR3D_HPP

#include <iostream>
#include <cstdio>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "../camera/camera.hpp"
#include "velodyne_driver.hpp"

namespace Lidar3D
{
	class projector
	{
	public:
		projector();
		
		double PCD(Velodyne::driver* driver,
		           int object_center_u,
		           int object_center_v,
		           bool view) const;
		
		cv::Mat RT;
	};
}
#endif // !LIDAR3D_HPP

