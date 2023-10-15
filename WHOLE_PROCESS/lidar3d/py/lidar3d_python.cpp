//
// Created by Jarry_Goon on 2023-07-18.
//
#include "lidar3d_python.hpp"

void Velodyne::python::driver::run_py()
{
	Py_BEGIN_ALLOW_THREADS
		
		std::thread receive(&driver::data_processing, this);
		receive.detach();        // 데이터 송신 및 처리 시작
	
	Py_END_ALLOW_THREADS
}

py::list Velodyne::python::driver::get_data_list()
{
	py::list  output;
	py::tuple temp;
	
	for (const auto &point: point_clouds)
	{
		temp = py::make_tuple(point.x, point.y, point.z);
		output.append(temp);
	}
	
	return output;
}

py::object Velodyne::python::driver::get_channel_data(int channel)
{
	std::vector<float> output_vec;
	std::vector<float> y_vec;
	
	int idx = SORTED_LASER_ANGEL_IDX[ channel ];
	int ang = LASER_ANGLE[ idx ];
	
	for (int i = 0; point_clouds[ i ].x != NULL && i < point_clouds.size(); i++)
	{
		double point_ang_tan = point_clouds[ i ].z /
		                       sqrt(point_clouds[ i ].x * point_clouds[ i ].x +
		                            point_clouds[ i ].y * point_clouds[ i ].y);
		int    point_ang     = (int) round(R2D(atan(point_ang_tan)));
		
		if ( ang != point_ang ) continue;
		
		output_vec.push_back(point_clouds[ i ].x);
		y_vec.push_back(point_clouds[ i ].y);
	}
	
	output_vec.insert(output_vec.end(), y_vec.begin(), y_vec.end());
	
	int row_size = (int) ((double) output_vec.size() * 0.5 );
	
	py::object output = np::from_data(output_vec.data(),
	                                  np::dtype::get_builtin<float>(),
	                                  py::make_tuple(2, row_size),
	                                  py::make_tuple(row_size * sizeof(float), sizeof(float)),
	                                  py::object());
	
	return output;
}

double Lidar3D::python::projector::PCD(const py::object &driver_py,
                                       int object_center_u,
                                       int object_center_v,
                                       bool view)
{
	std::shared_ptr<Velodyne::python::driver> driver = py::extract<std::shared_ptr<Velodyne::python::driver>>(
			driver_py);
	
	double dist  = 0.;
	bool   found = false;
	
	int     point_arr[4] = { 0, 0, 0, 1 };
	cv::Mat point(4, 1, CV_32FC1, point_arr);
	cv::Mat pixel_point;
	
	std::array<Velodyne::point_cloud, 20000> PCD       = driver->get_data();
	cv::Mat                                  calib_mat = RT;
	
	cv::Mat project_mat;
	int     pixel_x;
	int     pixel_y;
	
	int temp[2];
	
	
	for (int i = 0; PCD[ i ].x != 0. && PCD[ i ].y != 0. && PCD[ i ].z != 0. && i < 20000; i++)
	{
		point.at<float>(0, 0) = PCD[ i ].x;
		point.at<float>(1, 0) = PCD[ i ].y;
		point.at<float>(2, 0) = PCD[ i ].z;
		
		pixel_point = calib_mat * point;
		pixel_point = pixel_point / pixel_point.at<float>(2, 0);
		
		pixel_point.convertTo(pixel_point, CV_32SC1);
		
		pixel_x = pixel_point.at<int>(0, 0);
		pixel_y = pixel_point.at<int>(1, 0);
		
		if ( !found &&
		     pixel_x < object_center_u + 5 && pixel_x > object_center_u - 5 &&
		     pixel_y < object_center_v + 5 && pixel_y > object_center_v - 5 )
		{
			dist  = sqrt(PCD[ i ].x * PCD[ i ].x + PCD[ i ].y * PCD[ i ].y + PCD[ i ].z * PCD[ i ].z);
			found = true;
			if ( !view ) break;
		}
		
		if ( view )
		{
			temp[ 0 ] = pixel_x;
			temp[ 1 ] = pixel_y;
			
			project_mat.push_back(cv::Mat(1, 2, CV_32SC1, temp));
		}
	}
	
	if ( view )
	{
		auto project_arr = (int*) malloc(project_mat.total() * sizeof(int));
		
		memcpy(project_arr, project_mat.data, project_mat.total() * sizeof(int));
		
		project_data = np::from_data(project_arr,
		                             np::dtype::get_builtin<int>(),
		                             py::make_tuple(project_mat.rows, project_mat.cols),
		                             py::make_tuple(project_mat.cols * sizeof(int), sizeof(int)),
		                             py::object());
	}
	
	return dist;
}

py::object Lidar3D::python::projector::get_projection_data()
{
	return project_data;
}

BOOST_PYTHON_MODULE (lidar3d_py)
{
	np::initialize();
	
	py::class_<Velodyne::point_cloud>("point_cloud")
			.add_property("x", &Velodyne::point_cloud::x)
			.add_property("y", &Velodyne::point_cloud::y)
			.add_property("z", &Velodyne::point_cloud::z);
	
	py::class_<Velodyne::python::driver>("driver")
			.def("run", &Velodyne::python::driver::run_py)
			.def("get_data", &Velodyne::python::driver::get_data_list)
			.def("get_channel_data", &Velodyne::python::driver::get_channel_data);
	
	py::class_<Lidar3D::python::projector>("projector")
			.def("PCD", &Lidar3D::python::projector::PCD)
			.def("get_projection_data", &Lidar3D::python::projector::get_projection_data);
}