//
// Created by Jarry_Goon on 2023-07-17.
//

#include "camera_python.hpp"


np::ndarray camera_python::bev(const np::ndarray &frame)
{
	cv::Mat frame_mat((int) frame.shape(0), (int) frame.shape(1), CV_8UC3, (uchar*) frame.get_data());
	
	cv::Mat dist_coef_arr(1, 5, CV_32F, &dist_coef);
	
	cv::Mat frame_undistorted(frame_mat.rows, frame_mat.cols, frame_mat.type());
	
	cv::Mat new_camera_mat = cv::getOptimalNewCameraMatrix(K, dist_coef_arr, cv::Size(frame_mat.cols, frame_mat.rows),
	                                                       1);
	cv::undistort(frame_mat, frame_undistorted, K, dist_coef_arr, new_camera_mat);
	
	cv::Mat dest_mat(output_height, output_width, frame_mat.type());
	
	float   point1_values[4] = { world_x_min, world_y_max, 0., 1. };
	cv::Mat point1(4, 1, CV_32F, point1_values);
	
	float   point2_values[4] = { world_x_max, world_y_max, 0., 1. };
	cv::Mat point2(4, 1, CV_32F, point2_values);
	
	float   point3_values[4] = { world_x_max, world_y_min, 0., 1. };
	cv::Mat point3(4, 1, CV_32F, point3_values);
	
	float   point4_values[4] = { world_x_min, world_y_min, 0., 1. };
	cv::Mat point4(4, 1, CV_32F, point4_values);
	
	cv::Mat uv1 = KRT * point1;
	uv1 /= uv1.at<float>(2, 0);
	cv::Mat uv2 = KRT * point2;
	uv2 /= uv2.at<float>(2, 0);
	cv::Mat uv3 = KRT * point3;
	uv3 /= uv3.at<float>(2, 0);
	cv::Mat uv4 = KRT * point4;
	uv4 /= uv4.at<float>(2, 0);
	
	std::array<cv::Point2f, 4> pts_src = {
			cv::Point(uv1.at<float>(0, 0), uv1.at<float>(1, 0)),
			cv::Point(uv2.at<float>(0, 0), uv2.at<float>(1, 0)),
			cv::Point(uv3.at<float>(0, 0), uv3.at<float>(1, 0)),
			cv::Point(uv4.at<float>(0, 0), uv4.at<float>(1, 0))
	};
	
	std::array<cv::Point2f, 4> pts_dst = {
			cv::Point(0, 0),
			cv::Point(output_width - 1, 0),
			cv::Point(output_width - 1, output_height - 1),
			cv::Point(0, output_height - 1)
	};
	
	cv::Mat h = cv::getPerspectiveTransform(pts_src, pts_dst);
	cv::warpPerspective(frame_mat, dest_mat, h, cv::Size(output_width, output_height));
	
	auto* dest_arr = (uchar*) malloc(output_height * output_width * 3);
	
	memcpy(dest_arr, dest_mat.data, output_height * output_width * 3);
	
	np::ndarray dest = np::from_data(dest_arr,
	                                 np::dtype::get_builtin<uchar>(),
	                                 py::make_tuple(output_height, output_width, 3),
	                                 py::make_tuple(output_width * 3, 3, 1),
	                                 py::object());
	
	return dest;
}

BOOST_PYTHON_MODULE (camera_py)
{
	np::initialize();
	
	py::class_<camera_python>("camera",
	                          "Parameters"
	                          "    height: Height to ground [m]"
	                          "    x_min: Minimum x-coordinate relative to the ground under the camera [m]"
	                          "    x_max: Maximum x-coordinate relative to the ground under the camera [m]"
	                          "    y_min: Minimum y-coordinate relative to the ground below the camera [m]"
	                          "    y_max: Maximum y-coordinate relative to the ground under the camera [m]"
	                          "    cam_param_file: Camera parameter file path"
	                          "    pan: Left and right rotation angle [degree]"
	                          "    tilt: Vertical rotation angle [degree]"
	                          "    roll: View direction rotation angle [degree]",
	                          py::init<float, float, float, float, float, py::str, float, float, float>())
			.def("bev", &camera_python::bev);
}