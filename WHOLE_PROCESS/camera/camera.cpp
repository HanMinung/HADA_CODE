#include "../include/sensor/camera/camera.hpp"

/**
 * @class camera
 * @brief 특정 사양과 속성의 카메라를 표현
 *
 * 카메라 클래스는 높이, x 및 y 경계, 카메라 파일,
 * 팬, 틸트, 롤 각도를 가진 카메라를 표현하는 기능을 제공
 * 다양한 목적으로 카메라를 조작하고 제어하는데 사용될 수 있음
 */
camera::camera(float height, float x_min, float x_max, float y_min, float y_max,
               const std::string &cam_file, float pan, float tilt, float roll)
{
	this->height = height;
	
	cos_pan  = cosf(D2R(pan));
	sin_pan  = sinf(D2R(pan));
	cos_tilt = cosf(D2R(tilt - 90.));
	sin_tilt = sinf(D2R(tilt - 90.));
	cos_roll = cosf(D2R(roll));
	sin_roll = sinf(D2R(roll));
	
	world_x_max = x_max;
	world_x_min = x_min;
	world_y_max = y_max;
	world_y_min = y_min;
	
	output_height = (int) roundf(( y_max - y_min ) * 10.f) * 20;
	output_width  = (int) roundf(( x_max - x_min ) * 10.f) * 20;
	
	KRT = calibration(cam_file);
}

camera::~camera()
{
	KRT.release();
	K.release();
}

cv::Mat camera::get_calibration_mat()
{
	return KRT;
}

/**
 * @brief 입력 프레임에 bird's eye view 변환 적용.
 *
 * 이 함수는 입력 프레임을 인수로 받아 bird's eye view 변환을 적용
 * bird's eye view 변환은 입력 프레임을 위에서 아래로 보는 관점으로 변환하는 perspective 변환
 * 공중에서 바라보는 것처럼 보이게 이미지를 변환
 *
 * @param frame: 변환할 입력 프레임
 * @return bird's eye view: 관점으로 변환된 프레임
 */
cv::Mat camera::bev(const cv::Mat &frame)
{
	cv::Mat dist_coef_arr(1, 5, CV_32F, &dist_coef);
	
	cv::Mat frame_undistorted(frame.rows, frame.cols, frame.type());
	
	cv::Mat new_camera_mat = cv::getOptimalNewCameraMatrix(K, dist_coef_arr, cv::Size(frame.cols, frame.rows), 1);
	cv::undistort(frame, frame_undistorted, K, dist_coef_arr, new_camera_mat);
	
	
	cv::Mat dest(output_height, output_width, frame.type());
	
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
			cv::Point2f(uv1.at<float>(0, 0), uv1.at<float>(1, 0)),
			cv::Point2f(uv2.at<float>(0, 0), uv2.at<float>(1, 0)),
			cv::Point2f(uv3.at<float>(0, 0), uv3.at<float>(1, 0)),
			cv::Point2f(uv4.at<float>(0, 0), uv4.at<float>(1, 0))
	};
	
	std::array<cv::Point2f, 4> pts_dst = {
			cv::Point(0, 0),
			cv::Point(output_width - 1, 0),
			cv::Point(output_width - 1, output_height - 1),
			cv::Point(0, output_height - 1)
	};
	
	cv::Mat h = cv::getPerspectiveTransform(pts_src, pts_dst);
	cv::warpPerspective(frame, dest, h, cv::Size(output_width, output_height));
	
	return dest;
}

/**
 * @brief 제공된 카메라 파일을 사용하여 카메라 보정을 수행
 *
 * 이 함수는 주어진 카메라 파일을 사용하여 카메라를 보정합니다. 카메라 파일은 보정 과정에 필요한 설정 매개변수를 제공
 *
 * @param cam_file 보정에 사용될 카메라 파일. 보정 계수가 정해진 양식대로 있어야 함
 * @return 보정된 이미지를 반환
 *
 * @note 보정 과정이 성공하기 위해서는 카메라 파일이 올바른 형식이어야 하며 필요한 설정 매개변수를 포함해야 함.
 * 파일에는 다음 순서대로 보정계수가 있어야 하며, 계수는 줄바꿈으로 구별
 *
 */
cv::Mat camera::calibration(const std::string &cam_file)
{
	std::ifstream file(cam_file);
	if ( !file )
	{
		std::cerr << "File could not be opened!" << std::endl;
		
		return cv::Mat::zeros(0, 0, CV_32F);
	}
	
	std::array<std::string, 9> lines;
	std::string                line;
	
	for (int idx = 0; std::getline(file, line); idx++)
		lines[ idx ] = line;
	
	float cs = cos_pan * sin_tilt;
	float cc = cos_pan * cos_tilt;
	float ss = sin_pan * sin_tilt;
	float sc = sin_pan * cos_tilt;
	
	float r[12] = {
			cos_pan * cos_roll, sc + cs * sin_roll, ss - cc * sin_roll, 0.f,
			-sin_pan * cos_roll, cc - ss * sin_roll, cs + sc * sin_roll, 0.f,
			sin_roll, -sin_tilt * cos_roll, cos_tilt * cos_roll, 0.f
	};
	
	cv::Mat R(3, 4, CV_32F, r);
	
	float t[16] = {
			1.f, 0.f, 0.f, 0.f,
			0.f, 1.f, 0.f, 0.f,
			0.f, 0.f, 1.f, -height,
			0.f, 0.f, 0.f, 1.f
	};
	
	cv::Mat T(4, 4, CV_32F, t);
	
	float focusX  = std::stof(lines[ 0 ]);
	float focusY  = std::stof(lines[ 1 ]);
	float centerX = std::stof(lines[ 2 ]);
	float centerY = std::stof(lines[ 3 ]);
	
	float k[9] = {
			focusX, 0., centerX,
			0., focusY, centerY,
			0., 0., 1.
	};
	
	cv::Mat K_temp(3, 3, CV_32F, k);
	K_temp.copyTo(this->K);
	
	dist_coef[ 0 ] = std::stof(lines[ 4 ]);
	dist_coef[ 1 ] = std::stof(lines[ 5 ]);
	dist_coef[ 2 ] = std::stof(lines[ 6 ]);
	dist_coef[ 3 ] = std::stof(lines[ 7 ]);
	dist_coef[ 4 ] = std::stof(lines[ 8 ]);
	
	file.close();
	
	return K_temp * R * T;
}
