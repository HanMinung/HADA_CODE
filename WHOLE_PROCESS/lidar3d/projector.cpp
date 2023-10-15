#include "../include/sensor/lidar3d/projector.hpp"

namespace Lidar3D
{
	projector::projector()
	{
		float pan   = D2R(86.);
		float tilt  = 0.;
		float gamma = 0.;
		
		float x0 = 0.;
		float y0 = 0.33;
		float z0 = 0.2;
		
		float cos_pan   = cosf(pan);
		float sin_pan   = sinf(pan);
		float cos_tilt  = cosf(tilt);
		float sin_tilt  = sinf(tilt);
		float cos_gamma = cosf(gamma);
		float sin_gamma = sinf(gamma);
		
		float cs = cos_pan * sin_gamma;
		float cc = cos_pan * cos_gamma;
		float ss = sin_pan * sin_gamma;
		float sc = sin_pan * cos_gamma;
		
		float r[12] = {
				cos_gamma * cos_tilt, -cos_tilt * sin_gamma, sin_tilt, x0,
				cs + sc * sin_tilt, cc - ss * sin_tilt, -cos_tilt * sin_pan, y0,
				ss - cc * sin_tilt, sc + cs * sin_tilt, cos_pan * cos_tilt, z0
		};
		
		cv::Mat R = cv::Mat(3, 4, CV_32F, r);
		
		float centerX = 322.35003847;
		float centerY = 226.76398398;
		float focusX  = 280.38650513;
		float focusY  = 269.07183838;
		
		float k[9] = {
				focusX, 0., centerX,
				0., focusY, centerY,
				0., 0., 1.
		};
		
		cv::Mat K_temp = cv::Mat(3, 3, CV_32F, k);
		
		cv::Mat rt = K_temp * R;
		rt.copyTo(RT);
	}
	
	/**
	 * \fn void projector::PCD(velodyne::driver* driver, const int object_center_u, const int object_center_v, const bool view)
	 * \brief Velodyne 드라이버의 포인트 클라우드 데이터(PCD)를 평면에 투사.
	 *
	 * \param driver Velodyne 드라이버 객체 포인터
	 * \param object_center_u 객체 중심의 가로 좌표
	 * \param object_center_v 객체 중심의 세로 좌표
	 * \param view 투사된 데이터 표시 여부
	 *
	 * 이 함수는 드라이버의 PCD를 객체 중심 좌표에 의해 정의된 평면에 투사. 투사된 점들은 view 매개변수에 따라 저장되거나 표시됨.
	 */
	
	double projector::PCD(Velodyne::driver* driver,
	                      int object_center_u,
	                      int object_center_v,
	                      bool view) const
	{
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
		
		return dist;
	}
}