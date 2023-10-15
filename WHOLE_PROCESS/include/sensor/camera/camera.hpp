#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <cstdio>
#include <iostream>
#include <cmath>
#include <numbers>
#include <utility>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

namespace num = std::numbers;

#define D2R(X) ((X) * num::pi / 180.)
#define R2D(X) ((X) * 180. * num::inv_pi)

class camera
{
public:
	camera(float height, float x_min, float x_max, float y_min, float y_max,
	       const std::string &cam_file, float pan = 0.f, float tilt = 0.f, float roll = 0.f);
	
	~camera();
	
	cv::Mat get_calibration_mat();
	
	cv::Mat bev(const cv::Mat &frame);

protected:
	/*------------------------------------------------------------------*/
	/* 카메라 좌표계에서 실제 좌표계 변환에 필요한 변수                       */
	/*                                                                  */
	/* 참조: https://darkpgmr.tistory.com/84                             */
	/*------------------------------------------------------------------*/
	
	// 카메라 시선 각도에 대한 cos, sin 값
	float cos_pan;
	float sin_pan;
	float cos_tilt;
	float sin_tilt;
	float cos_roll;
	float sin_roll;
	
	// 월드 좌표계 중점(2D 라이다 중심)으로 부터 카메라가 떨어진 거리
	float height;
	
	cv::Mat K;
	cv::Mat KRT = cv::Mat::zeros(3, 4, CV_32F);   // 월드 좌표계에서 픽셀 좌표계 변환 행렬
	
	int output_width;                                               // 출력 이미지 폭
	int output_height;                                              // 출력 이미지 높이
	
	float world_x_min;                                              // BEV로 변환하려고 하는 영역 X 최솟값
	float world_x_max;                                              // BEV로 변환하려고 하는 영역 X 최댓값
	float world_y_min;                                              // BEV로 변환하려고 하는 영역 Y 최솟값
	float world_y_max;                                              // BEV로 변환하려고 하는 영역 Y 최댓값
	
	std::array<float, 5> dist_coef{};                               // 카메라 왜곡 보정값
	
	// 카메라 데이터 캘리브레이션
	cv::Mat calibration(const std::string &cam_file);
	
};    // class ipm

#endif // !CAMERA_HPP