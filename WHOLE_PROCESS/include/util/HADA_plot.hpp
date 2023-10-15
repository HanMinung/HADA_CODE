//
// Created by Jarry_Goon on 2023-09-28.
//

#ifndef HADA_HADA_PLOT_HPP
#define HADA_HADA_PLOT_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class plt
{
public:
	plt(int width = 800, int height = 800, cv::Scalar color = cv::Scalar(255, 255, 255));
	
	void xlim(double min, double max);
	
	void ylim(double min, double max);
	
	void plot(const std::vector<double> &x, const std::vector<double> &y, char line_type = '-', int line_width = 1);
	
	void view(int wait_time = 1);

//	template<typename T>
//	void data(const std::vector<T> &x, char line_type = '-', int line_width = 4);

private:
	int        Width;
	int        Height;
	cv::Scalar Color;
	
	double X_Max;
	double X_Min;
	double Y_Max;
	double Y_Min;
	
	cv::Mat Plot_Img;
	
};

#endif //HADA_HADA_PLOT_HPP
