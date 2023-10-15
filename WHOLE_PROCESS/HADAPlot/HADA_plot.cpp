//
// Created by Jarry_Goon on 2023-09-28.
//

#include "../include/util/HADA_plot.hpp"
#include "draw.cpp"

#include <cmath>

plt::plt(int width, int height, cv::Scalar color) :
		Width(width), Height(height), Color(std::move(color)),
		X_Min(0.), X_Max(0.), Y_Min(0.), Y_Max(0.)
{}

void plt::xlim(double min, double max)
{
	X_Min = min;
	X_Max = max;
}

void plt::ylim(double min, double max)
{
	Y_Min = min;
	Y_Max = max;
}

void plt::plot(const std::vector<double> &x, const std::vector<double> &y, char line_type, int line_width)
{
	int size_x = (int) x.size();
	int size_y = (int) y.size();
	
	if (size_x != size_y)
	{
		std::cerr << "The plot sizes are not the same." << std::endl;
		return;
	}
	
	std::array<int, 3> temp = { size_x, Width, Height };
	int                size = *std::max_element(temp.begin(), temp.end());
	
	X_Min = *std::min_element(x.begin(), x.end());
	X_Max = *std::max_element(x.begin(), x.end());
	Y_Min = *std::min_element(y.begin(), y.end());
	Y_Max = *std::max_element(y.begin(), y.end());
	
	cv::Mat data(size, size, CV_8UC3, Color);
	
	double x_factor = (double) size / Width;
	double y_factor = (double) size / Height;
	double factor   = sqrt(x_factor * x_factor + y_factor * y_factor);
	
	double x_len = X_Max - X_Min;
	double y_len = Y_Max - Y_Min;

//	printf("%f %f\n", x_len, y_len);
	
	draw_axis(data, size, x_factor, y_factor);
	
	switch (line_type)
	{
		default:
		case '-':
			draw_line(data, x, y, size_x, X_Min, x_len, Y_Min, y_len, factor, line_width);
			break;
		case 'o':
			draw_point(data, x, y, size_x, X_Min, x_len, Y_Min, y_len, x_factor, y_factor);
			break;
	}
	
	cv::resize(data, data, cv::Size(Width, Height), 0, 0, cv::INTER_LINEAR);
	
	std::string x_data;
	std::string y_data;
	
	for (int i = 0; i < 5; i++)
	{
		std::ostringstream strs;
		strs << std::fixed << std::setprecision(2) << X_Min + x_len * i / 4;
		x_data = strs.str();
		strs << std::fixed << std::setprecision(2) << Y_Min + y_len * i / 4;
		y_data = strs.str();
		
		double point = 0.9 * i / 4;
		
		cv::putText(data,
		            x_data,
		            cv::Point((int) (( 0.05 + point ) * data.rows - 30 ), (int) ( 0.95 * data.cols + 28 )),
		            cv::FONT_HERSHEY_SIMPLEX,
		            0.7,
		            cv::Scalar(0, 0, 0)
		);
	}
	
	data.copyTo(Plot_Img);
}

void plt::view(int wait_time)
{
	cv::imshow("Plot", Plot_Img);
	cv::waitKey(wait_time);
}