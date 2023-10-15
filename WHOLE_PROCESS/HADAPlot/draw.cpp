//
// Created by Jarry_Goon on 2023-09-28.
//
#include <opencv2/opencv.hpp>

#define LABEL_LINE_THICK 2
#define GRADUATION_LINE_LEN 10

void draw_axis(cv::Mat data, int size, double x_factor, double y_factor)
{
	int graduation_x = GRADUATION_LINE_LEN * y_factor;
	int graduation_y = GRADUATION_LINE_LEN * x_factor;
	
	// x축 선
	cv::line(data,
	         cv::Point((int) ( 0.05 * data.rows - graduation_y ), (int) ( 0.95 * data.cols )),
	         cv::Point((int) ( 0.95 * data.rows ), (int) ( 0.95 * data.cols )),
	         cv::Scalar(0, 0, 0),
	         LABEL_LINE_THICK * y_factor
	);
	
	// y축 선
	cv::line(data,
	         cv::Point((int) ( 0.05 * size ), (int) ( 0.05 * size )),
	         cv::Point((int) ( 0.05 * size ), (int) ( 0.95 * size + graduation_x )),
	         cv::Scalar(0, 0, 0),
	         LABEL_LINE_THICK * x_factor
	);
	
	// 축 눈금
	for (int i = 1; i < 5; i++)
	{
		double point = 0.9 * i / 4;
		
		cv::line(data,
		         cv::Point((int) (( 0.05 + point ) * data.rows ), (int) ( 0.95 * data.cols )),
		         cv::Point((int) (( 0.05 + point ) * data.rows ), (int) ( 0.95 * data.cols + graduation_x )),
		         cv::Scalar(0, 0, 0),
		         LABEL_LINE_THICK * y_factor
		);
		
		cv::line(data,
		         cv::Point((int) ( 0.05 * data.rows ), (int) (( 0.95 - point ) * data.cols )),
		         cv::Point((int) ( 0.05 * data.rows - graduation_y ), (int) (( 0.95 - point ) * data.cols )),
		         cv::Scalar(0, 0, 0),
		         LABEL_LINE_THICK * x_factor
		);
	}
}

void draw_line(cv::Mat data, const std::vector<double> &x, const std::vector<double> &y, int size,
               double x_min, double x_len, double y_min, double y_len, double factor, int line_width)
{
	for (int i = 1; i < size; i++)
	{
		if (( x[ i - 1 ] - x_min > x_len ) || ( y[ i - 1 ] - y_min > y_len )) continue;
		
		cv::line(data,
		         cv::Point((int) (( 0.05 + 0.9 * ( x[ i - 1 ] - x_min ) / x_len ) * data.cols ),
		                   (int) (( 0.95 - 0.9 * ( y[ i - 1 ] - y_min ) / y_len ) * data.rows )),
		         cv::Point((int) (( 0.05 + 0.9 * ( x[ i ] - x_min ) / x_len ) * data.cols ),
		                   (int) (( 0.95 - 0.9 * ( y[ i ] - y_min ) / y_len ) * data.rows )),
		         cv::Scalar(255, 0, 0),
		         (int) ( factor * line_width )
		);
	}
}

void draw_point(cv::Mat data, const std::vector<double> &x, const std::vector<double> &y, int size,
                double x_min, double x_len, double y_min, double y_len, double x_factor, double y_factor)
{
	for (int i = 0; i < size; i++)
	{
		if (( x[ i ] - x_min > x_len ) || ( y[ i ] - y_min > y_len )) continue;
		
		cv::ellipse(data,
		            cv::Point((int) (( 0.05 + 0.9 * ( x[ i ] - x_min ) / x_len ) * data.cols ),
		                      (int) (( 0.95 - 0.9 * ( y[ i ] - y_min ) / y_len ) * data.rows )),
		            cv::Size((int) ( 4 * x_factor ), (int) ( 4 * y_factor )),
		            0,
		            0,
		            360,
		            cv::Scalar(255, 0, 0),
		            cv::FILLED
		);
	}
}