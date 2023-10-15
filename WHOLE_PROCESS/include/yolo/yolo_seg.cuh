//
// Created by jarrygoon on 23. 8. 28.
//

#ifndef HADA_YOLO_SEG_CUH
#define HADA_YOLO_SEG_CUH

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

#include <NvInfer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

#include <cuda.h>
// #include <cuda_runtime_api.h>
#include <cuda_fp16.h>
#include <cublas_v2.h>

class YoloSeg
{
public:
	explicit YoloSeg(const std::string &_engine_path, int _num_class, float _threshold = 0.7f,
	                 bool _info_print = false);

//    ~YoloCpp();
	
	void run(cv::Mat &_input_img);

private:
	std::shared_ptr<nvinfer1::IRuntime>          runtime;
	std::shared_ptr<nvinfer1::ICudaEngine>       engine;
	std::shared_ptr<nvinfer1::IExecutionContext> context;
	
	cv::Size image_size;
	cv::Size input_size;
	
	int   num_class;
	int   num_box_rows;
	float threshold;
	
	class MyLogger : public nvinfer1::ILogger
	{
	public:
		void log(Severity _severity, const char* _msg) noexcept override;
	};
	
	void information();
	
	static std::string get_tensor_type(nvinfer1::DataType _type);
	
	cv::Mat tensor_process(cv::Mat &_input_img, half* _blob);
	
	std::vector<cv::cuda::GpuMat> mask_process(float* _pred_mask,
	                                           float* _output_mask,
	                                           const cv::Rect* _boxes_dev,
	                                           int _size) const;
	
	static cv::Mat drawMasks(const cv::Mat &_image,
	                         cv::Rect* _boxes,
	                         const int* _class_ids,
	                         int _size,
	                         double _mask_alpha = 0.3,
	                         std::vector<cv::cuda::GpuMat> _mask_maps = {});
};


#endif //HADA_YOLO_SEG_CUH
