//
// Created by jarrygoon on 23. 8. 28.
//

#include "../include/yolo/yolo_seg.cuh"
#include "cuda_kernel.cu"

YoloSeg::YoloSeg(const std::string &_engine_path, int _num_class, float _threshold, bool _info_print) :
		threshold(_threshold), num_class(_num_class)
{
	input_size   = cv::Size(640, 640);
	num_box_rows = 4 + _num_class + 32;
	
	MyLogger logger;
	
	std::ifstream file(_engine_path, std::ios::binary);
	if (!file)
	{
		std::cerr << "Fail to open " << _engine_path << " file" << std::endl;
		
		return;
	}
	
	file.seekg(0, std::ios::end);
	const size_t model_size = file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<char> engine_data(model_size);
	file.read(engine_data.plot(), model_size);
	file.close();
	
	runtime.reset(nvinfer1::createInferRuntime(logger));
	engine.reset(runtime->deserializeCudaEngine(engine_data.plot(), model_size));
	context.reset(engine->createExecutionContext());
	
	if (_info_print) information();
}

cv::cuda::GpuMat resizeKeepAspectRatioPadRightBottom(const cv::cuda::GpuMat &_input,
                                                     size_t _height,
                                                     size_t _width,
                                                     const cv::Scalar &_bgcolor)
{
	float r       = std::min((int) _width / ( _input.cols * 1.0 ), (int) _height / ( _input.rows * 1.0 ));
	int   unpad_w = static_cast<int>(r * _input.cols);
	int   unpad_h = static_cast<int>(r * _input.rows);
	
	cv::cuda::GpuMat re(unpad_h, unpad_w, CV_8UC3);
	cv::cuda::resize(_input, re, re.size());
	cv::cuda::GpuMat out(_height, _width, CV_8UC3, _bgcolor);
	
	re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
	
	return out;
}

template<typename T>
void BlobFromImage(cv::Mat &iImg, T &iBlob)
{
	int channels  = iImg.channels();
	int imgHeight = iImg.rows;
	int imgWidth  = iImg.cols;
	
	for (int c = 0; c < channels; c++)
	{
		for (int h = 0; h < imgHeight; h++)
		{
			for (int w = 0; w < imgWidth; w++)
			{
				iBlob[ c * imgWidth * imgHeight + h * imgWidth + w ] = typename std::remove_pointer<T>::type(
						( iImg.at<cv::Vec3b>(h, w)[ c ] ) / 255.0f);
			}
		}
	}
}

void YoloSeg::run(cv::Mat &_input_img)
{
	image_size = _input_img.size();
	
	cv::cuda::GpuMat input_img;
	input_img.upload(_input_img);
	
	int rows     = input_img.rows;
	int cols     = input_img.cols;
	int channels = input_img.channels();
	
	int size = rows * cols * channels;
	
	cv::cuda::cvtColor(input_img, input_img, cv::COLOR_BGR2RGB);
	cv::cuda::resize(input_img, input_img, input_size);
	input_img.convertTo(input_img, CV_32FC3);
	cv::cuda::divide(input_img, cv::Scalar::all(255.), input_img, 1, -1, cv::cuda::Stream::Null());
	
	float* img_data;
	cudaMalloc(&img_data, size * sizeof(float));
	cudaMemcpy(img_data, input_img.cudaPtr(), size * sizeof(float), cudaMemcpyHostToDevice);
	
	half* blob;
	cudaMalloc(&blob, 640 * 640 * 3 * sizeof(half));
	
	float2half<<<GRID(640 * 640 * 3), BLOCK>>>(img_data, blob, 640 * 640 * 3);
	cudaDeviceSynchronize();
	
	cv::Mat img = tensor_process(_input_img, blob);
	
	cv::imshow("test", img);
	
	cudaFree(blob);
	cudaFree(img_data);
	input_img.release();
}

void YoloSeg::information()
{
	int layers = engine->getNbIOTensors();
	
	for (int i = 0; i < layers; i++)
	{
		const char* engine_name = engine->getIOTensorName(i);
		nvinfer1::Dims32 shape = engine->getTensorShape(engine_name);
		
		std::cout << "Layer " << i << std::endl;
		std::cout << "Name: " << engine_name << std::endl;
		std::cout << "Type: " << get_tensor_type(engine->getTensorDataType(engine_name)) << std::endl;
		std::cout << "Shape: (";
		for (int j = 0; j < shape.nbDims; j++)
			std::cout << shape.d[ j ] << ", ";
		std::cout << ")\n" << std::endl;
	}
}

void YoloSeg::MyLogger::log(Severity _severity, const char* _msg) noexcept
{
	switch (_severity)
	{
		case Severity::kINTERNAL_ERROR:
			std::cerr << "INTERNAL_ERROR: " << _msg << std::endl;
			break;
		case Severity::kERROR:
			std::cerr << "ERROR: " << _msg << std::endl;
			break;
		case Severity::kWARNING:
			std::cerr << "WARNING: " << _msg << std::endl;
			break;
//        case Severity::kINFO:
//            std::cout << "INFO: " << msg << std::endl;
//            break;
		default:
			break;
	}
}

std::string YoloSeg::get_tensor_type(nvinfer1::DataType _type)
{
	switch (_type)
	{
		case ( nvinfer1::DataType::kUINT8 ):
			return "unsigned char";
		case ( nvinfer1::DataType::kINT8 ):
			return "char";
		case ( nvinfer1::DataType::kHALF ):
			return "float16";
		case ( nvinfer1::DataType::kBOOL ):
			return "unsigned boolean";
		case ( nvinfer1::DataType::kFLOAT ):
			return "float";
		case ( nvinfer1::DataType::kINT32 ):
			return "int";
		default:
			return "None";
	}
}

cv::Mat YoloSeg::tensor_process(cv::Mat &_input_img, half* _blob)
{
	cv::Mat mask_img;
	_input_img.copyTo(mask_img);
	
	half* output_mask;
	half* output_box;   // num_mask_data x 8400
	cudaMalloc(&output_mask, 32 * 160 * 160 * sizeof(half));
	cudaMalloc(&output_box, num_box_rows * 8400 * sizeof(half));
	
	void* buffer[] = { _blob, output_mask, output_box };

//	context->setTensorAddress("images", _blob);
//	context->setTensorAddress("output1", output_mask);
//	context->setTensorAddress("output0", output_box);
//
//	cudaStream_t cuda_stream;
//	cudaStreamCreate(&cuda_stream);
//
//	if (context->enqueueV3(cuda_stream))
//		return mask_img;
	if (!context->executeV2(buffer))
		return mask_img;
	
	float* box_data;   // 8400 x num_mask_data
	cudaMalloc(&box_data, num_box_rows * 8400 * sizeof(float));
	
	transpose_kernel<<<GRID2D(num_box_rows, 8400), BLOCK2D>>>(output_box, box_data, num_class);
	cudaDeviceSynchronize();
	
	float* mask_float;
	cudaMalloc(&mask_float, 32 * 160 * 160 * sizeof(float));
	half2float<<<GRID(32 * 160 * 160), BLOCK>>>(output_mask, mask_float, 32 * 160 * 160);
	cudaDeviceSynchronize();
	
	float* scores;
	int  * ids;
	cudaMalloc(&scores, 8400 * sizeof(float));
	cudaMalloc(&ids, 8400 * sizeof(int));
	
	int* size_dev;
	cudaMalloc(&size_dev, sizeof(int));
	cudaMemset(size_dev, 0, sizeof(int));
	find_max_score_kernel<<<GRID(8400), BLOCK>>>(box_data, scores, ids,
	                                             num_class, threshold, size_dev);
	cudaDeviceSynchronize();
	
	int size;
	cudaMemcpy(&size, size_dev, sizeof(int), cudaMemcpyDeviceToHost);
	
	if (size)
	{
		cv::Rect* boxes_dev;
		float   * predict_mask;
		int     * class_ids_dev;
		int     * confirmed_idx;
		cudaMalloc(&boxes_dev, size * sizeof(cv::Rect));
		cudaMalloc(&predict_mask, size * 32 * sizeof(float));
		cudaMalloc(&class_ids_dev, size * sizeof(int));
		cudaMalloc(&confirmed_idx, size * sizeof(int));
		
		float x_factor = (float) image_size.width / (float) input_size.width;
		float y_factor = (float) image_size.height / (float) input_size.height;
		
		score_check_kernel<<<1, 1>>>(ids, scores, class_ids_dev, confirmed_idx);
		predict_kernel<<<GRID(size), BLOCK>>>(confirmed_idx, box_data, predict_mask, boxes_dev, size, num_box_rows,
		                                      x_factor, y_factor, image_size);
		cudaDeviceSynchronize();
		
		int* class_ids = new int[size];
		cudaMemcpy(class_ids, class_ids_dev, size * sizeof(int), cudaMemcpyDeviceToHost);
		
		auto masks_maps = mask_process(predict_mask, mask_float, boxes_dev, size);
		
		auto boxes = new cv::Rect[size];
		cudaMemcpy(boxes, boxes_dev, size * sizeof(cv::Rect), cudaMemcpyDeviceToHost);
		
		mask_img = drawMasks(_input_img, boxes, class_ids, size, 0.3, masks_maps);
		
		delete[] boxes;
		delete[] class_ids;
		
		cudaFree(confirmed_idx);
		cudaFree(class_ids_dev);
		cudaFree(predict_mask);
		cudaFree(boxes_dev);
	}
	
	cudaFree(output_box);
	cudaFree(output_mask);
	cudaFree(size_dev);
	cudaFree(ids);
	cudaFree(scores);
	cudaFree(mask_float);
	
	return mask_img;
}

std::vector<cv::cuda::GpuMat> YoloSeg::mask_process(float* _pred_mask,
                                                    float* _output_mask,
                                                    const cv::Rect* _boxes_dev,
                                                    int _size) const
{
	cublasHandle_t handle;
	cublasCreate_v2(&handle);
	
	float alpha = 1.f;
	float beta  = 0.f;
	
	float* masks_dev;
	cudaMalloc(&masks_dev, _size * 160 * 160 * sizeof(float));
	cudaMemset(masks_dev, 0, _size * 160 * 160 * sizeof(float));
	
	cublasSgemm_v2(handle, CUBLAS_OP_N, CUBLAS_OP_N, 2, 2, 2, &alpha, _pred_mask, _size, _output_mask, 32, &beta,
	               masks_dev, _size);
	
	cublasDestroy_v2(handle);
	
	std::vector<cv::cuda::GpuMat> masks_vec(_size);
	
	for (int i = 0; i < _size; i++)
		masks_vec[ i ] = cv::cuda::GpuMat(160, 160, CV_32F, masks_dev + 160 * 160 * i);
	
	std::vector<cv::cuda::GpuMat> mask_maps(_size);
	
	float x_factor = 160.f / (float) image_size.width;
	float y_factor = 160.f / (float) image_size.height;
	
	cv::Size blur_size(static_cast<int>(1.f / x_factor), static_cast<int>(1.f / y_factor));
	if (blur_size.width % 2 == 0) blur_size.width++;
	if (blur_size.height % 2 == 0) blur_size.height++;
	
	cv::Ptr<cv::cuda::Filter> blur_filter = cv::cuda::createGaussianFilter(CV_32F, CV_32F, blur_size, 0);
	
	auto* boxes = (cv::Rect*) malloc(_size * sizeof(cv::Rect));
	cudaMemcpy(boxes, _boxes_dev, _size * sizeof(cv::Rect), cudaMemcpyDeviceToHost);
	
	for (int i = 0; i < _size; i++)
	{
		cv::Mat          mask_cpu;
		cv::cuda::GpuMat mask_map;
		cv::Rect         box;
		
		mask_cpu = cv::Mat::zeros(image_size.height, image_size.width, CV_8U);
		mask_map.upload(mask_cpu);
		
		box.x      = static_cast<int>((float) boxes[ i ].x * x_factor);
		box.y      = static_cast<int>((float) boxes[ i ].y * x_factor);
		box.width  = static_cast<int>((float) boxes[ i ].width * y_factor);
		box.height = static_cast<int>((float) boxes[ i ].height * y_factor);
		
		if (box.x + box.width > 160) box.width   = 160 - box.x;
		if (box.y + box.height > 160) box.height = 160 - box.y;
		
		cv::cuda::GpuMat scale_crop_mask = masks_vec[ i ](box);
		cv::cuda::GpuMat crop_mask;
		cv::cuda::resize(scale_crop_mask, crop_mask, box.size(), 0, 0, cv::INTER_CUBIC);
		scale_crop_mask.release();
		
		blur_filter->apply(crop_mask, crop_mask);
		
		cv::cuda::GpuMat temp;
		cv::cuda::compare(crop_mask, 0.5, temp, cv::CMP_GT);
		
		temp.copyTo(mask_map(box));
		temp.release();
		
		mask_map.copyTo(mask_maps[ i ]);
		mask_map.release();
	}
	
	delete[] boxes;
	cudaFree(masks_dev);
	
	return mask_maps;
}

cv::Mat YoloSeg::drawMasks(const cv::Mat &_image,
                           cv::Rect* _boxes,
                           const int* _class_ids,
                           int _size,
                           double _mask_alpha,
                           std::vector<cv::cuda::GpuMat> _mask_maps)
{
	cv::Mat mask_img;
	_image.convertTo(mask_img, mask_img.type());
	
	for (int i = 0; i < _size; i++)
	{
		cv::Rect box      = _boxes[ i ];
		int      class_id = _class_ids[ i ];
		
		cv::Scalar color = cv::Scalar(1, 0, 0);
		
		if (_mask_maps.empty())
		{
			cv::rectangle(mask_img, box, color, -1);
		}
		else
		{
			cv::Mat mask_map;
			_mask_maps[ i ].download(mask_map);
			cv::Mat subImg = mask_map(box);
			
			cv::Mat subImg32F;
			subImg.convertTo(subImg32F, CV_32FC3);
			cv::cvtColor(subImg32F, subImg32F, cv::COLOR_GRAY2RGB);
			
			cv::Mat colorMat(subImg.rows, subImg.cols, CV_32FC3, color);
			
			cv::cuda::multiply(subImg32F, ( 1 - 0.0 ), subImg32F);
			cv::cuda::addWeighted(subImg32F, 1.0, colorMat, 0.0, 0, subImg32F);
			
			cv::Mat subImg8U;
			subImg32F.convertTo(subImg8U, CV_8UC3);
			mask_img(box).copyTo(subImg8U);
		}
	}
	
	cv::Mat result;
	cv::cuda::addWeighted(mask_img, 1.0, _image, 1 - 0.0, 0, result);
	
	return result;
}