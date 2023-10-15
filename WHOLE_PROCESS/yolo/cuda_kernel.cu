//
// Created by Jarry_Goon on 2023-09-16.
//

#include <opencv2/opencv.hpp>

#include <cuda_runtime_api.h>
#include <cuda_fp16.h>

#define BLOCK   1024
#define BLOCK2D dim3(32, 32)
#define BLOCK3D dim3(16, 16, 4)

#define GRID(SIZE) (((SIZE) + BLOCK - 1) / BLOCK)
#define GRID2D(X, Y) dim3((((X) + BLOCK2D.x - 1) / BLOCK2D.x), \
                          (((Y) + BLOCK2D.y - 1) / BLOCK2D.y))
#define GRID3D(X, Y, Z) dim3((((X) + BLOCK3D.x - 1) / BLOCK3D.x), \
                             (((Y) + BLOCK3D.y - 1) / BLOCK3D.y), \
                             (((Z) + BLOCK3D.z - 1) / BLOCK3D.z))

__global__
void half2float(const half* src, float* dst, int size)
{
	int array_idx = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (array_idx < size)
		dst[ array_idx ] = __half2float(src[ array_idx ]);
}

__global__
void float2half(const float* src, half* dst, int size)
{
	int array_idx = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (array_idx < size)
	{
		dst[ array_idx ] = __float2half_rn(src[ array_idx ]);
	}
}

__global__
void img2blob_kernel(const float* _img, half* blob, int _rows, int _cols, int _channels)
{
	int row     = blockIdx.x * blockDim.x + threadIdx.x;
	int col     = blockIdx.y * blockDim.y + threadIdx.y;
	int channel = blockIdx.z * blockDim.z + threadIdx.z;
	
	if (row < _rows && col < _cols && channel < _channels)
	{
		blob[ channel * _rows * _cols + row * _cols + col ] = __float2half_rn(
				_img[ channel + col * _channels + row * _channels * _cols ] / 255.f);
	}
}

__global__
void transpose_kernel(const half* src, float* dst, int _num_classes)
{
	int row = 4 + _num_classes + 32;
	
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	
	if (x < row && y < 8400)
		dst[ x + row * y ] = __half2float(src[ 8400 * x + y ]);
}

__global__
void find_max_score_kernel(const float* _box_array, float* _scores, int* _ids,
                           int _num_class, float _threshold, int* _size)
{
	int rows = 4 + _num_class + 32;
	int col  = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (col < 8400)
	{
		float max = _box_array[ rows * col + 4 ];
		float temp;
		
		for (int i = 5; i < _num_class + 4; i++)
		{
			temp = _box_array[ rows * col + i ];
			if (temp > max)
			{
				max = temp;
				_ids[ col ] = i;
			}
		}
		
		if (max > 0.000001) printf("%f\n", max);
		
		if (max > _threshold)
		{
			_scores[ col ] = max;
			atomicAdd(_size, 1);
		}
		else
			_scores[ col ] = -1.f;
	}
}

__global__
void score_check_kernel(const int* _ids, const float* _scores, int* _class_ids, int* _confirmed_idx)
{
	int idx = 0;
	
	for (int i = 0; i < 8400; i++)
	{
		if (_scores[ i ] == -1.f) continue;
		
		_confirmed_idx[ idx ] = i;
		_class_ids[ idx ]     = _ids[ i ];
		idx++;
	}
}

__global__
void predict_kernel(const int* _confirmed_idx, const float* _box_data, float* _predict_mask,
                    cv::Rect* boxes, int _size, int _box_rows, float _x_factor, float _y_factor, cv::Size _img_size)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	
	if (idx < _size)
	{
		int conf_idx = _confirmed_idx[ idx ];
		
		float x = _box_data[ _box_rows * conf_idx ];
		float w = _box_data[ _box_rows * conf_idx + 2 ];
		float y = _box_data[ _box_rows * conf_idx + 1 ];
		float h = _box_data[ _box_rows * conf_idx + 3 ];
		
		int x_min = (int) (( x - 0.5 * w ) * _x_factor );
		int x_max = (int) (( x + 0.5 * w ) * _x_factor );
		int y_min = (int) (( y - 0.5 * h ) * _y_factor );
		int y_max = (int) (( y + 0.5 * h ) * _y_factor );
		
		if (x_min < 0) x_min                = 0;
		if (x_max > _img_size.width) x_max  = _img_size.width;
		if (y_min < 0) y_min                = 0;
		if (y_max > _img_size.height) y_max = _img_size.height;
		
		boxes[ idx ].x      = x_min;
		boxes[ idx ].y      = y_min;
		boxes[ idx ].width  = x_max - x_min;
		boxes[ idx ].height = y_max - y_min;
		
		for (int i = 0; i < 32; i++)
			_predict_mask[ 32 * conf_idx + i ] = _box_data[ _box_rows * ( conf_idx + 1 ) - 32 + i ];
	}
}
