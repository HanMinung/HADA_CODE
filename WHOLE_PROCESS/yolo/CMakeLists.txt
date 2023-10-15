add_library(yolo SHARED yolov8.cpp engine.cpp)# yolo_seg.cu cuda_kernel.cu)
target_include_directories(yolo PUBLIC
                           ${TENSORRT_INCLUDE_DIR}
                           ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(yolo PUBLIC
                      ${TENSORRT_LIBRARY}
                      ${TENSORRT_LIBRARY_PLUGIN}
                      ${CUDA_LIBRARIES}
                      ${CUDA_cublas_LIBRARY}
                      ${OpenCV_LIBS}
                      ${NVONNXPARSER_LIB}
)

add_library(yolo_lib STATIC yolov8.cpp engine.cpp)
target_include_directories(yolo_lib PUBLIC
                           ${TENSORRT_INCLUDE_DIR}
                           ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(yolo_lib PUBLIC
                      ${TENSORRT_LIBRARY}
                      ${TENSORRT_LIBRARY_PLUGIN}
                      ${CUDA_LIBRARIES}
                      ${CUDA_cublas_LIBRARY}
                      ${OpenCV_LIBS}
                      ${NVONNXPARSER_LIB}
)