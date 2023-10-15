cmake_minimum_required(VERSION 3.25)
project(HADA LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_ARCHITECTURES 86)

<<<<<<< Updated upstream
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# 파이썬 패키지 참조 탐색
find_package(PythonLibs REQUIRED)
if (EXISTS C:/Python310/libs/python310.lib)
    set(PYTHON_LIBRARIES "C:/Python310/libs/python310.lib")
elseif (${Python3_FOUND})
    message("Python found: " &{Python3_EXECUTABLE})
    set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
else ()
    message("Python not found")
endif ()


# OpenCV 패키지 참조 탐색
# 환경 변수 상에 OpenCV_DIR라는 이름으로 경로 등록 필요
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR})

# CDUA 패키지 참조 탐색
find_package(CUDA REQUIRED)

# TensorRT는 Cmake 상에 경로가 등록되지 않으므로, 수동으로 라이브러리와 헤더 위치 탐색
if (UNIX)
    set(TENSORRT_INCLUDE_DIR NvInfer PATHS /usr/local/TensorRT/targets/x86_64-linux-gnu/include)
    find_library(TENSORRT_LIBRARY nvinfer PATHS /usr/local/TensorRT/targets/x86_64-linux-gnu/lib)
    find_library(TENSORRT_LIBRARY nvinfer_plugin PATHS /usr/local/TensorRT/targets/x86_64-linux-gnu/lib)
elseif (WIN32)
    set(TENSORRT_INCLUDE_DIR C:/TensorRT/include)
    find_library(TENSORRT_LIBRARY nvinfer PATHS C:/TensorRT/lib)
    find_library(TENSORRT_LIBRARY nvinfer_plugin PATHS C:/TensorRT/lib)

    # Windows는 추가적으로 Boost 라이브러리 위치 탐색 필요
    # Boost 라이브러리 디렉토리 입력
    set(Boost_DIR C:/boost_1_82_0)
    # Boost 패키지 참조 탐색 및 해더 및 라이브러리 경로 설정
    find_package(Boost REQUIRED PATHS ${Boost_DIR}/stage/lib/cmake/Boost-1.82.0 COMPONENTS python numpy)
    set(Boost_LIBRARIES C:/boost_1_82_0/stage/lib)
    set(Boost_INCLUDE_DIR C:/boost_1_82_0)
endif ()

add_subdirectory(camera)
add_subdirectory(lidar3d)
add_subdirectory(yolo)

add_dependencies(lidar3d camera)
add_subdirectory(RS232)
=======
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
#
## 파이썬 패키지 참조 탐색
#find_package(PythonLibs REQUIRED)
##set(PYTHON_LIBRARIES "C:/Python310/libs/python310.lib")
#
## 황승언 개인 주소
##set(PYTHON_LIBRARIES "C:/Users/erich/AppData/Local/Programs/Python/Python310/libs/python310.lib")
#
#
## OpenCV 패키지 참조 탐색
## 환경 변수 상에 OpenCV_DIR라는 이름으로 경로 등록 필요
#find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR})
#
## CDUA 패키지 참조 탐색
#find_package(CUDA REQUIRED)
#
## TensorRT는 Cmake 상에 경로가 등록되지 않으므로, 수동으로 라이브러리와 헤더 위치 탐색
#if (UNIX)
#    set(TENSORRT_INCLUDE_DIR NvInfer PATHS /usr/local/TensorRT/targets/x86_64-linux-gnu/include)
#    find_library(TENSORRT_LIBRARY nvinfer PATHS /usr/local/TensorRT/targets/x86_64-linux-gnu/lib)
#    find_library(TENSORRT_LIBRARY nvinfer_plugin PATHS /usr/local/TensorRT/targets/x86_64-linux-gnu/lib)
#elseif (WIN32)
#    set(TENSORRT_INCLUDE_DIR C:/TensorRT/include)
#    find_library(TENSORRT_LIBRARY nvinfer PATHS C:/TensorRT/lib)
#    find_library(TENSORRT_LIBRARY nvinfer_plugin PATHS C:/TensorRT/lib)
#
#    # Windows는 추가적으로 Boost 라이브러리 위치 탐색 필요
#    # Boost 라이브러리 디렉토리 입력
#    set(Boost_DIR C:/boost_1_82_0)
#    # Boost 패키지 참조 탐색 및 해더 및 라이브러리 경로 설정
#    find_package(Boost REQUIRED PATHS ${Boost_DIR}/stage/lib/cmake/Boost-1.82.0 COMPONENTS python numpy)
#    set(Boost_LIBRARIES C:/boost_1_82_0/stage/lib)
#    set(Boost_INCLUDE_DIR C:/boost_1_82_0)
#endif ()
#
#add_subdirectory(camera)
#add_subdirectory(lidar3d)
#add_subdirectory(yolo)
#
#add_dependencies(lidar3d camera)
>>>>>>> Stashed changes

add_executable(toybox jarrys_toybox.cpp)
# add_subdirectory(Yolo)
