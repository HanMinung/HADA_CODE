add_library(lidar3d SHARED projector.cpp velodyne_driver.cpp)
target_link_libraries(lidar3d PUBLIC ${OpenCV_LIBS} camera_lib)
target_include_directories(lidar3d PUBLIC include)

add_library(lidar3d_lib STATIC projector.cpp velodyne_driver.cpp)
target_link_libraries(lidar3d_lib PUBLIC ${OpenCV_LIBS} camera_lib)
target_include_directories(lidar3d_lib PUBLIC include)

# PYD
add_library(lidar3d_py SHARED py/lidar3d_python.cpp)
target_link_libraries(lidar3d_py PRIVATE ${PYTHON_LIBRARIES} camera_py lidar3d_lib)
target_link_directories(lidar3d_py PRIVATE lidar3d/py ${Boost_LIBRARIES})
target_include_directories(lidar3d_py PRIVATE ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_DIR}/include)
set_target_properties(lidar3d_py PROPERTIES SUFFIX ".pyd")
