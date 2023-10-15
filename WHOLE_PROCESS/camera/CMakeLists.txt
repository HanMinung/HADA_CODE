add_library(camera SHARED camera.cpp)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS})
target_include_directories(camera PUBLIC ${OpenCV_DIR}/include)

add_library(camera_lib STATIC camera.cpp)
target_link_libraries(camera_lib PUBLIC ${OpenCV_LIBS})
target_include_directories(camera_lib PUBLIC ${OpenCV_DIR}/include)

# PYD
add_library(camera_py SHARED py/camera_python.cpp)
target_link_libraries(camera_py PRIVATE ${PYTHON_LIBRARIES} camera)
target_link_directories(camera_py PRIVATE ${Boost_LIBRARIES})
target_include_directories(camera_py PRIVATE ${OpenCV_DIR}/include ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
set_target_properties(camera_py PROPERTIES SUFFIX ".pyd")
