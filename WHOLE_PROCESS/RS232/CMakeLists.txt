add_library(RS232 SHARED RS232.cpp)

add_library(RS232_lib STATIC RS232.cpp)

add_library(RS232_py SHARED py/RS232_python.cpp)
target_link_libraries(RS232_py PRIVATE ${PYTHON_LIBRARIES} RS232_lib)
target_link_directories(RS232_py PRIVATE py ${Boost_LIBRARIES})
target_include_directories(RS232_py PRIVATE ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
set_target_properties(RS232_py PROPERTIES SUFFIX ".pyd")
