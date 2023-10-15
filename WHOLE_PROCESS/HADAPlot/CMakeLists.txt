add_library(HADAPlot SHARED HADA_plot.cpp draw.cpp)
target_link_libraries(HADAPlot PUBLIC ${OpenCV_LIBS})
target_include_directories(HADAPlot PUBLIC ${OpenCV_DIR}/include)

add_library(HADAPlot_lib STATIC HADA_plot.cpp draw.cpp)
target_link_libraries(HADAPlot_lib PUBLIC ${OpenCV_LIBS})
target_include_directories(HADAPlot_lib PUBLIC ${OpenCV_DIR}/include)
