add_executable(test0Cmake test0Cmake.cpp)
target_link_libraries(test0Cmake ${OpenCV_LIBS})


add_executable(test1readFrameData test1ReadFrameData.cpp)
target_link_libraries(test1readFrameData m3d_io ${OpenCV_LIBS} )


add_executable(test2FeatureExtractor test2FeatureExtractor.cpp)
target_link_libraries(test2FeatureExtractor ${OpenCV_LIBS})


add_executable(test3GmsFilter test3GmsFilter.cpp)
target_link_libraries(test3GmsFilter ${OpenCV_LIBS})