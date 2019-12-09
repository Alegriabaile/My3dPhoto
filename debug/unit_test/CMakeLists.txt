add_executable(test0Cmake test0Cmake.cpp)
target_link_libraries(test0Cmake ${OpenCV_LIBS})


add_executable(test1readFrameData test1ReadFrameData.cpp ${M3D_IO_SRCS})
target_link_libraries(test1readFrameData ${OpenCV_LIBS} )


add_executable(test2FeatureExtractor test2FeatureExtractor.cpp)
target_link_libraries(test2FeatureExtractor ${OpenCV_LIBS})


add_executable(test3GmsFilter test3GmsFilter.cpp)
target_link_libraries(test3GmsFilter ${OpenCV_LIBS})


add_executable(test6SphereProjection test6SphereProjection.cpp
        ../../src/6CubemapCapturer.cpp
        ../../src/6Cubemap2Sphere.cpp
        ${M3D_GLAD_SRCS})
target_link_libraries(test6SphereProjection ${OpenCV_LIBS} glfw libGL.so)


#add_executable(test4OverlappingPairs )