# unit test
add_subdirectory("unit_test")


add_executable(debug2FeatureExtractor debug2FeatureExtractor.cpp)
target_link_libraries(debug2FeatureExtractor ${OpenCV_LIBS} m3dbase)

add_executable(debug3FeatureMatcher debug3FeatureMatcher.cpp)
target_link_libraries(debug3FeatureMatcher ${OpenCV_LIBS} m3dbase)

add_executable(debug4OverlapEstimator debug4OverlapEstimator.cpp ${M3D_INITIALIZER_SRCS})
target_link_libraries(debug4OverlapEstimator m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(debug4GraphInitializer debug4GraphInitializer.cpp ${M3D_INITIALIZER_SRCS})
target_link_libraries(debug4GraphInitializer m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(debug5PoseNLLSOptimizer debug5PoseNLLSOptimizer.cpp ${M3D_INITIALIZER_SRCS})
target_link_libraries(debug5PoseNLLSOptimizer m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# pcl error... debug...
#add_executable(debugPclLib debugPclLib.cpp)
#target_link_libraries(debugPclLib m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES})



add_executable(debugCurrent debugCurrent.cpp ${M3D_INITIALIZER_SRCS} ${M3D_WARPER_SRCS})
target_link_libraries(debugCurrent m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} glfw libGL.so)


