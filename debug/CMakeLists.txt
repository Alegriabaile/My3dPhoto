# ==============================================================================
# ==============================================================================
# unit test
add_subdirectory("unit_test")

# ==============================================================================
# ==============================================================================
# different progresses of the entire program.
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

add_executable(debug6PanoramaCapturer debug6PanoramaCapturer.cpp ${M3D_INITIALIZER_SRCS} ${M3D_PANOCAPTURER_SRCS} ${M3D_GLAD_SRCS})
target_link_libraries(debug6PanoramaCapturer m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} glfw libGL.so)

add_executable(debug7PanoramaStitcher debug7PanoramaStitcher.cpp ${M3D_INITIALIZER_SRCS} ${M3D_PANOCAPTURER_SRCS} ${M3D_STITCHER_SRCS} ${M3D_GLAD_SRCS} )
target_link_libraries(debug7PanoramaStitcher m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} glfw libGL.so)

add_executable(debug8BackLayerContentGenerator debug8BackLayerContentGenerator.cpp ${M3D_INITIALIZER_SRCS} ${M3D_PANOCAPTURER_SRCS} ${M3D_STITCHER_SRCS} ${M3D_CONTENT_SRCS} ${M3D_GLAD_SRCS})
target_link_libraries(debug8BackLayerContentGenerator m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} glfw libGL.so)

add_executable(debug9Viewer debug9Viewer.cpp ${M3D_3DVIEWER_SRCS} ${M3D_GLAD_SRCS})
target_link_libraries(debug9Viewer ${OpenCV_LIBS} glfw libGL.so)

add_executable(debug10GenerateResult debug10GenerateResult.cpp ${M3D_INITIALIZER_SRCS} ${M3D_PANOCAPTURER_SRCS} ${M3D_STITCHER_SRCS} ${M3D_CONTENT_SRCS} ${M3D_GLAD_SRCS})
target_link_libraries(debug10GenerateResult m3dbase ${OpenCV_LIBS} ${CERES_LIBRARIES} glfw libGL.so)

add_executable(debug10ViewResult debug10ViewResult.cpp ${M3D_3DVIEWER_SRCS} ${M3D_GLAD_SRCS})
target_link_libraries(debug10ViewResult ${OpenCV_LIBS} glfw libGL.so)
# pcl error... debug...
#add_executable(debugPclLib debugPclLib.cpp)
#target_link_libraries(debugPclLib m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES})

# differences between Rodrigues and EulerAngle. debug...
#add_executable(debug6CompareRodriguesEulerAngle debug6CompareRodriguesEulerAngle.cpp)
#target_link_libraries(debug6CompareRodriguesEulerAngle  ${OpenCV_LIBS})



# ==============================================================================
# ==============================================================================
# the current debugging process.
add_executable(debugCurrent debugCurrent.cpp ${M3D_INITIALIZER_SRCS} ${M3D_WARPER_SRCS} ${M3D_STITCHER_SRCS} ${M3D_CONTENT_SRCS})
target_link_libraries(debugCurrent m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} glfw libGL.so)


