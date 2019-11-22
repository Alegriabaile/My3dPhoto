# unit test
add_subdirectory("unit_test")


add_executable(debug2FeatureExtractor debug2FeatureExtractor.cpp)
target_link_libraries(debug2FeatureExtractor ${OpenCV_LIBS} m3dbase)

add_executable(debug3FeatureMatcher debug3FeatureMatcher.cpp)
target_link_libraries(debug3FeatureMatcher ${OpenCV_LIBS} m3dbase)




find_package(PCL REQUIRED)#COMPONENTS common io)# visualization)# COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(debugPclLib debugPclLib.cpp)
target_link_libraries(debugPclLib m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(debugCurrent debugCurrent.cpp ${M3D_OVERLAP_SRCS})
target_link_libraries(debugCurrent m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES})


