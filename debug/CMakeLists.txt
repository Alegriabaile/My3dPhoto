# unit test
add_subdirectory("unit_test")





add_executable(debug2FeatureExtractor debug2FeatureExtractor.cpp)
target_link_libraries(debug2FeatureExtractor m3d_io ${OpenCV_LIBS})



add_executable(debugCurrent debugCurrent.cpp)
target_link_libraries(debugCurrent m3d_io ${OpenCV_LIBS})