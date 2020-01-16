
# find_package(OpenCV REQUIRED)
add_executable(testStitchViaGco testStitchViaGco.cpp 7PenaltiesGenerator.cpp)
target_link_libraries(testStitchViaGco gco ${OpenCV_LIBS})

# test multiband1. codes from csdn-blog
add_executable(testDetails testDetails.cpp)
target_link_libraries(testDetails ${OpenCV_LIBS})

# test multiband1. codes from csdn-blog
add_executable(testMultiband1 testMultiband1.cpp)
target_link_libraries(testMultiband1 ${OpenCV_LIBS})

# test costfunctor of gco.
add_executable(testCostFunctor testCostFunctor.cpp)
target_link_libraries(testCostFunctor gco ${OpenCV_LIBS})

# test rotation cost functor.
add_executable(testRotations testRotations.cpp)
target_link_libraries(testRotations m3dbase ${OpenCV_LIBS} ${PCL_LIBRARIES})

# test #define and typedef
add_executable(debug4RotationInitializer debug4RotationInitializer.cpp 4RotationInitializer.cpp  ${M3D_INITIALIZER_SRCS})
target_link_libraries(debug4RotationInitializer m3dbase gco ${OpenCV_LIBS} ${PCL_LIBRARIES})