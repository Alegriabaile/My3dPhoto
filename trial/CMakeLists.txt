

# find_package(OpenCV REQUIRED)
add_executable(testStitchViaGco testStitchViaGco.cpp 7PenaltiesGenerator.cpp)
target_link_libraries(testStitchViaGco gco ${OpenCV_LIBS})

# test multiband1. codes from csdn-blog
add_executable(testMultiband testMultiband.cpp)
target_link_libraries(testMultiband ${OpenCV_LIBS})

# test multiband1. codes from csdn-blog
add_executable(testMultiband1 testMultiband1.cpp)
target_link_libraries(testMultiband1 ${OpenCV_LIBS})