
add_executable(testCmake testCmake.cpp)
target_link_libraries(testCmake ${OpenCV_LIBS})
#set_property(TARGET testCmake PROPERTY FOLDER debug/)