cmake_minimum_required(VERSION 3.14)
project(My3dPhoto)

set(CMAKE_CXX_STANDARD 11)

#PROJECT_SOURCE_DIR: 工程路径
#PROJECT_BINARY_DIR: build路径
#find_package(Eigen3  REQUIRED)
#find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(glfw3 3.2 REQUIRED)
#find_package(OpenGL 3.3 REQUIRED)
#find_package(PCL REQUIRED)#COMPONENTS common io)# visualization)# COMPONENTS common io visualization)

# ////////////////////// binary/library dir for generating ///////////////////////////
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib)

## ==============================================================================
## pcl
## ==============================================================================
## - external by default
## ==============================================================================
find_package(PCL REQUIRED)#COMPONENTS common io)# visualization)# COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})



# ==============================================================================
# eigen3
# ==============================================================================
# - internal by default
# ==============================================================================
include_directories(${PROJECT_SOURCE_DIR}/3rd/eigen3)

# ==============================================================================
# ceres
# ==============================================================================
# - external by default
# ==============================================================================
find_package(Ceres REQUIRED)

# ==============================================================================
# glfw
# ==============================================================================
# - external by default
# ==============================================================================
find_package(glfw3 3.2 REQUIRED)

# ==============================================================================
# glad
# ==============================================================================
# - internal by default
# ==============================================================================
#file(GLOB GLAD_SRCS
#        3rd/glad/src/glad.c)
#
#add_library(glad STATIC ${GLAD_SRCS})
##set_target_properties(glad PROPERTIES COMPILE_FLAGS "-O2 -DNDEBUG")
## CMakeLists file include environments.
#target_include_directories(glad PRIVATE 3rd/glad/include)




# ==============================================================================
# m3d base: basic data structure, io...
# ==============================================================================
# - internal by default
# ==============================================================================
file(GLOB M3D_BASE_SRCS
        src/0m3d.cpp
        src/1Logger.cpp
        src/1FrameReader.cpp)
add_library(m3dbase STATIC ${M3D_BASE_SRCS})
target_include_directories(m3dbase PRIVATE include)


file(GLOB M3D_INITIALIZER_SRCS
        src/4OverlapEstimator.cpp
        src/4GraphInitializer.cpp)



# ==============================================================================
# source file include environments.
include_directories("3rd/glad/include")
include_directories("include")
include_directories("3rd/GMS-Feature-Matcher/include")






add_subdirectory("debug/")