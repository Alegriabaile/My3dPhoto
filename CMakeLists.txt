cmake_minimum_required(VERSION 3.14)
project(My3dPhoto)

set(CMAKE_CXX_STANDARD 11)

# ==============================================================================
# ==============================================================================
# ////////////////////// binary/library dir for generating ///////////////////////////
#PROJECT_SOURCE_DIR: 工程路径
#PROJECT_BINARY_DIR: build路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib)

# ==============================================================================
# ==============================================================================
# opencv 3.4.1 customly compiled from source code.
find_package(OpenCV REQUIRED)

# ==============================================================================
# pcl
find_package(PCL REQUIRED)#COMPONENTS common io)# visualization)# COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})

# ==============================================================================
# eigen3
include_directories(${PROJECT_SOURCE_DIR}/3rd/eigen3)

# ==============================================================================
# ceres
find_package(Ceres REQUIRED)

# ==============================================================================
# glfw
find_package(glfw3 3.2 REQUIRED)
#find_package(OpenGL 3.3 REQUIRED)

# ==============================================================================
# glad, glm
include_directories("3rd/glad/src")
include_directories("3rd/glad/include")
include_directories("3rd")



# ==============================================================================
# ==============================================================================
# m3d base: basic data structure, io...
file(GLOB M3D_BASE_SRCS
        src/0transform.cpp
        src/0Filters.cpp
        src/0m3d.cpp
        src/1Logger.cpp
        src/1FrameReader.cpp)
add_library(m3dbase STATIC ${M3D_BASE_SRCS})
target_include_directories(m3dbase PRIVATE include)

# m3d initialization part before optimization.
file(GLOB M3D_INITIALIZER_SRCS
        src/4OverlapEstimator.cpp
        src/4GraphInitializer.cpp)

# m3d panorama warper after optimization.
file(GLOB M3D_WARPER_SRCS
        3rd/glad/src/glad.c
        src/6OpenglManagerForWarper.cpp
        src/6PanoramaWarper.cpp)

# m3d panorama stitcher.
file(GLOB M3D_STITCHER_SRCS
        src/7PanoramaStitcher.cpp)

# m3d panorama back layer content generator.
file(GLOB M3D_CONTENT_SRCS
        src/8BackLayerContentGenerator.cpp)



# ==============================================================================
# ==============================================================================
# source file include environments.
include_directories("3rd/glad/include")
include_directories("include")
include_directories("3rd/GMS-Feature-Matcher/include")





# ==============================================================================
# ==============================================================================
add_subdirectory("debug/")