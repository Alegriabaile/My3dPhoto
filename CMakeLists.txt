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
# gco-v3.0
set(gco_lib_srcs
        3rd/gco-v3.0/GCoptimization.cpp
        3rd/gco-v3.0/GCoptimization.h
        3rd/gco-v3.0/LinkedBlockList.cpp
        3rd/gco-v3.0/LinkedBlockList.h
        3rd/gco-v3.0/block.h
        3rd/gco-v3.0/energy.h
        3rd/gco-v3.0/graph.h
        )
add_library(gco ${gco_lib_srcs})
include_directories(3rd/gco-v3.0)


# ==============================================================================
# ==============================================================================
# m3d base: basic data structure, io...
file(GLOB M3D_BASE_SRCS
        src/0transform.cpp
        src/0m3d.cpp
        src/1Logger.cpp
        src/1FrameReader.cpp)
add_library(m3dbase STATIC ${M3D_BASE_SRCS})
target_include_directories(m3dbase PRIVATE include)

# m3d initialization part before optimization.
file(GLOB M3D_INITIALIZER_SRCS
        src/4OverlapEstimator.cpp
        src/4GraphInitializer.cpp)

# glad.c
file(GLOB M3D_GLAD_SRCS
        3rd/glad/src/glad.c)

# m3d panorama capturer after optimization.
file(GLOB M3D_PANOCAPTURER_SRCS
        src/6CubemapCapturer.cpp
        src/6Cubemap2Sphere.cpp
        src/6PanoramaCapturer.cpp)

# m3d panorama stitcher.
file(GLOB M3D_STITCHER_SRCS
        src/7GcoStitchingSolver.cpp
        src/7PanoramaStitcher.cpp)

# m3d panorama back layer content generator.
file(GLOB M3D_CONTENT_SRCS
        src/8BackLayerContentGenerator.cpp)

# m3d 3d viewer.
file(GLOB M3D_3DVIEWER_SRCS
        src/9Viewer.cpp)


# ==============================================================================
# ==============================================================================
# source file include environments.
include_directories("3rd/glad/include")
include_directories("include")
include_directories("3rd/GMS-Feature-Matcher/include")





# ==============================================================================
# ==============================================================================
add_subdirectory("debug/")



# ==============================================================================
# ==============================================================================
# trial. including robust initial pose estimation, stitching, and novel view view content syn.
option(BUILD_TRIAL "build trial parts." ON)

if(BUILD_TRIAL)
    add_subdirectory("trial")

endif(BUILD_TRIAL)
