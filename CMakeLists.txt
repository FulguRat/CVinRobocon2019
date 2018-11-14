cmake_minimum_required(VERSION 3.0)

# Project Name
set(PROJ_NAME RoboconCV)
project(${PROJ_NAME})

# Check if c++11 is available
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Binary output path
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/src/*.h")
FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(${PROJ_NAME} ${SOURCE_LIST} ${HEADER_LIST})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJ_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "/wd4819")

# OpenCV
#set(OpenCV_DIR "C:/ST42Data/Code/opencv3Src/install/x86/vc15/lib") # 更改此路径为OpenCVConfig.cmake的路径（库目录）
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJ_NAME} ${OpenCV_LIBS})
endif()

# PCL
find_package(PCL 1.8 REQUIRED)
 
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
    target_link_libraries(${PROJ_NAME} ${PCL_LIBRARIES})
endif()

# Realsense D435
set(RealSense_DIR "C:/ST42Data/Code/librealsense/install") # 更改此路径为 librealsense SDK 安装目录

set(RealSense_INCLUDE_DIR "${RealSense_DIR}/include")
set(RealSense_LIB "${RealSense_DIR}/lib/realsense2.lib")

include_directories(${PROJ_NAME} ${RealSense_INCLUDE_DIR})
target_link_libraries(${PROJ_NAME} ${RealSense_LIB})
