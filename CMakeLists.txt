cmake_minimum_required(VERSION 3.0)

# Project Name
set(PROJ_NAME Test)
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

set (SERIAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc/serial/src)
set (CAMERA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc/mindVisionApi/src)
set (INFERENCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc/inference_engine)
set (GFLAGS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc/gflags)

FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/src/*.h")
FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(${PROJ_NAME} ${SOURCE_LIST} ${HEADER_LIST})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJ_NAME})

# OpenCV
#set(OpenCV_DIR "C:/ST42Data/Code/opencv3Src/install/x86/vc15/lib") # 更改此路径为OpenCVConfig.cmake的路径（库目录）
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJ_NAME} ${OpenCV_LIBS})
endif()

# PCL
find_package(PCL 1.8.1 REQUIRED)
 
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
    target_link_libraries(${PROJ_NAME} ${PCL_LIBRARIES})
endif()

IF(UNIX)
	# realsense2 unix
	find_package(realsense2 REQUIRED)
	if(realsense2_FOUND)
            message(STATUS "realsense2 found")
	    include_directories(${realsense2_INCLUDE_DIRS})
	    target_link_libraries(${PROJ_NAME} ${realsense2_LIBRARY})
	endif()
        
        #cuda
        FIND_PACKAGE(CUDA REQUIRED)
        if(CUDA_FOUND)
		message(STATUS "CUDA found")
           	include_directories(${CUDA_INCLUDE_DIRS})
		target_link_libraries(${PROJ_NAME} ${CUDA_LIBRARIES})
	endif()

        #object inc
	include_directories(${PROJECT_INCLUDE_DIR}/util)
	include_directories(${PROJECT_BINARY_DIR}/util)
	include_directories(
	    ${CMAKE_CURRENT_SOURCE_DIR}/inc/tensorRT4/ARM64/include
	    ${SERIAL_DIR}
	    ${CAMERA_DIR}
	    ${CMAKE_CURRENT_SOURCE_DIR}/inc/tensorRTplugin
	    ${CMAKE_CURRENT_SOURCE_DIR}/inc/util
	    ${CUDA_INCLUDE_DIRS}
	    ${INFERENCE_DIR}
	    ${GFLAGS_DIR}/include
	    ${CMAKE_CURRENT_SOURCE_DIR}/inc/mindVisionApi/include
	    )
	#object lib
	target_link_libraries(${PROJ_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/inc/tensorRT4/ARM64/lib/libnvcaffe_parser.so)
	target_link_libraries(${PROJ_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/inc/tensorRT4/ARM64/lib/libnvinfer.so)
	target_link_libraries(${PROJ_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/inc/tensorRT4/ARM64/lib/libnvinfer_plugin.so)
	target_link_libraries(${PROJ_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/inc/tensorRT4/ARM64/lib/libnvparsers.so)

	target_link_libraries(${PROJ_NAME}
		${OpenCV_LIBS}
		${CMAKE_CURRENT_SOURCE_DIR}/lib/libinference.so
		${CMAKE_CURRENT_SOURCE_DIR}/lib/libserial.so
		${CMAKE_CURRENT_SOURCE_DIR}/lib/libcamera.so
		${GFLAGS_DIR}/lib/libgflags_nothreads.a
		${CUDA_LIBRARIES}
	)
ELSE(WIN32)
	# Realsense D435
	set(RealSense_DIR "E:/CodeLibrary/LibRealsense/librealsense-2.16.5") # 更改此路径为 librealsense SDK 安装目录
	set(RealSense_INCLUDE_DI "${RealSense_DIR}/include/librealsense2")
	set(RealSense_INCLUDE_DIR "${RealSense_DIR}/include")
	set(RealSense_LIB "${RealSense_DIR}/lib/realsense2.lib")
        include_directories(${PROJ_NAME} ${RealSense_INCLUDE_DIR} ${RealSense_INCLUDE_DI})
        target_link_libraries(${PROJ_NAME} ${RealSense_LIB})
ENDIF()

