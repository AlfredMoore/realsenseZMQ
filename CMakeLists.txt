cmake_minimum_required(VERSION 3.16)
project(REALSENSE_ZMQ_CLIENT LANGUAGES CXX)

# ---- C++ 标准与编译选项 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 常用告警
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# fetch latest argparse
include(FetchContent)
FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
)
FetchContent_MakeAvailable(argparse)

# ---- 依赖：RealSense2 ----
find_package(realsense2 REQUIRED)  # 提供 target: realsense2::realsense2

# ---- 依赖：OpenCV ----
# 需要用到 core,imgcodecs,highgui（若你不显示就可以去掉 highgui）
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc highgui)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ---- 依赖：ZeroMQ (libzmq + cppzmq 头文件) ----
# cppzmq 是头文件；真正需要链接的是 libzmq
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(ZMQ QUIET libzmq)
endif()

# 若 pkg-config 没找到，再直接搜库
if(NOT ZMQ_FOUND)
  find_path(ZMQ_INCLUDE_DIR NAMES zmq.hpp)
  find_library(ZMQ_LIBRARY NAMES zmq libzmq)
  if(ZMQ_INCLUDE_DIR AND ZMQ_LIBRARY)
    set(ZMQ_FOUND TRUE)
    set(ZMQ_INCLUDE_DIRS ${ZMQ_INCLUDE_DIR})
    set(ZMQ_LIBRARIES ${ZMQ_LIBRARY})
  endif()
endif()

if(NOT ZMQ_FOUND)
  message(FATAL_ERROR "ZeroMQ not found. Please install libzmq and cppzmq headers.
  Ubuntu:
    sudo apt-get update
    sudo apt-get install -y libzmq3-dev cppzmq")
endif()

# ---- 目标 ----
add_executable(rs_zmq_publisher
  main.cpp     # 把你的源文件名改成实际文件名
)

target_include_directories(rs_zmq_publisher
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
)

target_link_libraries(rs_zmq_publisher
  PRIVATE
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${ZMQ_LIBRARIES}
    argparse
)

# Linux 上通常需要 pthread（ZeroMQ/RealSense 常见）
if(UNIX AND NOT APPLE)
  target_link_libraries(rs_zmq_publisher PRIVATE pthread)
endif()

# 可选：更易于调试的定义
target_compile_definitions(rs_zmq_publisher PRIVATE
  $<$<CONFIG:Debug>:DEBUG>
)

