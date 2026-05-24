cmake_minimum_required(VERSION 3.10)
project(unitree_im6014_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Debug)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 头文件目录
include_directories(include)

# ================= 1. 核心静态库 =================
add_library(unitree_im6014 STATIC 
    src/crc32.cpp 
    src/im6014_motor.cpp
    src/serial_port.cpp 
)
target_compile_options(unitree_im6014 PRIVATE -Wall -O2)

# ================= 2. C++ 示例程序 =================
add_executable(example_double_im6014 example/example_double_im6014.cpp)
target_link_libraries(example_double_im6014 PRIVATE unitree_im6014 pthread)

# ================= 3. Python 绑定 (pybind11) =================
find_package(pybind11 QUIET)
if(pybind11_FOUND)
    pybind11_add_module(pyim6014 python/pybind_im6014.cpp)
    target_link_libraries(pyim6014 PRIVATE unitree_im6014)
    # 复制 Python 示例到构建目录
    configure_file(example/example_double_im6014.py ${CMAKE_BINARY_DIR}/example_double_im6014.py COPYONLY)
    message(STATUS "Python bindings enabled")
else()
    message(WARNING "pybind11 not found. Python bindings skipped. Install with: sudo apt install pybind11-dev")
endif()