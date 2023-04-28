cmake_minimum_required(VERSION 3.10)
project(Test)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14 -pthread")

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

#引入头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 海康威视的头文件 和 库文件
include_directories(/opt/MVS/include/)
link_directories(/opt/MVS/lib/64/)

add_executable(Test main.cpp
        ${PROJECT_SOURCE_DIR}/camera/Mycamera.cpp
        ${PROJECT_SOURCE_DIR}/serial/serial_port.cpp
        ${PROJECT_SOURCE_DIR}/serial/general.cpp
        ${PROJECT_SOURCE_DIR}/yuan/yuan.cpp
        )

#把库包含进来
target_link_libraries(Test ${OpenCV_LIBRARIES} 
                               libMvCameraControl.so                                
                                libpthread.so
                                MVSDK
                                )
# libpthread.so 与 MVSDK 是迈德卫视的链接库
# libMvCameraControl.so 海康威视需要的库文件

#target_include_directories(Test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
