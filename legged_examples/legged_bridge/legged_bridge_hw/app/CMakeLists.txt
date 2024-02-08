cmake_minimum_required(VERSION 3.5)
project(MasterStack)

add_subdirectory(thirdparty)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
include_directories(
        thirdparty/soem/soem
        thirdparty/soem/osal
        thirdparty/soem/osal/linux
        thirdparty/soem/oshw/linux
        thirdparty/cpp-readline/src
)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES motor_control.c
            motor_control.h
            math_ops.c
            math_ops.h
        transmit.cpp
            transmit.h
            config.h
        queue.h command.cpp command.h)
LINK_LIBRARIES(m)
add_library(motor ${SOURCES})
target_link_libraries(motor soem)

add_executable(master_stack_test main.cpp)
target_link_libraries(master_stack_test motor soem cpp-readline)