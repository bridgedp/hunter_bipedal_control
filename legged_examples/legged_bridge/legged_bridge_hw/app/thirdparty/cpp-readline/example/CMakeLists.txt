#    /cpp-readline/example/CMakeLists.txt
#
#    @author zmij
#    @date May 17, 2016

cmake_minimum_required(VERSION 2.6)

add_executable(cpp-readline-example main.cpp)
target_link_libraries(cpp-readline-example ${lib_name})
