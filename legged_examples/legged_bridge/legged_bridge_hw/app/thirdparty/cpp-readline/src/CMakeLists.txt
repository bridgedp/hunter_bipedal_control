#    /cpp-readline/src/CMakeLists.txt
#
#    @author zmij
#    @date May 17, 2016

cmake_minimum_required(VERSION 2.6)

set(cpp_readline_SRCS
    Console.cpp
)

add_library(${lib_name} SHARED ${cpp_readline_SRCS})
set_target_properties(
    ${lib_name} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)
target_link_libraries(${lib_name} ${Readline_LIBRARY})
