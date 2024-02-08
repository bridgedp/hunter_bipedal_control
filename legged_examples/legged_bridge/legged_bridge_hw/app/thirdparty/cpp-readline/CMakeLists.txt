#    CMakeLists.txt for cpp-readline library
#
#    @author zmij
#    @date Nov 30, 2015

cmake_minimum_required(VERSION 2.6)

# Set library name here
set(lib_name cpp-readline)
string(TOUPPER ${lib_name} LIB_NAME)

set(_pname ${lib_name})
if (PROJECT_VERSION)
    set(_pversion ${PROJECT_VERSION})
else()
    set(_pversion 0.1.0)
endif()

if (${CMAKE_VERSION} VERSION_GREATER "3.0")
    cmake_policy(SET CMP0048 NEW)
    project(${_pname} VERSION ${_pversion})
else()
    project(${_pname})
    set(PROJECT_VERSION ${_pversion})
endif()

#dependencies
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/modules" )

find_package(Readline REQUIRED)

#options
option(BUILD_EXAMPLES "Build example application" ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${Readline_INCLUDE_DIRS}
)
add_definitions("-std=c++11")
add_definitions(-Wall -Werror -pedantic -Weffc++)

set(${LIB_NAME}_LIB ${lib_name})

# Add subdirectories here
add_subdirectory(src)
if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()

get_directory_property(has_parent PARENT_DIRECTORY)
if (has_parent)
    set(${LIB_NAME}_LIB ${lib_name} CACHE INTERNAL "Name of cpp-readline library target")
    set(${LIB_NAME}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src CACHE INTERNAL "Path to cpp-readline libaray includes" )
endif()
