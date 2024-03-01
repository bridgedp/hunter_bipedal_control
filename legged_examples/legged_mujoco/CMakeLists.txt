cmake_minimum_required(VERSION 3.10)
project(legged_mujoco)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(LCM_INCLUDE_DIRS "/usr/local/lcm")
set(LCM_LIBRARIES "/usr/local/lib/liblcm.so")

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
        # dynamic_reconfigure
        lcm_msg
        )
find_package(Threads REQUIRED)

# generate_dynamic_reconfigure_options(
#         cfg/touch.cfg
# )


catkin_package(
        INCLUDE_DIRS
                include
        LIBRARIES
                ${PROJECT_NAME}
        CATKIN_DEPENDS
                roscpp
                legged_hw
                # dynamic_reconfigure
                lcm_msg
)





###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${LCM_INCLUDE_DIRS}
        ${Threads_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#         # src/LeggedHW.cpp
#         # src/LeggedHWLoop.cpp
#         )

add_executable(${PROJECT_NAME}
        src/LeggedMujocoSim.cpp
        src/legged_mojuco_sim.cpp
        src/mujoco_lcm/MujocoLcm.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${LCM_LIBRARIES}
        ${Threads_LIBRARIES}
        
        )
        

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
