cmake_minimum_required(VERSION 3.1)
project(rbdltest)
add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
include_directories ( ${EIGEN3_INCLUDE_DIRS} )

include_directories ("${PROJECT_SOURCE_DIR}/")
include_directories ("${PROJECT_SOURCE_DIR}/lib/")

FILE (GLOB SRC_URDF lib/urdf/*.cpp)
add_library(URDF ${SRC_URDF})

FILE (GLOB SRC_COMMON common_utils/*.cpp)
add_library(COMMON ${SRC_COMMON})

FILE (GLOB SRC_RBDL lib/rbdl/*.cc)
add_library(RBDL ${SRC_RBDL})
target_link_libraries(RBDL URDF COMMON)

add_executable(puma puma.cpp)
target_link_libraries(puma RBDL)

# FILE (GLOB SRC_ROBOT_MODEL model/*.cpp)
# add_library(RobotModel ${SRC_ROBOT_MODEL})
# target_link_libraries(RobotModel RBDL)

# add_executable(rbdltest demo.cpp)
# target_link_libraries(rbdltest RobotModel)

# add_executable(get_jointpos test_get_jointpos.cpp)
# target_link_libraries(get_jointpos RobotModel)

# add_executable(rbdltest MAIN.cpp)
# target_link_libraries(rbdltest RobotModel)
