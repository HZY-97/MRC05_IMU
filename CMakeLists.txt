cmake_minimum_required(VERSION 3.8)
project(SGPM01_imu)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -pthread")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS "-O0 -Wall -Wextra -g -pthread")
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_PATCH_VERSION 1)
set(${PROJECT_NAME}_VERSION
  ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  ${std_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/includ
  ${tf2_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

# for rclcpp
find_package(rclcpp REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})


# for glog
find_package(glog REQUIRED)
# message("_--------------${GLOG_INCLUDE_DIRS}--------------------------------")

add_executable(sgp_imu src/sgp_imu_node.cpp)
target_link_libraries(sgp_imu 
 ${rclcpp_LIBRARIES}
 ${cpp_typesupport_target}
 ${geometry_msgs_LIBRARIES}
 ${sensor_msgs_LIBRARIES})

install(
    TARGETS sgp_imu
    DESTINATION bin
)

set_target_properties(
    sgp_imu
    PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)


execute_process(
    COMMAND tar --exclude=build 
                --exclude=.gitignore 
                --exclude=.cache
                --exclude=.git
                --exclude=.tag*
                --exclude=install
                --exclude=.vscode 
                --exclude=3dNavBlackBox 
                -czf ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_SourceCode_v${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION}.tar.gz 
                SGPM01_imu/
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../
)

# CPack
set(CPACK_GENERATOR "ZIP")  # ZIP TGZ DEB RPM
set(CPACK_PACKAGE_NAME "SGPM01_imu")
set(CPACK_PACKAGE_VERSION_MAJOR ${${PROJECT_NAME}_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${${PROJECT_NAME}_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${${PROJECT_NAME}_PATCH_VERSION})
set(CPACK_PACKAGE_DESCRIPTION "SIASUN")

include(CPack)

