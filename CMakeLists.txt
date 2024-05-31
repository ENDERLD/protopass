cmake_minimum_required(VERSION 3.22)
project(protopass)

set(CMAKE_CXX_STANDARD 17)
add_subdirectory(autonomous_proto)
#add_subdirectory(mw_api)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
if(GeographicLib_FOUND)
    message(STATUS "GeographicLib found!")
else()
    message(FATAL_ERROR "GeographicLib not found!")
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
)
add_executable(protopass main.cpp)

ament_target_dependencies(protopass
        rclcpp
        rosbag2_cpp
        std_msgs
)

target_link_libraries(protopass
        autonomous_proto
#        mw_api
        ${GeographicLib_LIBRARIES}
#        yaml-cpp
#        dl
)
install(TARGETS
        protopass
        DESTINATION lib/${PROJECT_NAME}
)
