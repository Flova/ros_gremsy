cmake_minimum_required(VERSION 2.8.3)
project(gremsy_base)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_geometry_msgs
        dynamic_reconfigure
        std_msgs
        geometry_msgs
        sensor_msgs
        )

generate_dynamic_reconfigure_options(
        cfg/ROSGremsy.cfg
)

catkin_package(
        INCLUDE_DIRS include
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

include_directories(
        src/gSDK_Linux/src/mavlink/include/mavlink/v2.0
        src/gSDK_Linux/src/
)

set(SOURCES src/ros_gremsy.cpp src/gSDK_Linux/src/serial_port.cpp src/gSDK_Linux/src/gimbal_interface.cpp)

add_executable(GimbalNode ${SOURCES})

add_dependencies(GimbalNode ${PROJECT_NAME}_gencfg)

target_link_libraries(GimbalNode ${catkin_LIBRARIES})
