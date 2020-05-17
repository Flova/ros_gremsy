cmake_minimum_required(VERSION 2.8.3)
project(ros_gremsy)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        dynamic_reconfigure
        std_msgs
        visualization_msgs
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
        gSDK_Linux/mavlink/include/mavlink/v2.0
)

set(SOURCES gSDK_Linux/mavlink_control.cpp gSDK_Linux/serial_port.cpp gSDK_Linux/gimbal_interface.cpp)

add_executable(GimbalNode ${SOURCES})

add_dependencies(GimbalNode ${PROJECT_NAME}_gencfg)

target_link_libraries(GimbalNode ${catkin_LIBRARIES})
