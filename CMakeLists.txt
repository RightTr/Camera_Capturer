cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(guidecam)

find_package(OpenCV REQUIRED)

include_directories(
    include
    )

add_executable(guidemono src/guidemono.cpp)
target_link_libraries(guidemono
    ${OpenCV_LIBS}
    pthread
    serial
    )

add_executable(guidestereo src/guidestereo.cpp)
target_link_libraries(guidestereo
    ${OpenCV_LIBS}
    pthread
    serial
    )

if(DEFINED ENV{ROS_PACKAGE_PATH})
    message(STATUS "ROS environment detected, adding ROS dependencies")

    find_package(catkin QUIET)
    if(catkin_FOUND)
    message(STATUS "ROS1 detected")
    set(USE_ROS1 ON)
    endif()

    find_package(amemt_cmake QUIET)
    if(ament_cmake_FOUND)
    message(STATUS "ROS2 detected")
    set(USE_ROS2 ON)
    endif()

    if(USE_ROS1)
        add_definitions(-DUSE_ROS1)

        find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            sensor_msgs
            cv_bridge
            )

        include_directories(
            ${catkin_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
            )

        add_executable(guidestereo_node src/guidestereo_ros.cpp)
        target_link_libraries(guidestereo_node
            ${catkin_LIBRARIES}
            pthread
            serial
            )
    endif()

    if(USE_ROS2)
        add_definitions(-DUSE_ROS2)

        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(cv_bridge REQUIRED)

        include_directories(
            ${rclcpp_INCLUDE_DIRS}
            ${std_msgs_INCLUDE_DIRS}
            ${sensor_msgs_INCLUDE_DIRS}
            ${cv_bridge_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
            )

        add_executable(guidestereo_node src/guidestereo_ros.cpp)
        target_link_libraries(guidestereo_node
            ${rclcpp_LIBRARIES}
            ${std_msgs_LIBRARIES}
            ${sensor_msgs_LIBRARIES}
            ${cv_bridge_LIBRARIES}
            pthread
            serial
            )
    endif()

endif()