cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(camera_capturer)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

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

add_executable(realsense src/realsense.cpp)
target_link_libraries(realsense
    ${OpenCV_LIBS}
    pthread
    realsense2::realsense2
    )
target_include_directories(realsense
    PRIVATE ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
)

add_executable(camera_RGBDT src/camera_RGBDT.cpp)
target_link_libraries(camera_RGBDT
    ${OpenCV_LIBS}
    pthread
    serial
    realsense2::realsense2
    )

target_include_directories(camera_RGBDT
    PRIVATE ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
)

find_package(catkin QUIET)
if(catkin_FOUND)
message(STATUS "ROS1 detected")
set(USE_ROS1 ON)
endif()

find_package(ament_cmake QUIET)
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
    
    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs
        INCLUDE_DIRS
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
    find_package(sensor_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)

    add_executable(guidestereo_node src/guidestereo_ros.cpp)
    ament_target_dependencies(guidestereo_node
        rclcpp
        sensor_msgs
        cv_bridge
    )
    target_link_libraries(guidestereo_node
        pthread
        serial
        )
    install(TARGETS guidestereo_node
        DESTINATION lib/${PROJECT_NAME})
    ament_package() # Don't forget this
endif()
