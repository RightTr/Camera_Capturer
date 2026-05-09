cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(camera_capturer)

find_package(OpenCV REQUIRED)
find_package(fastcdr REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(guidemono src/guidemono.cpp src/producer/guide_producer.cpp src/writer/guide_writer.cpp)
target_link_libraries(guidemono
    ${OpenCV_LIBS}
    pthread
    serial
    )
target_include_directories(guidemono
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS}
)

add_executable(guidestereo src/guidestereo.cpp src/producer/guide_producer.cpp src/writer/guide_writer.cpp)
target_link_libraries(guidestereo
    ${OpenCV_LIBS}
    pthread
    serial
    )
target_include_directories(guidestereo
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS}
)

add_executable(realsense src/realsense.cpp src/producer/realsense_producer.cpp)
target_link_libraries(realsense
    ${OpenCV_LIBS}
    pthread
    realsense2::realsense2
    )
target_include_directories(realsense
    PRIVATE ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
)

add_executable(camera_RGBDT
    src/camera_RGBDT.cpp
    src/producer/guide_producer.cpp
    src/producer/realsense_producer.cpp
    src/writer/guide_writer.cpp
    src/writer/realsense_writer.cpp)
target_link_libraries(camera_RGBDT
    ${OpenCV_LIBS}
    pthread
    serial
    realsense2::realsense2
    )

target_include_directories(camera_RGBDT
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
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

    add_executable(guidestereo_node src/ros/guidestereo_ros.cpp src/producer/guide_producer.cpp)
    add_executable(camera_RGBDT_node
        src/ros/camera_RGBDT_ros.cpp
        src/producer/guide_producer.cpp
        src/producer/realsense_producer.cpp
        src/writer/guide_writer.cpp
        src/writer/realsense_writer.cpp)
    target_link_libraries(guidestereo_node
        ${catkin_LIBRARIES}
        pthread
        serial
        )
    target_include_directories(guidestereo_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(camera_RGBDT_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        pthread
        serial
        realsense2::realsense2
    )
    target_include_directories(camera_RGBDT_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
    )

endif()

if(USE_ROS2)
    add_definitions(-DUSE_ROS2)

    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(fastrtps REQUIRED)

    add_executable(guidestereo_node src/ros/guidestereo_ros.cpp src/producer/guide_producer.cpp)
    add_executable(camera_RGBDT_node
        src/ros/camera_RGBDT_ros.cpp
        src/producer/guide_producer.cpp
        src/producer/realsense_producer.cpp
        src/writer/guide_writer.cpp
        src/writer/realsense_writer.cpp)
    ament_target_dependencies(guidestereo_node
        rclcpp
        sensor_msgs
        std_msgs
        cv_bridge
    )
    ament_target_dependencies(camera_RGBDT_node
        rclcpp
        sensor_msgs
        std_msgs
    )
    ament_target_dependencies(realsense
        rclcpp
        sensor_msgs
        std_msgs
        cv_bridge
    )
    ament_target_dependencies(camera_RGBDT
        rclcpp
        std_msgs
    )
    target_link_libraries(guidestereo_node
        pthread
        serial
        )
    target_include_directories(guidestereo_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(camera_RGBDT_node
        ${OpenCV_LIBS}
        pthread
        serial
        realsense2::realsense2
    )
    target_include_directories(camera_RGBDT_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
    )
    install(TARGETS guidestereo_node camera_RGBDT_node
        DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
    install(DIRECTORY rviz_cfg
        DESTINATION share/${PROJECT_NAME})
    ament_package() # Don't forget this
endif()
