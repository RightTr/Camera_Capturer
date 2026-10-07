cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(camera_capturer)

include(CTest)
if(BUILD_TESTING)
    add_executable(imu_interpolation_test tests/imu_interpolation_test.cpp)
    target_include_directories(imu_interpolation_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_test(NAME imu_interpolation_test COMMAND imu_interpolation_test)
    add_executable(stereo_pair_buffer_test tests/stereo_pair_buffer_test.cpp)
    target_include_directories(stereo_pair_buffer_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_test(NAME stereo_pair_buffer_test COMMAND stereo_pair_buffer_test)
    add_executable(trigger_slot_test tests/trigger_slot_test.cpp)
    target_include_directories(trigger_slot_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_test(NAME trigger_slot_test COMMAND trigger_slot_test)
    add_executable(vio_order_test tests/vio_order_test.cpp)
    target_include_directories(vio_order_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_test(NAME vio_order_test COMMAND vio_order_test)
    add_executable(imu_continuity_test tests/imu_continuity_test.cpp)
    target_include_directories(imu_continuity_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_test(NAME imu_continuity_test COMMAND imu_continuity_test)
endif()

find_package(OpenCV REQUIRED)
find_package(fastcdr REQUIRED)
find_package(realsense2 REQUIRED)
find_library(GPIOD_LIB gpiod)

if(NOT GPIOD_LIB)
    message(FATAL_ERROR "libgpiod not found")
endif()

add_library(sync_bridge src/sync_bridge/sync_bridge.cpp)
target_link_libraries(sync_bridge
    pthread
    serial
    ${GPIOD_LIB}
)
target_include_directories(sync_bridge
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

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

    add_executable(guidestereo_node
        src/ros/guidestereo_ros.cpp
        src/producer/guide_producer.cpp
        src/writer/guide_writer.cpp)
    add_executable(guidestereo_trigger_node
        src/ros/guidestereo_trigger_ros.cpp
        src/producer/guide_producer.cpp
        src/writer/guide_writer.cpp)
    add_executable(realsense_node
        src/ros/realsense_ros.cpp
        src/producer/realsense_producer.cpp
        src/writer/realsense_writer.cpp)
    add_executable(camera_RGBDT_node
        src/ros/camera_RGBDT_ros.cpp
        src/producer/guide_producer.cpp
        src/producer/realsense_producer.cpp
        src/writer/guide_writer.cpp
        src/writer/realsense_writer.cpp)
    add_executable(rgbdt_trigger_node
        src/ros/RGBDT_trigger_ros.cpp
        src/producer/guide_producer.cpp
        src/producer/realsense_producer.cpp
        src/writer/guide_writer.cpp
        src/writer/realsense_writer.cpp)
    target_link_libraries(guidestereo_node
        ${catkin_LIBRARIES}
        pthread
        serial
        )
    target_link_libraries(guidestereo_trigger_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        pthread
        serial
        sync_bridge
        )
    target_include_directories(guidestereo_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS}
    )
    target_include_directories(guidestereo_trigger_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(camera_RGBDT_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        pthread
        serial
        realsense2::realsense2
    )
    target_link_libraries(rgbdt_trigger_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        pthread
        serial
        sync_bridge
        realsense2::realsense2
    )
    target_link_libraries(realsense_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        pthread
        realsense2::realsense2
    )
    target_include_directories(camera_RGBDT_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
    )
    target_include_directories(rgbdt_trigger_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
    )
    target_include_directories(realsense_node
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

    add_executable(guidestereo_node
        src/ros/guidestereo_ros.cpp
        src/producer/guide_producer.cpp
        src/writer/guide_writer.cpp)
    add_executable(guidestereo_trigger_node
        src/ros/guidestereo_trigger_ros.cpp
        src/producer/guide_producer.cpp
        src/writer/guide_writer.cpp)
    add_executable(realsense_node
        src/ros/realsense_ros.cpp
        src/producer/realsense_producer.cpp
        src/writer/realsense_writer.cpp)
    add_executable(camera_RGBDT_node
        src/ros/camera_RGBDT_ros.cpp
        src/producer/guide_producer.cpp
        src/producer/realsense_producer.cpp
        src/writer/guide_writer.cpp
        src/writer/realsense_writer.cpp)
    add_executable(rgbdt_trigger_node
        src/ros/RGBDT_trigger_ros.cpp
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
    ament_target_dependencies(guidestereo_trigger_node
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
    ament_target_dependencies(rgbdt_trigger_node
        rclcpp
        sensor_msgs
        std_msgs
    )
    ament_target_dependencies(realsense_node
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
    target_link_libraries(guidestereo_trigger_node
        ${OpenCV_LIBS}
        pthread
        serial
        sync_bridge
        )
    target_include_directories(guidestereo_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS}
    )
    target_include_directories(guidestereo_trigger_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(camera_RGBDT_node
        ${OpenCV_LIBS}
        pthread
        serial
        realsense2::realsense2
    )
    target_link_libraries(rgbdt_trigger_node
        ${OpenCV_LIBS}
        pthread
        serial
        sync_bridge
        realsense2::realsense2
    )
    target_link_libraries(realsense_node
        ${OpenCV_LIBS}
        pthread
        realsense2::realsense2
    )
    target_include_directories(camera_RGBDT_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
    )
    target_include_directories(rgbdt_trigger_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
    )
    target_include_directories(realsense_node
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
    )
    install(TARGETS guidestereo_node guidestereo_trigger_node realsense_node camera_RGBDT_node rgbdt_trigger_node
        DESTINATION lib/${PROJECT_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
        install(DIRECTORY launch
            DESTINATION share/${PROJECT_NAME})
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz_cfg")
        install(DIRECTORY rviz_cfg
            DESTINATION share/${PROJECT_NAME})
    endif()
    if(BUILD_TESTING)
        get_target_property(recovery_sources rgbdt_trigger_node SOURCES)
        list(REMOVE_ITEM recovery_sources src/ros/RGBDT_trigger_ros.cpp)
        add_executable(sync_recovery_test tests/sync_recovery_test.cpp ${recovery_sources})
        target_include_directories(sync_recovery_test PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS})
        target_compile_options(sync_recovery_test PRIVATE -UNDEBUG)
        ament_target_dependencies(sync_recovery_test rclcpp sensor_msgs std_msgs)
        target_link_libraries(sync_recovery_test ${OpenCV_LIBS} pthread serial
            sync_bridge realsense2::realsense2)
        add_test(NAME sync_recovery_test COMMAND sync_recovery_test)
        set_tests_properties(sync_recovery_test PROPERTIES TIMEOUT 30)
    endif()
    ament_package() # Don't forget this
endif()
