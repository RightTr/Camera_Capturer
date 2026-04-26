#include <array>
#include <memory>

#include <cv_bridge/cv_bridge.h>

#ifdef USE_ROS1
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#endif

#define guide_consumer camera_rgbdt_file_guide_consumer
#define realsense_consumer camera_rgbdt_file_realsense_consumer
#define imu_consumer camera_rgbdt_file_imu_consumer
#define main camera_rgbdt_standalone_main
#include "camera_RGBDT.cpp"
#undef main
#undef imu_consumer
#undef realsense_consumer
#undef guide_consumer

#ifdef USE_ROS1
using ImagePublisher = ros::Publisher;
using ImuPublisher = ros::Publisher;
#elif defined(USE_ROS2)
using ImagePublisher = rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr;
using ImuPublisher = rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr;
#endif

std::array<ImagePublisher, 2> g_guide_image_pubs;
std::array<ImagePublisher, 2> g_guide_temp_pubs;
ImagePublisher g_rs_rgb_pub;
ImagePublisher g_rs_depth_pub;
ImuPublisher g_rs_accel_pub;
ImuPublisher g_rs_gyro_pub;

template <typename CovarianceArray>
void mark_covariance_unavailable(CovarianceArray& covariance)
{
    std::fill(std::begin(covariance), std::end(covariance), 0.0);
    covariance[0] = -1.0;
}

template <typename CovarianceArray>
void mark_covariance_unknown(CovarianceArray& covariance)
{
    std::fill(std::begin(covariance), std::end(covariance), 0.0);
}

inline bool ros_ok()
{
#ifdef USE_ROS1
    return ros::ok();
#elif defined(USE_ROS2)
    return rclcpp::ok();
#else
    return true;
#endif
}

#ifdef USE_ROS1
ros::Time to_ros_time(long sec, long nsec)
{
    return ros::Time(sec, nsec);
}

ros::Time to_ros_time_sec_usec(long sec, long usec)
{
    return ros::Time(sec, usec * 1000L);
}

ros::Time to_ros_time_ns(uint64_t ns)
{
    ros::Time stamp;
    stamp.fromNSec(ns);
    return stamp;
}

void publish_image(const ros::Publisher& pub, const cv::Mat& image, const std::string& encoding,
                   const std::string& frame_id, const ros::Time& stamp)
{
    auto msg = cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();
    msg->header.frame_id = frame_id;
    msg->header.stamp = stamp;
    pub.publish(msg);
}

void publish_imu(const ros::Publisher& pub, const StampedImuFrame& frame, const std::string& frame_id)
{
    sensor_msgs::Imu msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = to_ros_time_ns(frame.sensor_ns);
    mark_covariance_unavailable(msg.orientation_covariance);
    if (frame.stream_type == RS2_STREAM_ACCEL) {
        msg.linear_acceleration.x = frame.x;
        msg.linear_acceleration.y = frame.y;
        msg.linear_acceleration.z = frame.z;
        mark_covariance_unknown(msg.linear_acceleration_covariance);
        mark_covariance_unavailable(msg.angular_velocity_covariance);
    } else {
        msg.angular_velocity.x = frame.x;
        msg.angular_velocity.y = frame.y;
        msg.angular_velocity.z = frame.z;
        mark_covariance_unknown(msg.angular_velocity_covariance);
        mark_covariance_unavailable(msg.linear_acceleration_covariance);
    }
    pub.publish(msg);
}
#elif defined(USE_ROS2)
rclcpp::Time to_ros_time(long sec, long nsec)
{
    return rclcpp::Time(sec, nsec, RCL_SYSTEM_TIME);
}

rclcpp::Time to_ros_time_sec_usec(long sec, long usec)
{
    return rclcpp::Time(sec, usec * 1000L, RCL_SYSTEM_TIME);
}

rclcpp::Time to_ros_time_ns(uint64_t ns)
{
    return rclcpp::Time(static_cast<int64_t>(ns), RCL_SYSTEM_TIME);
}

void publish_image(const ImagePublisher& pub, const cv::Mat& image, const std::string& encoding,
                   const std::string& frame_id, const rclcpp::Time& stamp)
{
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg();
    msg->header.frame_id = frame_id;
    msg->header.stamp = stamp;
    pub->publish(*msg);
}

void publish_imu(const ImuPublisher& pub, const StampedImuFrame& frame, const std::string& frame_id)
{
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = to_ros_time_ns(frame.sensor_ns);
    mark_covariance_unavailable(msg.orientation_covariance);
    if (frame.stream_type == RS2_STREAM_ACCEL) {
        msg.linear_acceleration.x = frame.x;
        msg.linear_acceleration.y = frame.y;
        msg.linear_acceleration.z = frame.z;
        mark_covariance_unknown(msg.linear_acceleration_covariance);
        mark_covariance_unavailable(msg.angular_velocity_covariance);
    } else {
        msg.angular_velocity.x = frame.x;
        msg.angular_velocity.y = frame.y;
        msg.angular_velocity.z = frame.z;
        mark_covariance_unknown(msg.angular_velocity_covariance);
        mark_covariance_unavailable(msg.linear_acceleration_covariance);
    }
    pub->publish(msg);
}
#endif

void guide_consumer(int id)
{
    while (!quitFlag.load()) {
        StampedGuideFrame frame;
        {
            std::unique_lock<std::mutex> lock(lr_queue_mutex[id]);
            lr_cv[id].wait(lock, [&]{ return !lr_output_queue[id].empty() || quitFlag.load(); });
            if (quitFlag.load()) break;
            frame = lr_output_queue[id].front();
            lr_output_queue[id].pop();
        }
        lr_cv[id].notify_one();
        if (if_save) save_guide_frame(frame);

        const auto stamp = to_ros_time_sec_usec(frame.sensor_sec, frame.sensor_microsec);
        const std::string frame_id = (id == 0) ? "guide_left" : "guide_right";
        publish_image(g_guide_image_pubs[id], frame.gray_image, "mono8", frame_id, stamp);
        publish_image(g_guide_temp_pubs[id], frame.temperature_celsius, "32FC1", frame_id, stamp);
    }
}

void realsense_consumer()
{
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        {
            std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
            rgbd_cv.wait(lock, [&]{ return !rgbd_output_queue.empty() || quitFlag.load(); });
            if (quitFlag.load()) break;
            frame = rgbd_output_queue.front();
            rgbd_output_queue.pop();
        }
        rgbd_cv.notify_one();
        if (if_save) save_realsense_frame(frame);

        const auto stamp = to_ros_time_sec_usec(frame.sensor_sec, frame.sensor_microsec);
        publish_image(g_rs_rgb_pub, frame.color_image, "bgr8", "realsense_color", stamp);
        publish_image(g_rs_depth_pub, frame.depth_image_raw, "16UC1", "realsense_depth", stamp);
    }
}

void imu_consumer()
{
    while (!quitFlag.load()) {
        StampedImuFrame frame;
        {
            std::unique_lock<std::mutex> lock(imu_queue_mutex);
            imu_cv.wait(lock, [&]{ return !imu_output_queue.empty() || quitFlag.load(); });
            if (quitFlag.load()) {
                while (!imu_output_queue.empty()) {
                    if (if_save) save_imu_frame(imu_output_queue.front());
                    imu_output_queue.pop();
                }
                break;
            }
            frame = imu_output_queue.front();
            imu_output_queue.pop();
        }
        imu_cv.notify_one();
        if (if_save) save_imu_frame(frame);

        if (frame.stream_type == RS2_STREAM_ACCEL) {
            publish_imu(g_rs_accel_pub, frame, "realsense_accel");
        } else if (frame.stream_type == RS2_STREAM_GYRO) {
            publish_imu(g_rs_gyro_pub, frame, "realsense_gyro");
        }
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int trigger_fps = 30;
    outputdir = "/data/home/pi/Cap";

#ifdef USE_ROS1
    ros::init(argc, argv, "camera_rgbdt_node");
    ros::NodeHandle nh;
    nh.param("sync_enable", rs_enable, 1);
    nh.param("if_save", if_save, 0);
    nh.param("temp_incre_detect", tempIncre_detect, 0);
    nh.param<std::string>("output_dir", outputdir, "/data/home/pi/Cap");
    g_guide_image_pubs[0] = nh.advertise<sensor_msgs::Image>("guide_left/image", 5);
    g_guide_image_pubs[1] = nh.advertise<sensor_msgs::Image>("guide_right/image", 5);
    g_guide_temp_pubs[0] = nh.advertise<sensor_msgs::Image>("guide_left/temperature", 5);
    g_guide_temp_pubs[1] = nh.advertise<sensor_msgs::Image>("guide_right/temperature", 5);
    g_rs_rgb_pub = nh.advertise<sensor_msgs::Image>("realsense/rgb/image", 5);
    g_rs_depth_pub = nh.advertise<sensor_msgs::Image>("realsense/depth_raw/image", 5);
    g_rs_accel_pub = nh.advertise<sensor_msgs::Imu>("realsense/imu/accel", 50);
    g_rs_gyro_pub = nh.advertise<sensor_msgs::Imu>("realsense/imu/gyro", 200);
    auto sync_sub = nh.subscribe<std_msgs::Int32>("guidecam/sync", 1,
        [&](const std_msgs::Int32::ConstPtr& msg) {
            for (int i = 0; i < 2; ++i) {
                serial_cmd[i].store(msg->data ? SerialCmd::SYNC_ON : SerialCmd::SYNC_OFF);
                serial_cv[i].notify_one();
            }
        });
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("camera_rgbdt_node");
    rs_enable = node->declare_parameter<int>("sync_enable", 1);
    if_save = node->declare_parameter<int>("if_save", 0);
    tempIncre_detect = node->declare_parameter<int>("temp_incre_detect", 0);
    outputdir = node->declare_parameter<std::string>("output_dir", "/data/home/pi/Cap");
    g_guide_image_pubs[0] = node->create_publisher<sensor_msgs::msg::Image>("guide_left/image", 5);
    g_guide_image_pubs[1] = node->create_publisher<sensor_msgs::msg::Image>("guide_right/image", 5);
    g_guide_temp_pubs[0] = node->create_publisher<sensor_msgs::msg::Image>("guide_left/temperature", 5);
    g_guide_temp_pubs[1] = node->create_publisher<sensor_msgs::msg::Image>("guide_right/temperature", 5);
    g_rs_rgb_pub = node->create_publisher<sensor_msgs::msg::Image>("realsense/rgb/image", 5);
    g_rs_depth_pub = node->create_publisher<sensor_msgs::msg::Image>("realsense/depth_raw/image", 5);
    g_rs_accel_pub = node->create_publisher<sensor_msgs::msg::Imu>("realsense/imu/accel", 50);
    g_rs_gyro_pub = node->create_publisher<sensor_msgs::msg::Imu>("realsense/imu/gyro", 200);
    auto sync_sub = node->create_subscription<std_msgs::msg::Int32>("guidecam/sync", 1,
        [&](const std_msgs::msg::Int32::SharedPtr msg) {
            for (int i = 0; i < 2; ++i) {
                serial_cmd[i].store(msg->data ? SerialCmd::SYNC_ON : SerialCmd::SYNC_OFF);
                serial_cv[i].notify_one();
            }
        });
#endif

    printf("trigger_fps %d, sync_enable %d, if_save %d, tempIncre_detect %d, outputdir %s\n",
           trigger_fps, rs_enable, if_save, tempIncre_detect, outputdir.c_str());

    if (if_save) prepare_dirs(outputdir);

    const char* dev_left = device_path::kLeftCamera;
    const char* dev_right = device_path::kRightCamera;
    const std::string dev_rs(device_path::kRealSenseSerial);

    for (auto& cmd : serial_cmd) cmd.store(SerialCmd::NONE);

    if (init_v4l2cam(dev_left, &lr_fd[0], &lr_buffers[0], 1280, 513) != 0) return EXIT_FAILURE;
    if (init_v4l2cam(dev_right, &lr_fd[1], &lr_buffers[1], 1280, 513) != 0) {
        free(lr_buffers[0]);
        close(lr_fd[0]);
        return EXIT_FAILURE;
    }
    if (open_serial_port(0) < 0 || open_serial_port(1) < 0) {
        free(lr_buffers[0]);
        free(lr_buffers[1]);
        close(lr_fd[0]);
        close(lr_fd[1]);
        return EXIT_FAILURE;
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(lr_fd[0], VIDIOC_STREAMON, &type) < 0 || ioctl(lr_fd[1], VIDIOC_STREAMON, &type) < 0) {
        perror("Starting Capture");
        free(lr_buffers[0]);
        free(lr_buffers[1]);
        close(lr_fd[0]);
        close(lr_fd[1]);
        return EXIT_FAILURE;
    }

    std::vector<std::thread> consumers;
    for (int i = 0; i < 2; ++i) consumers.emplace_back(guide_consumer, i);
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(imu_consumer);

    std::vector<std::thread> producers;
    for (int i = 0; i < 2; ++i) producers.emplace_back(guide_producer, trigger_fps, i);
    producers.emplace_back(realsense_producer, dev_rs);

    std::vector<std::thread> serial_worker_threads;
    for (int i = 0; i < 2; ++i) serial_worker_threads.emplace_back(serial_worker, i);

    std::vector<std::thread> serial_query_threads;
    for (int i = 0; i < 2; ++i) serial_query_threads.emplace_back(serial_query, i);

#ifdef USE_ROS1
    ros::Rate rate(100);
    while (ros::ok() && !quitFlag.load()) {
        ros::spinOnce();
        rate.sleep();
    }
#elif defined(USE_ROS2)
    rclcpp::Rate rate(100);
    while (rclcpp::ok() && !quitFlag.load()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
#endif

    quitFlag.store(true);
    rgbd_cv.notify_all();
    imu_cv.notify_all();
    for (int i = 0; i < 2; ++i) {
        lr_cv[i].notify_all();
        serial_cv[i].notify_all();
        if (serials[i].IsOpen()) serials[i].Close();
    }

    for (auto& t : producers) t.join();
    for (auto& t : consumers) t.join();
    for (auto& t : serial_worker_threads) t.join();
    for (auto& t : serial_query_threads) t.join();

#ifdef USE_ROS2
    rclcpp::shutdown();
#endif
    return EXIT_SUCCESS;
}
