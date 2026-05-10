#pragma once

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

#ifdef USE_ROS1
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt64.h>
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#endif

#ifdef USE_ROS1
template <typename MsgT>
using Publisher = ros::Publisher;

template <typename MsgT>
using Subscription = ros::Subscriber;

template <typename MsgT>
using MessageConstPtr = typename MsgT::ConstPtr;

using NodePtr = std::shared_ptr<ros::NodeHandle>;
using Time = ros::Time;
using Rate = ros::Rate;
using ImageMsg = sensor_msgs::Image;
using ImuMsg = sensor_msgs::Imu;
using Int32Msg = std_msgs::Int32;
using UInt64Msg = std_msgs::UInt64;
#elif defined(USE_ROS2)
template <typename MsgT>
using Publisher = typename rclcpp::Publisher<MsgT>::SharedPtr;

template <typename MsgT>
using Subscription = typename rclcpp::Subscription<MsgT>::SharedPtr;

template <typename MsgT>
using MessageConstPtr = typename MsgT::ConstSharedPtr;

using NodePtr = rclcpp::Node::SharedPtr;
using Time = rclcpp::Time;
using Rate = rclcpp::Rate;
using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;
using Int32Msg = std_msgs::msg::Int32;
using UInt64Msg = std_msgs::msg::UInt64;
#endif

inline NodePtr &node_instance()
{
    static NodePtr node;
    return node;
}

inline const char *logger_name()
{
    return "camera_capturer";
}

inline void format_log_message(char *buffer, std::size_t size, const char *fmt, va_list args)
{
    vsnprintf(buffer, size, fmt, args);
}

inline void ros_log_info(const char *fmt, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    format_log_message(buffer, sizeof(buffer), fmt, args);
    va_end(args);
#ifdef USE_ROS1
    ROS_INFO("%s", buffer);
#elif defined(USE_ROS2)
    RCLCPP_INFO(rclcpp::get_logger(logger_name()), "%s", buffer);
#endif
}

inline void ros_log_warn(const char *fmt, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    format_log_message(buffer, sizeof(buffer), fmt, args);
    va_end(args);
#ifdef USE_ROS1
    ROS_WARN("%s", buffer);
#elif defined(USE_ROS2)
    RCLCPP_WARN(rclcpp::get_logger(logger_name()), "%s", buffer);
#endif
}

inline void ros_log_error(const char *fmt, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    format_log_message(buffer, sizeof(buffer), fmt, args);
    va_end(args);
#ifdef USE_ROS1
    ROS_ERROR("%s", buffer);
#elif defined(USE_ROS2)
    RCLCPP_ERROR(rclcpp::get_logger(logger_name()), "%s", buffer);
#endif
}

#ifdef USE_ROS1
inline void ros_init(int argc, char **argv, const std::string &node_name)
{
    if (!ros::isInitialized()) {
        ros::init(argc, argv, node_name);
    }
    auto &node = node_instance();
    if (!node) {
        node = std::make_shared<ros::NodeHandle>();
    }
}
#elif defined(USE_ROS2)
inline void ros_init(int argc, char **argv, const std::string &node_name)
{
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }
    auto &node = node_instance();
    if (!node) {
        node = rclcpp::Node::make_shared(node_name);
    }
}
#endif

inline NodePtr &node()
{
    auto &instance = node_instance();
    if (!instance) {
#ifdef USE_ROS1
        instance = std::make_shared<ros::NodeHandle>();
#elif defined(USE_ROS2)
        instance = rclcpp::Node::make_shared(logger_name());
#endif
    }
    return instance;
}

inline bool ok()
{
#ifdef USE_ROS1
    return ros::ok();
#elif defined(USE_ROS2)
    return rclcpp::ok();
#endif
}

inline void shutdown()
{
#ifdef USE_ROS1
    if (ros::isStarted()) {
        ros::shutdown();
    }
#elif defined(USE_ROS2)
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
#endif
}

inline void spin_once()
{
#ifdef USE_ROS1
    ros::spinOnce();
#elif defined(USE_ROS2)
    auto &node = node_instance();
    if (node) {
        rclcpp::spin_some(node);
    }
#endif
}

#ifdef USE_ROS1
inline Time make_time_sec_nsec(long sec, long nsec)
{
    return Time(sec, nsec);
}

inline Time make_time_sec_usec(long sec, long usec)
{
    return Time(sec, usec * 1000L);
}

inline Time make_time_ns(uint64_t ns)
{
    Time stamp;
    stamp.fromNSec(ns);
    return stamp;
}
#elif defined(USE_ROS2)
inline Time make_time_sec_nsec(long sec, long nsec)
{
    return Time(sec, nsec, RCL_SYSTEM_TIME);
}

inline Time make_time_sec_usec(long sec, long usec)
{
    return Time(sec, usec * 1000L, RCL_SYSTEM_TIME);
}

inline Time make_time_ns(uint64_t ns)
{
    return Time(static_cast<int64_t>(ns), RCL_SYSTEM_TIME);
}
#endif

template <typename MsgT>
inline Publisher<MsgT> advertise(const std::string &topic, uint32_t queue_size)
{
#ifdef USE_ROS1
    return node()->advertise<MsgT>(topic, queue_size);
#elif defined(USE_ROS2)
    return node()->create_publisher<MsgT>(topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));
#endif
}

template <typename MsgT, typename Callback>
inline Subscription<MsgT> subscribe(const std::string &topic, uint32_t queue_size, Callback cb)
{
#ifdef USE_ROS1
    return node()->subscribe<MsgT>(topic, queue_size, cb);
#elif defined(USE_ROS2)
    return node()->create_subscription<MsgT>(topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)), cb);
#endif
}

#ifdef USE_ROS1
template <typename T>
inline T get_param(const std::string &name, const T &default_value)
{
    T value = default_value;
    node()->param<T>(name, value, default_value);
    return value;
}
#elif defined(USE_ROS2)
template <typename T>
inline T get_param(const std::string &name, const T &default_value)
{
    if (!node()->has_parameter(name)) {
        node()->declare_parameter<T>(name, default_value);
    }

    T value = default_value;
    node()->get_parameter_or<T>(name, value, default_value);
    return value;
}
#endif

#ifdef USE_ROS1
template <typename PubT, typename MsgT>
inline void publish(const PubT &pub, const MsgT &msg)
{
    pub.publish(msg);
}
#elif defined(USE_ROS2)
template <typename PubT, typename MsgT>
inline void publish(const PubT &pub, const MsgT &msg)
{
    if (pub) {
        pub->publish(msg);
    }
}
#endif

template <typename PubT>
inline void publish_image(const PubT &pub,
                          const cv::Mat &image,
                          const std::string &encoding,
                          const std::string &frame_id,
                          const Time &stamp)
{
    ImageMsg msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.height = static_cast<uint32_t>(image.rows);
    msg.width = static_cast<uint32_t>(image.cols);
    msg.encoding = encoding;
    msg.is_bigendian = false;

    const std::size_t row_bytes = static_cast<std::size_t>(image.cols) * image.elemSize();
    msg.step = static_cast<decltype(msg.step)>(row_bytes);
    msg.data.resize(row_bytes * static_cast<std::size_t>(image.rows));

    if (image.isContinuous() && image.step == row_bytes) {
        std::memcpy(msg.data.data(), image.data, msg.data.size());
    } else {
        for (int row = 0; row < image.rows; ++row) {
            std::memcpy(msg.data.data() + row_bytes * static_cast<std::size_t>(row),
                        image.ptr(row),
                        row_bytes);
        }
    }

    publish(pub, msg);
}

template <typename CovarianceArray>
inline void fill_covariance(CovarianceArray &covariance, double first_value)
{
    std::fill(std::begin(covariance), std::end(covariance), 0.0);
    covariance[0] = first_value;
}

template <typename PubT>
inline void publish_accel_measurement(const PubT &pub,
                                      const std::string &frame_id,
                                      const Time &stamp,
                                      double x,
                                      double y,
                                      double z)
{
    ImuMsg msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    fill_covariance(msg.orientation_covariance, -1.0);
    msg.linear_acceleration.x = x;
    msg.linear_acceleration.y = y;
    msg.linear_acceleration.z = z;
    fill_covariance(msg.linear_acceleration_covariance, 0.0);
    fill_covariance(msg.angular_velocity_covariance, -1.0);

    publish(pub, msg);
}

template <typename PubT>
inline void publish_gyro_measurement(const PubT &pub,
                                     const std::string &frame_id,
                                     const Time &stamp,
                                     double x,
                                     double y,
                                     double z)
{
    ImuMsg msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    fill_covariance(msg.orientation_covariance, -1.0);
    msg.angular_velocity.x = x;
    msg.angular_velocity.y = y;
    msg.angular_velocity.z = z;
    fill_covariance(msg.angular_velocity_covariance, 0.0);
    fill_covariance(msg.linear_acceleration_covariance, -1.0);

    publish(pub, msg);
}
