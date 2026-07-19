#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <string>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

struct StampedRealSenseFrame {
    cv::Mat color_image;
    cv::Mat depth_image_raw;
    long host_sec;
    long host_nanosec;
    long sensor_sec;
    long sensor_microsec;
    std::int64_t trigger_unix_ns = 0;
};

struct StampedImuFrame {
    rs2_stream stream_type;
    uint64_t host_ns;
    uint64_t sensor_ns;
    float x, y, z;
};

class RealSenseProducer {
public:
    RealSenseProducer(
        std::string dev,
        std::function<bool()> running,
        std::function<void()> fail = {},
        std::function<void(const rs2::pipeline_profile&)> on_start = {},
        std::function<void(double)> on_scale = {});
    ~RealSenseProducer();

    static uint64_t host_time_ns_now();
    static void save_intrinsics(const rs2::pipeline_profile& profile, const std::string& output_dir);
    static void save_depth_scale(double scale, const std::string& output_dir);

    void set_sync_mode(int sync_mode);
    void set_camera_fps(int camera_fps);
    void set_imu_enabled(bool imu);
    void set_imu_fps(int imu_fps);
    void set_align_enabled(bool align);
    void set_filter_enabled(bool filter);
    void set_rgb_queue_size(int rgb_max);
    void set_imu_queue_size(int imu_max);

    void run();
    bool pop_rgbd(StampedRealSenseFrame& frame);
    bool pop_imu(StampedImuFrame& frame);
    void stop();

private:
    static bool configure_sync(rs2::depth_sensor& depth_sensor, int sync_mode);
    bool live() const;
    bool push_rgbd(StampedRealSenseFrame&& frame);
    bool push_imu(StampedImuFrame&& frame);

    std::string dev_;
    int sync_mode_ = 0;
    int camera_fps_ = 30;
    bool imu_ = true;
    int imu_fps_ = 200;
    bool align_ = true;
    bool filter_ = true;
    int rgb_max_ = 30;
    int imu_max_ = 200;
    std::function<bool()> running_;
    std::function<void()> fail_;
    std::function<void(const rs2::pipeline_profile&)> on_start_;
    std::function<void(double)> on_scale_;

    mutable std::mutex rgb_mutex_;
    mutable std::mutex imu_mutex_;
    std::condition_variable rgb_cv_;
    std::condition_variable imu_cv_;
    std::queue<StampedRealSenseFrame> rgb_queue_;
    std::queue<StampedImuFrame> imu_queue_;
    std::atomic<bool> stopped_{false};
};
