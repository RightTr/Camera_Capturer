#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <string>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

struct StampedRealSenseFrame {
    rs2::frameset frameset;
    rs2::frame color_frame;
    rs2::frame depth_frame;
    cv::Mat color_image;
    cv::Mat depth_image_raw;
    std::uint64_t color_frame_number = 0;
    std::uint64_t depth_frame_number = 0;
    bool has_depth = false;
    std::uint32_t trigger_step = 1;
    long color_host_sec = 0;
    long color_host_nanosec = 0;
    long color_sensor_sec = 0;
    long color_sensor_microsec = 0;
    long depth_host_sec = 0;
    long depth_host_nanosec = 0;
    long depth_sensor_sec = 0;
    long depth_sensor_microsec = 0;
    std::int64_t trigger_unix_ns = 0;
    float temperature_celsius = std::numeric_limits<float>::quiet_NaN();
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
    void set_imu_csv_enabled(bool enabled);
    void set_align_enabled(bool align);
    void set_filter_enabled(bool filter);
    void set_depth_stream_enabled(bool enabled);
    void set_depth_processing_enabled(bool enabled);
    void set_rgbd_queue_size(int rgbd_max);
    void set_imu_queue_size(int imu_max);
    void reset_rgbd_tracking();

    void run();
    bool pop_rgbd(StampedRealSenseFrame& frame);
    bool process_rgbd(StampedRealSenseFrame& frame);
    bool pop_accel(StampedImuFrame& frame);
    bool pop_gyro(StampedImuFrame& frame);
    bool pop_imu_csv(StampedImuFrame& frame);
    void clear_rgbd();
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
    bool imu_csv_ = false;
    int imu_fps_ = 200;
    bool align_ = true;
    bool filter_ = true;
    bool depth_stream_enabled_ = true;
    bool depth_processing_enabled_ = true;
    int rgbd_max_ = 30;
    int imu_max_ = 200;
    std::function<bool()> running_;
    std::function<void()> fail_;
    std::function<void(const rs2::pipeline_profile&)> on_start_;
    std::function<void(double)> on_scale_;

    mutable std::mutex rgb_mutex_;
    mutable std::mutex rgb_state_mutex_;
    mutable std::mutex accel_mutex_;
    mutable std::mutex gyro_mutex_;
    mutable std::mutex imu_save_mutex_;
    std::condition_variable rgb_cv_;
    std::condition_variable accel_cv_;
    std::condition_variable gyro_cv_;
    std::condition_variable imu_save_cv_;
    std::queue<StampedRealSenseFrame> rgbd_queue_;
    std::queue<StampedImuFrame> accel_queue_;
    std::queue<StampedImuFrame> gyro_queue_;
    std::queue<StampedImuFrame> imu_save_queue_;
    std::atomic<bool> stopped_{false};
    std::uint64_t last_color_frame_number_ = 0;
    std::uint64_t last_depth_frame_number_ = 0;
    bool rgbd_tracking_initialized_ = false;
    rs2::align align_to_color_{RS2_STREAM_COLOR};
    rs2::spatial_filter spatial_filter_;
    rs2::temporal_filter temporal_filter_;
    std::atomic<std::uint64_t> processed_count_{0};
    std::atomic<std::uint64_t> depth_processed_count_{0};
    std::atomic<std::uint64_t> depth_skipped_count_{0};
    std::atomic<std::uint64_t> align_ns_{0};
    std::atomic<std::uint64_t> filter_ns_{0};
};
