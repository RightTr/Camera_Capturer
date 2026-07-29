#pragma once

#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include "producer/realsense_producer.h"

class RealSenseWriter {
public:
    explicit RealSenseWriter(std::string output_dir, bool save_images = true);
    ~RealSenseWriter();

    bool open();
    void close();
    void write_rgbd(const StampedRealSenseFrame& frame);
    void write_imu(const StampedImuFrame& frame);
    void write_intrinsics(const rs2::pipeline_profile& profile);
    void write_depth_scale(double scale);

private:
    void imu_loop();

    std::string output_dir_;
    bool save_images_;
    std::ofstream time_stream_;
    std::ofstream accel_stream_;
    std::ofstream gyro_stream_;
    std::mutex imu_mutex_;
    std::condition_variable imu_cv_;
    std::deque<StampedImuFrame> imu_queue_;
    std::thread imu_thread_;
    bool imu_stop_ = false;
};
