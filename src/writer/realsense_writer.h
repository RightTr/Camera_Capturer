#pragma once

#include <fstream>
#include <optional>
#include <string>

#include "producer/realsense_producer.h"

class RealSenseWriter {
public:
    explicit RealSenseWriter(std::string output_dir, bool save_images = true);
    ~RealSenseWriter();

    bool open();
    void close();
    bool write_rgbd(const StampedRealSenseFrame& frame);
    bool write_imu(const StampedImuFrame& frame,
                   std::optional<std::int64_t> trigger_ns = std::nullopt);
    bool write_intrinsics(const rs2::pipeline_profile& profile);
    bool write_depth_scale(double scale);

private:
    std::string output_dir_;
    bool save_images_;
    std::ofstream time_stream_;
    std::ofstream accel_stream_;
    std::ofstream gyro_stream_;
};
