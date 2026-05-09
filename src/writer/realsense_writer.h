#pragma once

#include <fstream>
#include <string>

#include "producer/realsense_producer.h"

class RealSenseWriter {
public:
    explicit RealSenseWriter(std::string output_dir);
    ~RealSenseWriter();

    bool open();
    void close();
    void write_rgbd(const StampedRealSenseFrame& frame);
    void write_imu(const StampedImuFrame& frame);
    void write_intrinsics(const rs2::pipeline_profile& profile);
    void write_depth_scale(double scale);

private:
    std::string output_dir_;
    std::ofstream time_stream_;
    std::ofstream accel_stream_;
    std::ofstream gyro_stream_;
};
