#include "writer/realsense_writer.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#include "utils/common_utils.h"

namespace {

std::int64_t effective_stamp_ns(const StampedRealSenseFrame& frame)
{
    if (frame.trigger_unix_ns != 0) {
        return frame.trigger_unix_ns;
    }
    return static_cast<std::int64_t>(frame.host_sec) * 1000000000LL
         + static_cast<std::int64_t>(frame.host_nanosec);
}

std::string format_ns(std::int64_t ns)
{
    const auto sec = ns / 1000000000LL;
    const auto nsec = ns % 1000000000LL;
    std::ostringstream oss;
    oss << sec << "." << std::setw(9) << std::setfill('0') << nsec;
    return oss.str();
}

}  // namespace

RealSenseWriter::RealSenseWriter(std::string output_dir)
    : output_dir_(std::move(output_dir))
{
}

RealSenseWriter::~RealSenseWriter()
{
    close();
}

bool RealSenseWriter::open()
{
    try {
        std::filesystem::create_directories(output_dir_ + "/realsense/rgb");
        std::filesystem::create_directories(output_dir_ + "/realsense/depth_raw");
        std::filesystem::create_directories(output_dir_ + "/realsense/imu");
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Failed to create realsense output dirs: " << e.what() << std::endl;
        return false;
    }

    time_stream_.open(output_dir_ + "/realsense/times.csv", std::ios::out);
    accel_stream_.open(output_dir_ + "/realsense/imu/accel.csv", std::ios::out);
    gyro_stream_.open(output_dir_ + "/realsense/imu/gyro.csv", std::ios::out);
    if (!time_stream_.is_open() || !accel_stream_.is_open() || !gyro_stream_.is_open()) {
        return false;
    }

    time_stream_ << "stamp_time,host_time\n";
    accel_stream_ << "host_sec,sensor_sec,ax,ay,az\n";
    gyro_stream_ << "host_sec,sensor_sec,gx,gy,gz\n";
    return true;
}

void RealSenseWriter::close()
{
    if (time_stream_.is_open()) time_stream_.close();
    if (accel_stream_.is_open()) accel_stream_.close();
    if (gyro_stream_.is_open()) gyro_stream_.close();
}

void RealSenseWriter::write_rgbd(const StampedRealSenseFrame& frame)
{
    if (!time_stream_.is_open()) {
        return;
    }

    const std::string host_time = format_timestamp_sec_nsec(frame.host_sec, frame.host_nanosec);
    const std::string stamp_time = format_ns(effective_stamp_ns(frame));

    time_stream_ << stamp_time << "," << host_time << std::endl;

    std::ostringstream ss;
    ss << output_dir_ << "/realsense/rgb/" << stamp_time << ".png";
    cv::imwrite(ss.str(), frame.color_image);

    ss.str("");
    ss.clear();
    ss << output_dir_ << "/realsense/depth_raw/" << stamp_time << ".png";
    cv::imwrite(ss.str(), frame.depth_image_raw);
}

void RealSenseWriter::write_imu(const StampedImuFrame& frame)
{
    std::ostream* out = nullptr;
    if (frame.stream_type == RS2_STREAM_ACCEL) {
        out = &accel_stream_;
    } else if (frame.stream_type == RS2_STREAM_GYRO) {
        out = &gyro_stream_;
    }
    if (!out || !out->good()) {
        return;
    }

    (*out) << std::fixed << std::setprecision(9)
           << to_sec_from_ns(frame.host_ns) << ","
           << to_sec_from_ns(frame.sensor_ns) << ","
           << std::fixed << std::setprecision(6)
           << frame.x << "," << frame.y << "," << frame.z << "\n";
}

void RealSenseWriter::write_intrinsics(const rs2::pipeline_profile& profile)
{
    RealSenseProducer::save_intrinsics(profile, output_dir_);
}

void RealSenseWriter::write_depth_scale(double scale)
{
    RealSenseProducer::save_depth_scale(scale, output_dir_);
}
