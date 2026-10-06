#include "writer/realsense_writer.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#include "utils/common_utils.h"

RealSenseWriter::RealSenseWriter(std::string output_dir, bool save_images)
    : output_dir_(std::move(output_dir)),
      save_images_(save_images)
{
}

RealSenseWriter::~RealSenseWriter()
{
    close();
}

bool RealSenseWriter::open()
{
    try {
        std::filesystem::create_directories(output_dir_ + "/realsense");
        if (save_images_) {
            std::filesystem::create_directories(output_dir_ + "/realsense/rgb");
            std::filesystem::create_directories(output_dir_ + "/realsense/depth_raw");
        }
        std::filesystem::create_directories(output_dir_ + "/realsense/imu");
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Failed to create realsense output dirs: " << e.what() << std::endl;
        return false;
    }

    time_stream_.open(output_dir_ + "/realsense/times.csv", std::ios::out);
    accel_stream_.open(output_dir_ + "/realsense/imu/accel.csv", std::ios::out);
    gyro_stream_.open(output_dir_ + "/realsense/imu/gyro.csv", std::ios::out);
    if (!time_stream_.is_open() || !accel_stream_.is_open() || !gyro_stream_.is_open()) {
        std::cerr << "Failed to open realsense output files under "
                  << std::filesystem::absolute(output_dir_ + "/realsense").string()
                  << " (times=" << time_stream_.is_open()
                  << ", accel=" << accel_stream_.is_open()
                  << ", gyro=" << gyro_stream_.is_open()
                  << ")" << std::endl;
        return false;
    }

    std::cout << "RealSense output dir: "
              << std::filesystem::absolute(output_dir_ + "/realsense").string()
              << " (save_images=" << save_images_ << ")" << std::endl;
    time_stream_ << "color_frame_number,color_sensor_time,color_host_time,"
                    "depth_frame_number,depth_sensor_time,depth_host_time\n";
    accel_stream_ << "host_time,sensor_time,ax,ay,az,trigger_time\n";
    gyro_stream_ << "host_time,sensor_time,gx,gy,gz,trigger_time\n";
    time_stream_.flush();
    accel_stream_.flush();
    gyro_stream_.flush();
    return time_stream_.good() && accel_stream_.good() && gyro_stream_.good();
}

void RealSenseWriter::close()
{
    if (time_stream_.is_open()) time_stream_.close();
    if (accel_stream_.is_open()) accel_stream_.close();
    if (gyro_stream_.is_open()) gyro_stream_.close();
}

bool RealSenseWriter::write_rgbd(const StampedRealSenseFrame& frame)
{
    if (!time_stream_.is_open()) {
        return false;
    }

    const auto color_sensor_ns = to_ns_from_sec_usec(
        frame.color_sensor_sec,
        frame.color_sensor_microsec);
    const auto depth_sensor_ns = frame.has_depth
        ? to_ns_from_sec_usec(frame.depth_sensor_sec, frame.depth_sensor_microsec)
        : 0;
    const auto color_stamp_ns = frame.trigger_unix_ns != 0 ? frame.trigger_unix_ns : color_sensor_ns;
    const auto depth_stamp_ns = frame.trigger_unix_ns != 0 ? frame.trigger_unix_ns : depth_sensor_ns;
    const std::string color_sensor_time = format_timestamp_ns(color_sensor_ns);
    const std::string depth_sensor_time = frame.has_depth
        ? format_timestamp_ns(depth_sensor_ns)
        : std::string{};
    const std::string color_stamp_time = format_timestamp_ns(color_stamp_ns);
    const std::string depth_stamp_time = format_timestamp_ns(depth_stamp_ns);
    const std::string color_host_time = format_timestamp_sec_nsec(
        frame.color_host_sec,
        frame.color_host_nanosec);
    const std::string depth_host_time = frame.has_depth
        ? format_timestamp_sec_nsec(frame.depth_host_sec, frame.depth_host_nanosec)
        : std::string{};

    time_stream_ << frame.color_frame_number << ","
                 << color_sensor_time << ","
                 << color_host_time << ","
                 << (frame.has_depth ? std::to_string(frame.depth_frame_number) : std::string{}) << ","
                 << depth_sensor_time << ","
                 << depth_host_time << '\n';

    if (save_images_) {
        std::ostringstream ss;
        ss << output_dir_ << "/realsense/rgb/" << color_stamp_time << ".png";
        if (!cv::imwrite(ss.str(), frame.color_image)) return false;

        if (!frame.depth_image_raw.empty()) {
            ss.str("");
            ss.clear();
            ss << output_dir_ << "/realsense/depth_raw/" << depth_stamp_time << ".png";
            if (!cv::imwrite(ss.str(), frame.depth_image_raw)) return false;
        }
    }
    time_stream_.flush();
    return time_stream_.good();
}

bool RealSenseWriter::write_imu(const StampedImuFrame& frame,
                               std::optional<std::int64_t> trigger_ns)
{
    std::ostream* out = nullptr;
    if (frame.stream_type == RS2_STREAM_ACCEL) {
        out = &accel_stream_;
    } else if (frame.stream_type == RS2_STREAM_GYRO) {
        out = &gyro_stream_;
    }
    if (!out || !out->good()) {
        return false;
    }

    (*out) << format_timestamp_ns(frame.host_ns) << ","
           << format_timestamp_ns(frame.sensor_ns) << ","
           << std::fixed << std::setprecision(6)
           << frame.x << "," << frame.y << "," << frame.z << ",";
    if (trigger_ns) {
        (*out) << format_timestamp_ns(*trigger_ns);
    }
    (*out) << '\n';
    out->flush();
    return out->good();
}

bool RealSenseWriter::write_intrinsics(const rs2::pipeline_profile& profile)
{
    return RealSenseProducer::save_intrinsics(profile, output_dir_);
}

bool RealSenseWriter::write_depth_scale(double scale)
{
    return RealSenseProducer::save_depth_scale(scale, output_dir_);
}
